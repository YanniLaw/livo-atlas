"""Evaluate Gaussian submaps on held-out views of a finalized SLAM snapshot."""
import argparse
from dataclasses import dataclass
import hashlib
import importlib.metadata
import math
from pathlib import Path
import resource
import subprocess
import sys
from tempfile import TemporaryDirectory
import time

import numpy as np

from bundle import load_bundle
from gaussian_map import (_inspect_gaussian_map, _json, _project, _transform, _yaml,
                          inspect_gaussian_finalization)
from gaussian_state import _unsigned
from joint_renderer import _ALPHA_THRESHOLD, _check_frontier, _real_directory, infer_blocks
from render_gaussian_map import _exclusive_write, _json_bytes, _new_output, _png, _render_report


_MAX_FRAMES = 512
_MAX_PRIMARY = 32
_MAX_PIXELS = 1_000_000
_MAX_CLOUD_POINTS = 500_000
_MAX_REPORT_BYTES = 2 * 1024 * 1024
_SEED_POLICIES = ('uniform', 'coverage')
_DEPTH_SAMPLE_DTYPE = np.dtype([('pixel_index', '<u4'), ('observed_z_m', '<f4'),
                               ('single_z_m', '<f4'), ('joint_z_m', '<f4'),
                               ('single_covered', '?'), ('joint_covered', '?')])


@dataclass(frozen=True)
class TrainedBlock:
    submap_id: int
    parent_from_submap: np.ndarray
    parameters: dict[str, np.ndarray]


def split_primary_ids(first: int, count: int) -> tuple[list[int], list[int]]:
    if (type(first) is not int or type(count) is not int or first < 0 or
            not 3 <= count <= _MAX_PRIMARY or first + count > _MAX_FRAMES):
        raise ValueError(f'Primary group at frame {first} requires 3 to 32 covered frames')
    held_out = [first + 1]
    if count >= 6:
        held_out.append(first + count // 2)
    return [key for key in range(first, first + count) if key not in held_out], held_out


def camera_from_final_parent(view: dict, final_parent_from_body: np.ndarray) -> np.ndarray:
    # The exporter projects depth in the fixed body frame. Only the pose changes.
    camera_from_body = view['camera_from_parent'] @ view['parent_from_body']
    return camera_from_body @ np.linalg.inv(final_parent_from_body)


def file_sha256(path):
    if path.is_symlink() or not path.is_file():
        raise ValueError(f'Expected a regular input file at {path}')
    digest = hashlib.sha256()
    with path.open('rb') as source:
        for block in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def check_split_identity(observations):
    for field in ('keyframe_id', 'capture_time', 'image_sha256'):
        seen = {}
        for observation in observations:
            previous = seen.setdefault(observation[field], observation)
            if previous['split'] != observation['split']:
                raise ValueError(f"Training and held-out keyframes {previous['keyframe_id']} "
                                 f"and {observation['keyframe_id']} share {field}")


def _group_splits(group):
    return {key: split for split, ids in (('train', group['train_ids']),
                                         ('validation', group['held_out_ids'])) for key in ids}


def freeze_split(directory: Path, keyframes: Path, prepare: Path) -> tuple[dict, list]:
    map_hashes = {name: file_sha256(directory / name)
                  for name in ('map.json', 'status.json', 'finalization.json')}
    expected = _json(directory / 'map.json')
    version = _unsigned(expected['pose_version'])
    frames = _unsigned(expected['frame_count'])
    if not 1 <= frames <= _MAX_FRAMES:
        raise ValueError('Expected 1 to 512 frames in the finalized dataset')
    _check_frontier(keyframes, frames, version)
    records = keyframes / 'online_submaps/records'
    _real_directory(keyframes / 'online_submaps')
    _real_directory(records)
    summary, document, poses, _ = _inspect_gaussian_map(directory, keyframes, version, frames)
    if document != expected:
        raise ValueError('Gaussian map changed during input validation')
    inspect_gaussian_finalization(directory, summary)

    paths = [Path('dataset.yml')]
    if version:
        paths.append(Path('online_submaps/poses') / f'{version}.yml')
    groups, observations = [], []
    for row in document['entries']:
        submap_id = _unsigned(row['submap_id'])
        anchor = _unsigned(row['anchor_keyframe_id'])
        revision = _unsigned(row['membership_revision'])
        train, held = split_primary_ids(anchor, row['primary_count'])
        _real_directory(records / str(submap_id))
        paths.append(Path('online_submaps/records') / str(submap_id) / f'{revision}.yml')
        group = dict(submap_id=submap_id, anchor_id=anchor, membership_revision=revision,
                     frame_count=frames, train_ids=train, held_out_ids=held)
        groups.append(group)
        for key, split in sorted(_group_splits(group).items()):
            frame = Path('frames') / str(key)
            metadata = _yaml(keyframes / frame / 'frame.yml')
            if metadata['has_image'] != 1:
                raise ValueError(f'Keyframe {key} has no image')
            capture_time = float(metadata['image_timestamp'])
            if not math.isfinite(capture_time):
                raise ValueError(f'Keyframe {key} has an invalid capture time')
            if _unsigned(metadata['cloud_points']) > _MAX_CLOUD_POINTS:
                raise ValueError(f'Keyframe {key} exceeds the point budget')
            paths.extend(frame / name for name in ('frame.yml', 'image.png', 'cloud.pcd'))
            observations.append(dict(keyframe_id=key, split=split, capture_time=capture_time))

    frame_hashes = {str(path): file_sha256(keyframes / path) for path in sorted(paths)}
    for row in observations:
        row['image_sha256'] = frame_hashes[f"frames/{row['keyframe_id']}/image.png"]
    check_split_identity(observations)
    split = dict(schema=1, session_id=document['session_id'], pose_version=version,
                 parent_frame=document['parent_frame'], frame_count=frames,
                 optimized_frame_count=_unsigned(document['optimized_frame_count']),
                 map_sequence=summary['map_sequence'], groups=groups, observations=observations,
                 keyframe_sha256=frame_hashes, gaussian_sha256=map_hashes,
                 prepare_executable_sha256=file_sha256(prepare))
    verify_split_sources(split, directory, keyframes, prepare)
    return split, poses


def verify_split_sources(split, directory, keyframes, prepare):
    for root, hashes in ((directory, split['gaussian_sha256']), (keyframes, split['keyframe_sha256'])):
        for name, digest in hashes.items():
            if file_sha256(root / name) != digest:
                raise ValueError(f'Input changed during evaluation at {root / name}')
    if file_sha256(prepare) != split['prepare_executable_sha256']:
        raise ValueError('Preparation executable changed during evaluation')


def read_prepared_group(keyframes, output, group, source_sha256):
    manifest = _json(output / 'bundle.json')
    entries = manifest.get('views')
    expected = _group_splits(group)
    if (manifest.get('source_manifest_sha256') != source_sha256 or
            not isinstance(entries, list) or len(entries) != len(expected) or
            len(entries) > _MAX_PRIMARY):
        raise ValueError(f"Prepared bundle differs from submap {group['submap_id']} split")
    assets = (('image', 'image.png'), ('image_mask', 'image_mask.png'),
              ('depth_mask', 'depth_mask.png'), ('depth_m', 'depth.f32le'))
    _real_directory(output / 'views')
    seen = set()
    for entry in entries:
        key = _unsigned(entry['keyframe_id'])
        width, height = entry['width'], entry['height']
        if (key in seen or key not in expected or entry['split'] != expected[key] or
                _unsigned(entry['frame_count']) != group['frame_count'] or
                entry['pose_version'] != '0' or 'parent_from_body' not in entry or
                type(width) is not int or type(height) is not int or
                width <= 0 or height <= 0 or width * height > _MAX_PIXELS):
            raise ValueError(f'Prepared view {key} differs from the frozen split or raw pose frontier')
        seen.add(key)
        relative = Path('views') / str(key)
        _real_directory(output / relative)
        for field, filename in assets:
            asset = relative / filename
            path = output / asset
            if entry[field] != asset.as_posix() or path.is_symlink() or not path.is_file():
                raise ValueError(f'Prepared view {key} has an invalid {field} asset path')

    views = {view['keyframe_id']: view for view in load_bundle(output)}
    for key, view in views.items():
        raw = _yaml(keyframes / 'frames' / str(key) / 'frame.yml')
        if (view['image_timestamp'] != float(raw['image_timestamp']) or
                view['calibration_id'] != _unsigned(raw['calibration_id']) or
                not np.allclose(view['parent_from_body'], _transform(raw['odom_from_body']),
                                atol=1e-12, rtol=0)):
            raise ValueError(f'Prepared view {key} does not match its source observation')
        # In raw odom the body transforms cancel. The C++ exporter first
        # projects the recorded rotations back onto SO(3).
        raw_camera = _project(_transform(raw['camera_from_odom']))
        if not np.allclose(view['camera_from_parent'], raw_camera, atol=1e-9, rtol=1e-12):
            raise ValueError(f'Prepared camera pose does not match raw keyframe {key}')
        if not view['image_mask'].any() or (view['split'] == 'train' and not view['depth_mask'].any()):
            raise ValueError(f'Keyframe {key} lacks usable image or training depth')
    return views


def prepare_group(prepare, keyframes, output, group, source_sha256):
    subprocess.run([str(prepare), '--dataset', str(keyframes), '--output', str(output),
                    '--train-ids', ','.join(map(str, group['train_ids'])),
                    '--validation-ids', ','.join(map(str, group['held_out_ids'])),
                    '--source-sha256', source_sha256, '--max-pixels', str(_MAX_PIXELS),
                    '--max-cloud-points', str(_MAX_CLOUD_POINTS)], check=True)
    return read_prepared_group(keyframes, output, group, source_sha256)


def train_group(group, train_views, poses, device, *, seed_policy='uniform'):
    train_ids = group['train_ids']
    if ([view['keyframe_id'] for view in train_views] != train_ids or
            any(view['split'] != 'train' for view in train_views) or
            set(train_ids).intersection(group['held_out_ids'])):
        raise ValueError(f"Observations differ from the training split of submap {group['submap_id']}")

    import torch
    from online_model import OnlineGaussianModel, _MAX_VIEWS

    started = time.perf_counter()
    torch.cuda.reset_peak_memory_stats(device)
    model = OnlineGaussianModel(device)
    model.reset(1)
    anchor_pose = poses[group['anchor_id']]
    training = []
    for index, view in enumerate(train_views):
        key = view['keyframe_id']
        camera = camera_from_final_parent(view, poses[key])
        local = dict(view, camera_from_parent=camera @ anchor_pose, generation='1')
        _, metrics = model.train_candidate(local, seed_policy=seed_policy)
        retained = train_ids[max(0, index + 1 - _MAX_VIEWS):index + 1]
        supervised = metrics['training_frame_ids']
        if (metrics['frame_ids'] != retained or metrics['steps'] != 20 or
                len(supervised) != metrics['steps'] or key not in supervised or
                set(supervised).difference(retained)):
            raise RuntimeError(f'Unexpected training provenance for keyframe {key}')
        model.decide(True)
        training.append(dict(metrics, keyframe_id=key))

    origin_ids, counts = np.unique(model.origin_ids.cpu().numpy(), return_counts=True)
    if origin_ids.tolist() != model.frame_ids:
        raise RuntimeError(f"Unexpected Gaussian origins for submap {group['submap_id']}")
    parameters = {name: value.detach().cpu().numpy().copy() for name, value in model.parameters.items()}
    torch.cuda.synchronize(device)
    evidence = dict(submap_id=group['submap_id'], anchor_id=group['anchor_id'],
                    parent_from_submap=anchor_pose.tolist(), training=training,
                    retained_frame_ids=model.frame_ids,
                    gaussian_origins={str(key): int(count) for key, count in zip(origin_ids, counts)},
                    training_seconds=time.perf_counter() - started,
                    gpu_peak_allocated_bytes=torch.cuda.max_memory_allocated(device),
                    gpu_peak_reserved_bytes=torch.cuda.max_memory_reserved(device))
    # The caller renders only the owned CPU arrays, after this model goes out of scope.
    return TrainedBlock(group['submap_id'], anchor_pose, parameters), evidence


def _record_depth(samples, result, mode):
    pixels = samples['pixel_index']
    covered = result['alpha'].ravel()[pixels] >= _ALPHA_THRESHOLD
    samples[f'{mode}_covered'] = covered
    samples[f'{mode}_z_m'][covered] = result['depth_m'].ravel()[pixels[covered]]


def render_query(output, query, camera_from_parent, current, previous, device, role,
                 *, save_depth_samples=False):
    query_id = query['keyframe_id']
    height, width = query['rgb'].shape[:2]
    views = output / 'views' / str(query_id)
    views.mkdir(parents=True)
    _png(views / 'reference.png', query['rgb'])
    samples = None
    if save_depth_samples:
        pixels = np.flatnonzero(query['image_mask'] & query['depth_mask'])
        samples = np.zeros(pixels.size, dtype=_DEPTH_SAMPLE_DTYPE)
        samples['pixel_index'] = pixels
        samples['observed_z_m'] = query['depth_m'].ravel()[pixels]
    own = (current.submap_id, current.parameters, camera_from_parent @ current.parent_from_submap)
    rendered = infer_blocks([own], query['K'], width, height, device)
    single = _render_report(rendered, query)
    if samples is not None:
        _record_depth(samples, rendered, 'single')
    _png(views / 'single.png', rendered['rgb'])
    del rendered
    joint = None
    if previous is not None:
        neighbor = (previous.submap_id, previous.parameters,
                    camera_from_parent @ previous.parent_from_submap)
        rendered = infer_blocks([neighbor, own], query['K'], width, height, device)
        joint = _render_report(rendered, query)
        if samples is not None:
            _record_depth(samples, rendered, 'joint')
        _png(views / 'joint.png', rendered['rgb'])
    report = dict(keyframe_id=query_id, submap_id=current.submap_id, role=role,
                  capture_time=query['image_timestamp'], calibration_id=query['calibration_id'],
                  width=width, height=height, K=query['K'].tolist(),
                  camera_from_parent=camera_from_parent.tolist(), single=single, joint=joint)
    if samples is not None:
        path = views / 'depth-samples.npy'
        with path.open('xb') as stream:
            np.save(stream, samples, allow_pickle=False)
        report.update(depth_samples=path.relative_to(output).as_posix(),
                      depth_samples_sha256=file_sha256(path))
    return report


def _aggregate_metrics(results):
    if not results:
        return None
    metrics = [result['metrics'] for result in results]
    image_pixels = sum(row['image_pixels'] for row in metrics)
    covered = sum(row['covered_pixels'] for row in metrics)
    depth_pixels = sum(row['depth_pixels_used'] for row in metrics)
    rgb_mse = sum(row['rgb_mse'] * row['image_pixels'] for row in metrics) / image_pixels
    rgb_l1 = sum(row['rgb_l1'] * row['image_pixels'] for row in metrics) / image_pixels
    depth_l1 = depth_rmse = None
    if depth_pixels:
        depth_l1 = sum(row['depth_l1_m'] * row['depth_pixels_used'] for row in metrics
                       if row['depth_pixels_used']) / depth_pixels
        depth_rmse = math.sqrt(sum(row['depth_rmse_m'] ** 2 * row['depth_pixels_used'] for row in metrics
                                   if row['depth_pixels_used']) / depth_pixels)
    return dict(image_pixels=image_pixels, covered_pixels=covered, alpha_coverage=covered / image_pixels,
                rgb_l1=rgb_l1, rgb_mse=rgb_mse, rgb_perfect=rgb_mse == 0,
                rgb_psnr_db=-10 * math.log10(rgb_mse) if rgb_mse else None,
                depth_pixels_available=sum(row['depth_pixels_available'] for row in metrics),
                depth_pixels_used=depth_pixels, depth_l1_m=depth_l1, depth_rmse_m=depth_rmse)


def summarize_views(queries):
    paired = [query for query in queries if query['joint'] is not None]
    selections = dict(paired=paired,
                      single_only=[query for query in queries if query['joint'] is None])
    for role in ('near_group_start', 'interior'):
        selections[role] = [query for query in paired if query['role'] == role]
    return {name: dict(query_count=len(rows),
                       single=_aggregate_metrics([query['single'] for query in rows]),
                       joint=_aggregate_metrics([query['joint'] for query in rows if query['joint'] is not None]))
            for name, rows in selections.items()}


def run(args) -> dict:
    started = time.perf_counter()
    seed_policy = getattr(args, 'seed_policy', 'uniform')
    save_depth_samples = getattr(args, 'save_depth_samples', False)
    if seed_policy not in _SEED_POLICIES:
        raise ValueError('Unknown Gaussian seed policy')
    directory = args.map_directory.resolve(strict=True)
    keyframes = args.keyframes.resolve(strict=True)
    prepare = args.prepare_executable.resolve(strict=True)
    output = _new_output(directory, keyframes, args.output)
    split, poses = freeze_split(directory, keyframes, prepare)
    validated = time.perf_counter()
    output.mkdir(mode=0o700)
    split_path = output / 'split.json'
    _exclusive_write(split_path, _json_bytes(split, _MAX_REPORT_BYTES))
    split_sha256 = file_sha256(split_path)

    import torch
    from online_model import checked_cuda_device

    torch.set_num_threads(1)
    device = checked_cuda_device()
    groups, queries = [], []
    previous = None
    for group in split['groups']:
        prepare_started = time.perf_counter()
        with TemporaryDirectory(prefix='.prepare-', dir=output) as stage:
            views = prepare_group(prepare, keyframes, Path(stage) / 'bundle', group, split_sha256)
        prepare_seconds = time.perf_counter() - prepare_started
        current, evidence = train_group(group, [views[key] for key in group['train_ids']], poses,
                                        device, seed_policy=seed_policy)
        evidence.update(prepare_seconds=prepare_seconds,
                        gpu_allocated_after_training_bytes=torch.cuda.memory_allocated(device))
        groups.append(evidence)
        for index, key in enumerate(group['held_out_ids']):
            camera = camera_from_final_parent(views[key], poses[key])
            role = 'near_group_start' if index == 0 else 'interior'
            queries.append(render_query(output, views[key], camera, current, previous, device, role,
                                        save_depth_samples=save_depth_samples))
        previous = current
        del views

    verify_split_sources(split, directory, keyframes, prepare)
    if file_sha256(split_path) != split_sha256:
        raise RuntimeError('Published split changed during evaluation')
    report = dict(schema=1, kind='livo_atlas.gaussian_holdout_evaluation', status='succeeded',
                  split_sha256=split_sha256, seed_policy=seed_policy, session_id=split['session_id'],
                  pose_version=split['pose_version'], frame_count=split['frame_count'],
                  optimized_frame_count=split['optimized_frame_count'], parent_frame=split['parent_frame'],
                  map_sequence=split['map_sequence'], submaps=groups, views=queries,
                  summary=summarize_views(queries),
                  environment=dict(python=sys.version.split()[0], numpy=np.__version__,
                                   torch=str(torch.__version__), cuda=torch.version.cuda,
                                   gsplat=importlib.metadata.version('gsplat'),
                                   device=torch.cuda.get_device_name(device)),
                  snapshot_validation_seconds=validated - started,
                  elapsed_seconds=time.perf_counter() - started,
                  peak_rss_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024)
    pending = output / 'report.json.pending'
    _exclusive_write(pending, _json_bytes(report, _MAX_REPORT_BYTES))
    pending.replace(output / 'report.json')
    return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('--keyframes', '--map-directory', '--prepare-executable', '--output'):
        parser.add_argument(name, type=Path, required=True)
    parser.add_argument('--seed-policy', choices=_SEED_POLICIES, default='uniform')
    parser.add_argument('--save-depth-samples', action='store_true',
                        help='Save covered and observed sparse depth for paired comparisons')
    args = parser.parse_args(argv)
    try:
        run(args)
    except Exception as error:
        print(f'{type(error).__name__}: {error}', file=sys.stderr)
        return 1
    print(f'Evaluation report: {args.output / "report.json"}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
