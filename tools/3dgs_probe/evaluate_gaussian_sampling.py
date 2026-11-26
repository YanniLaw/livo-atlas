"""Compare sampling policies with a frozen split, schedule and reference selection."""

import argparse
from dataclasses import asdict
import hashlib
from pathlib import Path
import random
import resource
import shutil
import sys
from tempfile import TemporaryDirectory
import time

import numpy as np

from evaluate_gaussian_submaps import prepare_group, train_group
from evaluate_heldout_snapshot import (_estimate, _save_samples, initialize_cuda,
                                       load_settings)
from evaluation_metrics import aggregate_metrics, measure
from gaussian_map import _project, _require
from gaussian_state import _open_file, _pose
from heldout_query import (canonical_query, load_evaluation_snapshot,
                           verify_evaluation_snapshot)
from heldout_selection import EvaluationSelector
from holdout_split import canonical_json
from joint_renderer import _real_directory, infer_blocks
from prepare_holdout_split import file_sha
from render_gaussian_map import _exclusive_write, _json_bytes, _new_output, _sources_report
from worker_config import load_worker_config


_REPORT_BYTES = 2 * 1024 * 1024


def frozen_groups(snapshot):
    """Keep every primary observation, including heldout frames not queried."""
    groups = []
    for row in snapshot.document['entries']:
        owner = int(row['submap_id'])
        frames = [(key, frame) for key, frame in snapshot.split.frames.items()
                  if int(frame['submap_id']) == owner]
        train = [key for key, frame in frames if frame['role'] == 'train']
        held = [key for key, frame in frames if frame['role'] == 'heldout']
        _require(train and 1 <= len(frames) <= 32, 'A controlled group requires training observations')
        groups.append(dict(submap_id=owner, anchor_id=int(row['anchor_keyframe_id']),
                           frame_count=len(snapshot.poses), train_ids=train, held_out_ids=held))
    return groups


def admit_training_view(split, view):
    """Verify original sensor values before canonicalizing the camera for training."""
    result = dict(view)
    for name in ('image_mask', 'depth_mask'):
        mask = np.asarray(result[name])
        _require(mask.dtype in (np.dtype('bool'), np.dtype('u1')) and np.all(mask <= 1),
                 'Invalid frozen training mask')
        result[name] = mask.astype(bool, copy=True)
    for name in ('rgb', 'depth_m'):
        result[name] = np.asarray(result[name], dtype='<f4').copy()
    if 'camera_from_body' not in result:
        result['camera_from_body'] = result['camera_from_parent'] @ _project(result['parent_from_body'])
    key = split.check_view(result)
    frame = split.frames[key]
    result['K'] = np.asarray(frame['intrinsics'], dtype='f8').reshape(3, 3).copy()
    result['camera_from_body'] = np.asarray(frame['camera_from_body'], dtype='f8').reshape(4, 4).copy()
    # train_group composes C_P * P_B. Project the recorded P_B exactly as the
    # native preparer/online path does, retaining the independently verified C_B.
    raw_body = result.get('parent_from_body')
    if raw_body is None:
        raw_body = np.linalg.inv(result['camera_from_parent']) @ result['camera_from_body']
    result['parent_from_body'] = _project(raw_body)
    result['camera_from_parent'] = result['camera_from_body'] @ np.linalg.inv(result['parent_from_body'])
    result['split'] = 'train'
    return result


def load_final_poses(root, report, frame_count):
    """Bind individual camera matrices to the complete recorded pose digest."""
    _require(type(frame_count) is int and 1 <= frame_count <= 512, 'Invalid final pose count')
    expected = frame_count * 16 * 8
    with _open_file(root / 'poses.f64le', expected) as stream:
        raw = stream.read()
    _require(len(raw) == expected and hashlib.sha256(raw).hexdigest() == report['final_poses_sha256'],
             'Final pose payload differs from recorded digest')
    poses = np.frombuffer(raw, dtype='<f8').reshape(frame_count, 4, 4)
    return np.asarray([_pose(pose) for pose in poses])


def normalized_training_options(report):
    """Supply only the known historical method default, preserving other types."""
    options = dict(report['training_options'])
    options.setdefault('coverage_sampling', 'resample')
    return options


def verify_configuration(root, report):
    """Bind an archived YAML to both its byte digest and effective optimizer."""
    import yaml

    _require(report.get('config_file') == 'config.yaml', 'Missing archived config identity')
    path = root / 'config.yaml'
    with _open_file(path, 256 * 1024) as stream:
        raw = stream.read()
    _require(hashlib.sha256(raw).hexdigest() == report['config_sha256'], 'Archived config digest differs')
    options = normalized_training_options(report)
    _require(canonical_json(asdict(load_worker_config(path).options)) == canonical_json(options),
             'Archived config differs from effective training options')
    _require(file_sha(path) == report['config_sha256'], 'Archived config changed during verification')
    return yaml.safe_load(raw)


def verify_artifacts(root, artifacts, queries):
    """Check the complete advertised sample/PNG set before certification."""
    expected = {}
    for row in queries:
        key = row['keyframe_id']
        width, height = row['query']['width'], row['query']['height']
        _require(type(width) is int and type(height) is int and width > 0 and height > 0
                 and width * height <= 1_000_000, 'Invalid artifact dimensions')
        pixels = width * height
        files = ['reference.png', 'single.png', 'samples.npz']
        if row['joint'] is not None:
            files.append('joint.png')
        for name in files:
            expected[f'views/{key}/{name}'] = pixels * (58 if name.endswith('.npz') else 3) + 65536
    _require(isinstance(artifacts, dict) and artifacts.keys() == expected.keys(),
             'Incomplete or unexpected artifact receipt set')
    _real_directory(root / 'views')
    for relative, maximum in expected.items():
        path = root / relative
        _real_directory(path.parent)
        receipt = artifacts[relative]
        _require(isinstance(receipt, dict) and receipt.keys() == {'bytes', 'sha256'}
                 and type(receipt['bytes']) is int, 'Invalid artifact receipt')
        with _open_file(path, maximum) as stream:
            raw = stream.read()
        _require(len(raw) == receipt['bytes'] and hashlib.sha256(raw).hexdigest() == receipt['sha256'],
                 'Output artifact differs from receipt')


def _render(snapshot, view, selected, blocks, device):
    query = canonical_query(snapshot.split, view)
    key = query['keyframe_id']
    owner = int(snapshot.split.frames[key]['submap_id'])
    camera = query['camera_from_body'] @ np.linalg.inv(snapshot.poses[key])
    height, width = query['rgb'].shape[:2]

    def render(ids):
        inputs = [(group, blocks[group].parameters, camera @ blocks[group].parent_from_submap)
                  for group in ids]
        return infer_blocks(inputs, query['K'], width, height, device)

    single = render([owner])
    joint = render(selected) if len(selected) == 2 else None
    report = dict(keyframe_id=key, owner_submap_id=owner, selected_ids=selected,
                  query=dict(width=width, height=height, K=query['K'].tolist(),
                             camera_from_parent=camera.ravel().tolist()),
                  single=measure(single, query), joint=measure(joint, query) if joint is not None else None)
    return dict(report=report, query=query, single=single, joint=joint)


def run(args):
    started = time.perf_counter()
    _require(args.seed_policy in ('uniform', 'coverage'), 'Invalid sampling policy')
    _require(type(args.random_seed) is int and 0 <= args.random_seed < 2**32, 'Invalid random seed')
    output = _new_output(args.map_directory, args.keyframes, args.output)
    settings = load_settings(args.config)
    with _open_file(args.config, 256 * 1024) as stream:
        configuration = stream.read()
    _require(hashlib.sha256(configuration).hexdigest() == settings.config_sha256,
             'Config changed during preflight')
    _require(settings.min_free_disk_bytes >= 1024**3, 'At least 1 GiB disk reserve is required')
    snapshot = load_evaluation_snapshot(args.map_directory, args.keyframes)
    if settings.split_file:
        from holdout_split import HoldoutSplit, MAX_SPLIT_BYTES
        with _open_file(Path(settings.split_file), MAX_SPLIT_BYTES) as stream:
            _require(HoldoutSplit(stream.read()).split_id == snapshot.split.split_id,
                     'Configured split differs from the reference snapshot')
    executable = args.prepare_executable.resolve(strict=True)
    executable_sha = file_sha(executable)
    selector = EvaluationSelector(snapshot, settings.policy,
        args.selector_executable or executable.with_name('livo_atlas_select_gaussian_neighbor'))
    groups = frozen_groups(snapshot)
    plan = []
    for key, frame in snapshot.split.frames.items():
        if frame['query']:
            selection = selector.select(frame)
            _require(selection['selected_ids'], 'Reference query has no current owner model')
            if selection['selection'] is not None:
                selection['selection'].pop('selection_seconds', None)
            plan.append(dict(keyframe_id=key, **selection))
    predicted, _ = _estimate(snapshot, settings.policy)
    # The preparation bundle contains up to 32 full observations at a time.
    scratch = max(sum(frame['width'] * frame['height'] * 32 + 128 * 1024
                      for frame in snapshot.split.frames.values()
                      if int(frame['submap_id']) == group['submap_id']) for group in groups)
    _require(predicted <= settings.max_output_bytes, 'Controlled output budget is insufficient')
    _require(shutil.disk_usage(output.parent).free >= predicted + scratch + settings.min_free_disk_bytes,
             'Controlled preparation disk reserve is insufficient')
    report = dict(schema=1, kind='livo_atlas.gaussian_sampling_evaluation', status='running',
        seed_policy=args.seed_policy, random_seed=args.random_seed, split_id=snapshot.split.split_id,
        selection_policy=settings.policy, selection_semantics='frozen_reference_online_models',
        selector_runtime_sha256=selector.runtime_sha256, selection_plan=plan,
        selection_plan_sha256=hashlib.sha256(canonical_json(plan)).hexdigest(),
        sources=_sources_report(snapshot), source_sha256=snapshot.source_digests,
        final_poses_sha256=hashlib.sha256(np.asarray(snapshot.poses, dtype='<f8').tobytes()).hexdigest(),
        config_sha256=settings.config_sha256, config_file='config.yaml', preparer_sha256=executable_sha,
        training_options=asdict(settings.worker), groups=groups, submaps=[], queries=[], artifacts={},
        training_calls=0, training_steps=0)
    output.mkdir(mode=0o700)
    try:
        _exclusive_write(output / 'split.json', snapshot.split.payload)
        _exclusive_write(output / 'config.yaml', configuration)
        _exclusive_write(output / 'poses.f64le', np.asarray(snapshot.poses, dtype='<f8').tobytes())
        random.seed(args.random_seed)
        np.random.seed(args.random_seed)
        device, report['software'] = initialize_cuda(settings.worker)
        import torch
        torch.manual_seed(args.random_seed)
        torch.cuda.manual_seed_all(args.random_seed)
        blocks = {}
        reference_held = next(key for key, frame in snapshot.split.frames.items() if frame['role'] == 'heldout')
        for group in groups:
            _require(shutil.disk_usage(output).free >= scratch + settings.min_free_disk_bytes,
                     'Controlled preparation disk reserve fell below budget')
            prepared_group = dict(group, held_out_ids=group['held_out_ids'] or [reference_held])
            with TemporaryDirectory(prefix='.prepare-', dir=output) as temporary:
                views = prepare_group(executable, snapshot.keyframes, Path(temporary) / 'bundle',
                                      prepared_group, snapshot.split.source['camera_sha256'])
            training = [admit_training_view(snapshot.split, views[key]) for key in group['train_ids']]
            block, evidence = train_group(group, training, snapshot.poses, device,
                seed_policy=args.seed_policy, options=settings.worker)
            evidence['parameters_sha256'] = hashlib.sha256(b''.join(
                name.encode() + block.parameters[name].tobytes() for name in sorted(block.parameters))).hexdigest()
            report['submaps'].append(evidence)
            report['training_calls'] += len(evidence['training'])
            report['training_steps'] += sum(row['steps'] for row in evidence['training'])
            blocks[group['submap_id']] = block
            for row in plan:
                key = row['keyframe_id']
                if int(snapshot.split.frames[key]['submap_id']) != group['submap_id']:
                    continue
                value = _render(snapshot, views[key], row['selected_ids'], blocks, device)
                report['artifacts'].update(_save_samples(output, value))
                report['queries'].append(value['report'])
            del training, views
        report['summary'] = {mode: aggregate_metrics([row[mode] for row in report['queries']
                                                     if row[mode] is not None])
                             for mode in ('single', 'joint')}
        report.update(status='succeeded', elapsed_seconds=time.perf_counter()-started,
                      peak_rss_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024)
        verify_evaluation_snapshot(snapshot)
        selector.verify()
        _require(file_sha(args.config) == settings.config_sha256 and file_sha(executable) == executable_sha,
                 'Config or preparation executable changed during training')
        _require((output / 'split.json').read_bytes() == snapshot.split.payload, 'Output split changed')
        load_final_poses(output, report, len(snapshot.poses))
        verify_configuration(output, report)
        verify_artifacts(output, report['artifacts'], report['queries'])
        payload = _json_bytes(report, _REPORT_BYTES)
        used = sum(path.stat().st_size for path in output.rglob('*') if path.is_file())
        _require(used + len(payload) <= settings.max_output_bytes, 'Controlled output exceeded budget')
        _require(shutil.disk_usage(output).free >= settings.min_free_disk_bytes + len(payload),
                 'Controlled publication disk reserve is insufficient')
        _exclusive_write(output / 'report.pending.json', payload)
        verify_artifacts(output, report['artifacts'], report['queries'])
        verify_configuration(output, report)
        (output / 'report.pending.json').rename(output / 'report.json')
        return report
    except (Exception, KeyboardInterrupt) as error:
        failure = dict(schema=1, status='failed', error_type=type(error).__name__, error=str(error)[:1024],
                       split_id=snapshot.split.split_id, queries=plan)
        _exclusive_write(output / 'failure.json', _json_bytes(failure, _REPORT_BYTES))
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('keyframes', 'map-directory', 'prepare-executable', 'config', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--selector-executable', type=Path)
    parser.add_argument('--seed-policy', choices=('uniform', 'coverage'), required=True)
    parser.add_argument('--random-seed', type=int, default=0)
    args = parser.parse_args(argv)
    try:
        run(args)
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        print(f'{type(error).__name__}: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
