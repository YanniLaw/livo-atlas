"""Compare two completed Gaussian holdout runs on the same observations."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

from evaluate_gaussian_submaps import (_DEPTH_SAMPLE_DTYPE, _MAX_FRAMES, _MAX_PIXELS,
    _MAX_REPORT_BYTES, _SEED_POLICIES, check_split_identity, file_sha256, split_primary_ids)
from gaussian_map import _require
from gaussian_state import _open_file, _pose, _reject_constant, _unique_object
from joint_renderer import _ALPHA_THRESHOLD, _real_directory
from render_gaussian_map import _exclusive_write, _json_bytes


_FRONTIER_FIELDS = ('session_id', 'pose_version', 'frame_count', 'optimized_frame_count',
                    'parent_frame', 'map_sequence')
_TRAINING_FIELDS = ('keyframe_id', 'frame_ids', 'training_frame_ids', 'steps', 'gaussian_count')
_MODES = ('single', 'joint')
_RUNS = ('baseline', 'candidate')


def _read_json(path):
    with _open_file(path, _MAX_REPORT_BYTES) as source:
        data = source.read()
    document = json.loads(data, object_pairs_hook=_unique_object, parse_constant=_reject_constant)
    _require(isinstance(document, dict) and type(document.get('schema')) is int and
             document['schema'] == 1, f'Unsupported schema in {path}')
    return document, hashlib.sha256(data).hexdigest()


def _read_run(root):
    _real_directory(root)
    report, report_hash = _read_json(root / 'report.json')
    split, split_hash = _read_json(root / 'split.json')
    _require(report.get('kind') == 'livo_atlas.gaussian_holdout_evaluation' and
             report.get('status') == 'succeeded' and report.get('seed_policy') in _SEED_POLICIES,
             f'Expected a complete holdout evaluation in {root}')
    _require(report.get('split_sha256') == split_hash and
             all(report[name] == split[name] for name in _FRONTIER_FIELDS),
             f'Report differs from the frozen split in {root}')
    return report, split, report_hash


def _by_keyframe(rows, frame_count, label):
    _require(isinstance(rows, list) and len(rows) <= frame_count, f'Invalid {label} list')
    result = {}
    for row in rows:
        key = row['keyframe_id']
        _require(type(key) is int and 0 <= key < frame_count and key not in result,
                 f'Invalid or duplicate {label} {key}')
        result[key] = row
    return result


def _check_training(groups, baseline, candidate):
    _require(len(groups) == len(baseline) == len(candidate) and len(groups) > 0,
             'Submaps differ from the training split')
    counts = {}
    for group, old, new in zip(groups, baseline, candidate):
        keys = group['train_ids']
        submap_id = group['submap_id']
        _require(type(submap_id) is int and submap_id >= 0 and submap_id not in counts and
                 submap_id == old['submap_id'] == new['submap_id'] and
                 group['anchor_id'] == old['anchor_id'] == new['anchor_id'] and
                 old['parent_from_submap'] == new['parent_from_submap'] and
                 len(keys) == len(old['training']) == len(new['training']),
                 f'Submap {submap_id} differs from the training split or anchor')
        previous_counts = {}
        for index, (key, a, b) in enumerate(zip(keys, old['training'], new['training'])):
            retained = keys[max(0, index + 1 - 10):index + 1]
            older = retained[:-1] or [key]
            schedule = [key] * 10 + [older[step % len(older)] for step in range(10)]
            _require(a['keyframe_id'] == key and a['frame_ids'] == retained and a['steps'] == 20 and
                     a['training_frame_ids'] == schedule and
                     all(a[name] == b[name] for name in _TRAINING_FIELDS),
                     f'Keyframe {key} has a different training schedule or Gaussian count')
            older_count = sum(previous_counts[frame] for frame in retained[:-1])
            seed_count = a['gaussian_count'] - older_count
            _require(type(seed_count) is int and 1 <= seed_count <= 1000,
                     f'Keyframe {key} exceeds the Gaussian training budget')
            previous_counts[key] = seed_count
        retained = keys[-10:]
        origins = {str(key): previous_counts[key] for key in retained}
        _require(old['retained_frame_ids'] == new['retained_frame_ids'] == retained and
                 old['gaussian_origins'] == new['gaussian_origins'] == origins,
                 f'Submap {submap_id} has different retained training origins')
        counts[submap_id] = sum(origins.values())
    return counts


def _query_plan(split):
    frame_count = split['frame_count']
    _require(type(frame_count) is int and 1 <= frame_count <= _MAX_FRAMES, 'Invalid split frontier')
    observations = _by_keyframe(split['observations'], frame_count, 'observation ID')
    check_split_identity(split['observations'])
    seen, queries = set(), {}
    previous = None
    for group in split['groups']:
        train, held = split_primary_ids(group['anchor_id'], len(group['train_ids']) + len(group['held_out_ids']))
        _require(train == group['train_ids'] and held == group['held_out_ids'] and
                 not seen.intersection(train + held) and
                 all(observations.get(key, {}).get('split') == 'train' for key in train) and
                 all(observations.get(key, {}).get('split') == 'validation' for key in held),
                 'Group differs from the frozen training and holdout split')
        seen.update(train + held)
        current = group['submap_id']
        for index, key in enumerate(held):
            queries[key] = (current, previous, 'interior' if index else 'near_group_start',
                            observations[key]['capture_time'])
        previous = current
    _require(set(observations) == seen and bool(queries), 'Incomplete split observations')
    return queries


def _read_depth_samples(root, view, pixels):
    relative = Path('views') / str(view['keyframe_id']) / 'depth-samples.npy'
    _require(view.get('depth_samples') == relative.as_posix(), f'Expected depth file {relative}')
    path = root / relative
    _real_directory(path.parent.parent)
    _real_directory(path.parent)
    maximum = pixels * _DEPTH_SAMPLE_DTYPE.itemsize + 10_000
    with _open_file(path, maximum) as stream:
        data = stream.read(6)
    _require(file_sha256(path) == view.get('depth_samples_sha256'), f'Depth SHA256 differs at {path}')
    _require(data == b'\x93NUMPY', f'Invalid NPY depth record at {path}')
    samples = np.load(path, mmap_mode='r', allow_pickle=False)
    _require(samples.dtype == _DEPTH_SAMPLE_DTYPE and samples.ndim == 1 and len(samples) <= pixels,
             f'Invalid sparse depth record shape or dtype at {path}')
    indices, observed = samples['pixel_index'], samples['observed_z_m']
    _require(not len(indices) or (indices[-1] < pixels and (indices[1:] > indices[:-1]).all()),
             f'Invalid sparse depth pixel indices at {path}')
    _require(np.isfinite(observed).all() and (observed > 0).all(), f'Invalid observed depth labels at {path}')
    for mode in _MODES:
        covered, predicted = samples[f'{mode}_covered'], samples[f'{mode}_z_m']
        _require(np.isfinite(predicted).all() and (predicted[covered] > 0).all() and
                 (predicted[~covered] == 0).all(), f'Invalid {mode} depth values at {path}')
        if view[mode] is None:
            _require(not covered.any(), f'Unexpected {mode} depth coverage at {path}')
    return samples


def _quality(render, samples, mode, pixels, counts):
    metrics = render['metrics']
    image, covered = metrics['image_pixels'], metrics['covered_pixels']
    mse = metrics['rgb_mse']
    _require(type(image) is int and type(covered) is int and 0 <= covered <= image <= pixels and
             image > 0 and type(mse) in (int, float) and 0 <= mse <= 1 and
             metrics['alpha_threshold'] == _ALPHA_THRESHOLD,
             f'Invalid {mode} image metrics')
    _require(render['gaussian_counts'] == counts and render['gaussian_count'] == sum(counts.values()),
             f'Invalid {mode} Gaussian counts')
    used = int(samples[f'{mode}_covered'].sum())
    _require(metrics['depth_pixels_available'] == len(samples) <= image and
             metrics['depth_pixels_used'] == used <= covered, f'Invalid {mode} depth counts')
    return dict(image_pixels=image, covered_pixels=covered, rgb_mse=mse,
                rgb_psnr_db=-10 * math.log10(mse) if mse else None, alpha_coverage=covered / image)


def _delta(old, new):
    return None if old is None or new is None else new - old


def _depth_comparison(old, new, mode):
    a, b = old[f'{mode}_covered'], new[f'{mode}_covered']
    observed = old['observed_z_m'].astype(np.float64)
    errors = (np.abs(old[f'{mode}_z_m'] - observed), np.abs(new[f'{mode}_z_m'] - observed))
    result = {}
    for name, mask, sides in (('common', a & b, (0, 1)), ('baseline_only', a & ~b, (0,)),
                              ('candidate_only', ~a & b, (1,)), ('neither', ~a & ~b, ())):
        count = int(mask.sum())
        result[name] = dict(count=count, **{f'{_RUNS[i]}_l1_m': float(errors[i][mask].mean())
                                           if count else None for i in sides})
    common = result['common']
    common['delta_l1_m'] = _delta(common['baseline_l1_m'], common['candidate_l1_m'])
    return result


def _compare_query(old, new, roots, plan, counts):
    key = old['keyframe_id']
    current, previous, role, capture_time = plan
    fields = ('submap_id', 'role', 'capture_time', 'calibration_id', 'K', 'camera_from_parent')
    _require((old['submap_id'], old['role'], old['capture_time']) == (current, role, capture_time) and
             all(old[field] == new[field] for field in fields), f'Query {key} camera or identity differs')
    width, height = old['width'], old['height']
    _require(all(type(view[dim]) is int for view in (old, new) for dim in ('width', 'height')) and
             (width, height) == (new['width'], new['height']) and width > 0 and height > 0 and
             width * height <= _MAX_PIXELS, f'Query {key} dimensions differ or exceed budget')
    _pose(old['camera_from_parent'])
    intrinsic = np.asarray(old['K'], dtype=np.float64)
    _require(intrinsic.shape == (3, 3) and np.isfinite(intrinsic).all() and
             intrinsic[0, 0] > 0 and intrinsic[1, 1] > 0, f'Invalid query {key} camera intrinsics')
    _require(all(view['single'] is not None and (view['joint'] is None) == (previous is None)
                 for view in (old, new)), f'Query {key} has a different selected submap count')
    a, b = (_read_depth_samples(root, view, width * height) for root, view in zip(roots, (old, new)))
    _require(np.array_equal(a['pixel_index'], b['pixel_index']), f'Query {key} sparse pixel indices differ')
    _require(np.array_equal(a['observed_z_m'], b['observed_z_m']), f'Query {key} sparse depth labels differ')
    result = {field: old[field] for field in fields}
    result.update(keyframe_id=key, width=width, height=height, joint=None)
    for mode in _MODES:
        if old[mode] is None:
            continue
        rendered = {str(current): counts[current]}
        if mode == 'joint':
            rendered[str(previous)] = counts[previous]
        left = _quality(old[mode], a, mode, width * height, rendered)
        right = _quality(new[mode], b, mode, width * height, rendered)
        _require(left['image_pixels'] == right['image_pixels'] == result['single']['baseline']['image_pixels']
                 if mode == 'joint' else left['image_pixels'] == right['image_pixels'],
                 f'Query {key} valid image counts differ')
        result[mode] = dict(baseline=left, candidate=right, depth=_depth_comparison(a, b, mode))
    return result


def _summary(pairs):
    result = dict(query_count=len(pairs), baseline=None, candidate=None, delta=None, depth=None)
    if not pairs:
        return result
    for side in _RUNS:
        quality = [pair[side] for pair in pairs]
        pixels = sum(row['image_pixels'] for row in quality)
        covered = sum(row['covered_pixels'] for row in quality)
        mse = math.fsum(row['rgb_mse'] * row['image_pixels'] for row in quality) / pixels
        result[side] = dict(image_pixels=pixels, covered_pixels=covered, rgb_mse=mse,
                            rgb_psnr_db=-10 * math.log10(mse) if mse else None, alpha_coverage=covered / pixels)
    old, new = (result[side] for side in _RUNS)
    result['delta'] = dict(rgb_psnr_db=_delta(old['rgb_psnr_db'], new['rgb_psnr_db']),
                           coverage_percentage_points=100 * (new['alpha_coverage'] - old['alpha_coverage']))
    depth = {}
    for part in ('common', 'baseline_only', 'candidate_only', 'neither'):
        rows = [pair['depth'][part] for pair in pairs]
        count = sum(row['count'] for row in rows)
        aggregate = dict(count=count)
        for field in ('baseline_l1_m', 'candidate_l1_m'):
            if field in rows[0]:
                aggregate[field] = (math.fsum(row[field] * row['count'] for row in rows if row['count']) / count
                                    if count else None)
        depth[part] = aggregate
    common = depth['common']
    common['delta_l1_m'] = _delta(common['baseline_l1_m'], common['candidate_l1_m'])
    result['depth'] = depth
    return result


def _timing(seconds):
    return dict(count=len(seconds), total_seconds=math.fsum(seconds),
                median_seconds=float(np.median(seconds)) if seconds else None,
                p95_seconds=float(np.percentile(seconds, 95)) if seconds else None)


def _resources(report, digest):
    groups = report['submaps']
    renders = {mode: [row[mode] for row in report['views'] if row[mode] is not None] for mode in _MODES}
    operations = [row for group in groups for row in group['training']]
    seeds = [row['seed_selection'] for row in operations if 'seed_selection' in row]
    prepass = _timing([row['prepass_seconds'] for row in seeds if row['prepass_seconds'] is not None])
    peaks = groups + renders['single'] + renders['joint']
    return dict(report_sha256=digest, seed_policy=report['seed_policy'], environment=report['environment'],
                elapsed_seconds=report['elapsed_seconds'], peak_rss_bytes=report['peak_rss_bytes'],
                training_seconds=math.fsum(group['training_seconds'] for group in groups),
                gpu_peak_allocated_bytes=max(row['gpu_peak_allocated_bytes'] for row in peaks),
                gpu_peak_reserved_bytes=max(row['gpu_peak_reserved_bytes'] for row in peaks),
                seed_prepass=prepass, reserved_uncovered=sum(row['reserved_uncovered'] for row in seeds),
                render={mode: _timing([row['timings']['render_seconds'] for row in results])
                        for mode, results in renders.items()})


def compare(baseline, candidate, output):
    roots = Path(baseline), Path(candidate)
    old, split, old_hash = _read_run(roots[0])
    new, _, new_hash = _read_run(roots[1])
    _require(old['split_sha256'] == new['split_sha256'], 'Frozen split SHA256 differs')
    plan = _query_plan(split)
    counts = _check_training(split['groups'], old['submaps'], new['submaps'])
    left, right = (_by_keyframe(report['views'], split['frame_count'], 'query ID') for report in (old, new))
    _require(left.keys() == right.keys() == plan.keys(), 'Query IDs differ from the frozen split')
    views = [_compare_query(left[key], right[key], roots, view_plan, counts) for key, view_plan in plan.items()]
    summary = {mode: _summary([row[mode] for row in views if row[mode] is not None]) for mode in _MODES}
    summary['joint_by_role'] = {role: _summary([row['joint'] for row in views if row['joint'] is not None
                                              and row['role'] == role])
                                for role in ('near_group_start', 'interior')}
    updates = sum(len(group['train_ids']) for group in split['groups'])
    result = dict(schema=1, kind='livo_atlas.gaussian_holdout_comparison', status='succeeded',
                  split_sha256=old['split_sha256'], views=views, summary=summary,
                  submap_count=len(counts), training_updates=updates, training_steps=20 * updates,
                  baseline=_resources(old, old_hash), candidate=_resources(new, new_hash),
                  **{name: split[name] for name in _FRONTIER_FIELDS})
    _exclusive_write(Path(output), _json_bytes(result, _MAX_REPORT_BYTES))
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('--baseline', '--candidate', '--output'):
        parser.add_argument(name, type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        compare(args.baseline, args.candidate, args.output)
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(f'{type(error).__name__}: {error}', file=sys.stderr)
        return 1
    print(f'Comparison report: {args.output}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
