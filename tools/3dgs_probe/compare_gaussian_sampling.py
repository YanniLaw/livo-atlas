"""Verify controlled sampling runs and diagnose holes and submap handoff views."""

import argparse
import copy
import hashlib
import math
from pathlib import Path
import shutil
import sys

import numpy as np

from compare_gaussian_holdout import _check_training, _read_json
from evaluate_gaussian_sampling import (load_final_poses, normalized_training_options,
                                       verify_artifacts, verify_configuration)
from evaluation_metrics import aggregate_metrics, measure
from gaussian_map import _require
from gaussian_state import _inspect_npz, _open_file
from holdout_split import HoldoutSplit, MAX_SPLIT_BYTES, canonical_json
from joint_renderer import _ALPHA_THRESHOLD, _real_directory
from prepare_holdout_split import file_sha
from render_gaussian_map import _exclusive_write, _json_bytes


_CONTROLS = ('random_seed', 'split_id', 'selection_policy', 'selection_semantics',
    'selector_runtime_sha256', 'selection_plan', 'selection_plan_sha256', 'sources',
    'source_sha256', 'final_poses_sha256', 'config_sha256', 'preparer_sha256',
    'training_options', 'software', 'groups', 'training_calls', 'training_steps')
_REFERENCE = ('reference_rgb', 'reference_depth_m', 'image_mask', 'depth_mask')


def _same_configuration(a, b):
    """Compare resolved YAML values without sorting or stringifying its keys."""
    if type(a) is not type(b):
        return False
    if isinstance(a, dict):
        left = {(type(key), key): value for key, value in a.items()}
        right = {(type(key), key): value for key, value in b.items()}
        return left.keys() == right.keys() and all(_same_configuration(value, right[key])
                                                   for key, value in left.items())
    if isinstance(a, (list, tuple)):
        return len(a) == len(b) and all(_same_configuration(x, y) for x, y in zip(a, b))
    if isinstance(a, set):
        return {(type(value), value) for value in a} == {(type(value), value) for value in b}
    return a == b


def depth_comparison(baseline, candidate, reference, labels):
    mask = labels & (baseline['alpha'] >= _ALPHA_THRESHOLD) & (candidate['alpha'] >= _ALPHA_THRESHOLD)
    count = int(mask.sum())
    result = dict(pixels=count, reference_pixels=int(labels.sum()),
                  reason=None if count else 'no_common_covered_depth')
    for name, value in (('baseline', baseline), ('candidate', candidate)):
        errors = value['depth_m'][mask].astype('f8') - reference[mask].astype('f8')
        result[name+'_absolute_sum'] = float(np.abs(errors).sum())
        result[name+'_squared_sum'] = float(np.square(errors).sum())
        result[name+'_l1_m'] = float(np.abs(errors).mean()) if count else None
        result[name+'_rmse_m'] = float(np.sqrt(np.square(errors).mean())) if count else None
    return result


def _depth_total(rows):
    count = sum(row['pixels'] for row in rows)
    result = dict(pixels=count, reference_pixels=sum(row['reference_pixels'] for row in rows),
                  reason=None if count else 'no_common_covered_depth')
    for name in ('baseline', 'candidate'):
        absolute = sum(row[name+'_absolute_sum'] for row in rows)
        squared = sum(row[name+'_squared_sum'] for row in rows)
        result[name+'_l1_m'] = absolute/count if count else None
        result[name+'_rmse_m'] = math.sqrt(squared/count) if count else None
    return result


def coverage_transition(reference_alpha, target_alpha, image_mask):
    """Pixel turnover against the supplied reference, not against an implicit run."""
    reference, target, mask = map(np.asarray, (reference_alpha, target_alpha, image_mask))
    _require(reference.shape == target.shape == mask.shape and mask.ndim == 2
             and 0 < mask.size <= 1_000_000 and mask.dtype in (np.dtype('bool'), np.dtype('u1'))
             and np.all(mask <= 1), 'Invalid coverage reference or mask')
    for alpha in (reference, target):
        _require(alpha.dtype.kind == 'f' and np.isfinite(alpha).all()
                 and np.all(alpha >= 0) and np.all(alpha <= 1+1e-6), 'Invalid coverage alpha')
    mask = mask.astype(bool, copy=False)
    old, new = reference >= _ALPHA_THRESHOLD, target >= _ALPHA_THRESHOLD
    return dict(newly_covered=int((mask & ~old & new).sum()),
                newly_uncovered=int((mask & old & ~new).sum()),
                common_holes=int((mask & ~old & ~new).sum()))


def _requires_configuration(root, report):
    # Missing a declaration must not turn a modern result into a legacy one.
    options = report.get('training_options', {})
    path = root / 'config.yaml'
    return ('config_file' in report or
            (isinstance(options, dict) and any(key in options for key in
                ('coverage_uncovered_ratio', 'coverage_sampling', 'seed_distribution'))) or
            path.exists() or path.is_symlink())


def _load(root):
    _real_directory(root)
    report, digest = _read_json(root / 'report.json')
    _require(report.get('kind') == 'livo_atlas.gaussian_sampling_evaluation'
             and report.get('status') == 'succeeded', 'Expected complete controlled sampling report')
    _require(type(report.get('random_seed')) is int and 0 <= report['random_seed'] < 2**32,
             'Invalid controlled random seed')
    with _open_file(root / 'split.json', MAX_SPLIT_BYTES) as stream:
        split = HoldoutSplit(stream.read())
    poses = load_final_poses(root, report, len(split.frames))
    _require(report['split_id'] == split.split_id, 'Report differs from frozen split')
    _require(hashlib.sha256(canonical_json(report['selection_plan'])).hexdigest()
             == report['selection_plan_sha256'], 'Selection plan digest differs')
    expected_queries = [key for key, frame in split.frames.items() if frame['query']]
    _require([row['keyframe_id'] for row in report['selection_plan']] == expected_queries
             == [row['keyframe_id'] for row in report['queries']], 'Incomplete query plan')
    seen = []
    for group in report['groups']:
        owner = group['submap_id']
        primary = [key for key, frame in split.frames.items() if int(frame['submap_id']) == owner]
        train = [key for key in primary if split.frames[key]['role'] == 'train']
        held = [key for key in primary if split.frames[key]['role'] == 'heldout']
        _require(primary and group['anchor_id'] == primary[0] and group['train_ids'] == train
                 and group['held_out_ids'] == held, 'Training roles differ from frozen split')
        seen.extend(primary)
    _require(seen == list(split.frames), 'Incomplete or repeated primary membership')
    calls = sum(len(group['train_ids']) for group in report['groups'])
    _require(report['training_calls'] == calls and report['training_steps'] == 20*calls,
             'Training budget differs from complete frozen observations')
    for row, selection in zip(report['queries'], report['selection_plan']):
        key = row['keyframe_id']
        owner = int(split.frames[key]['submap_id'])
        _require(row['owner_submap_id'] == owner, 'Query owner differs from frozen split')
        ids = row['selected_ids']
        _require(ids == selection['selected_ids'] and ids == sorted(set(ids)) and owner in ids
                 and ids[-1] == owner and 1 <= len(ids) <= 2
                 and (row['joint'] is not None) == (len(ids) == 2), 'Query selection differs from frozen plan')
        frame = split.frames[key]
        _require(row['query']['width'] == frame['width'] and row['query']['height'] == frame['height']
                 and np.array_equal(np.asarray(row['query']['K']).ravel(), frame['intrinsics']),
                 'Query camera differs from frozen calibration')
        body_camera = np.asarray(frame['camera_from_body'], dtype='f8').reshape(4, 4)
        camera = np.asarray(row['query']['camera_from_parent'], dtype='f8').reshape(4, 4)
        _require(np.allclose(camera, body_camera @ np.linalg.inv(poses[key]), atol=1e-9, rtol=1e-12),
                 'Query camera differs from frozen final pose')
    verify_artifacts(root, report['artifacts'], report['queries'])
    if _requires_configuration(root, report):
        verify_configuration(root, report)
    return report, split, digest


def _samples(root, report, row, frame):
    key = row['keyframe_id']
    relative = f'views/{key}/samples.npz'
    _real_directory(root / 'views')
    _real_directory(root / f'views/{key}')
    receipt = report['artifacts'][relative]
    path = root / relative
    pixels = row['query']['width'] * row['query']['height']
    _require(type(pixels) is int and 0 < pixels <= 1_000_000, 'Invalid sample dimensions')
    maximum = pixels*58 + 64*1024
    expected = set(_REFERENCE) | {mode+'_'+field for mode in ('single', 'joint')
                                 if row[mode] is not None for field in ('rgb', 'alpha', 'depth_m')}
    with _open_file(path, maximum) as stream:
        _require(path.stat().st_size == receipt['bytes'] and file_sha(path) == receipt['sha256'],
                 'Sample artifact differs from receipt')
        shape = (row['query']['height'], row['query']['width'])
        specs = {name: (np.dtype('u1') if name.endswith('mask') else np.dtype('<f4'),
                        (*shape, 3) if name.endswith('rgb') else shape) for name in expected}
        _inspect_npz(stream, specs)
        stream.seek(0)
        with np.load(stream, allow_pickle=False) as source:
            values = {name: source[name] for name in expected}
    query = dict(rgb=values['reference_rgb'], depth_m=values['reference_depth_m'],
                 image_mask=values['image_mask'], depth_mask=values['depth_mask'])
    shape = (row['query']['height'], row['query']['width'])
    _require(query['rgb'].shape == (*shape, 3), 'Sample dimensions differ from query')
    for name, field in (('reference_rgb', 'rgb_float'), ('reference_depth_m', 'depth'),
                        ('image_mask', 'image_mask'), ('depth_mask', 'depth_mask')):
        _require(hashlib.sha256(values[name].tobytes()).hexdigest() == frame[field+'_sha256'],
                 'Reference artifact differs from frozen observation')
    for mode in ('single', 'joint'):
        if row[mode] is None:
            continue
        rendered = {field: values[mode+'_'+field] for field in ('rgb', 'alpha', 'depth_m')}
        _require(all(value.dtype == np.dtype('<f4') for value in rendered.values()),
                 'Sample render must preserve float32')
        _require(measure(rendered, query) == row[mode], 'Reported metric differs from float samples')
    return values


def _rendered(values, mode):
    return {field: values[mode+'_'+field] for field in ('rgb', 'alpha', 'depth_m')}


def _diagnostic(path, baseline, candidate, mode, labels=('uniform', 'coverage')):
    from PIL import Image, ImageDraw

    rows = []
    titles = ['reference', 'single', mode, 'RGB error [0,.5]', 'holes (magenta)',
              'depth error [0,1m]', 'joint gain [-.2,.2]']
    for label, values in zip(labels, (baseline, candidate)):
        query = values['reference_rgb']
        mask = values['image_mask'].astype(bool)
        labels = values['depth_mask'].astype(bool)
        rgb, alpha, z = (values[mode+'_'+key] for key in ('rgb', 'alpha', 'depth_m'))
        error = np.abs(rgb-query).mean(axis=2)
        red = np.zeros_like(rgb)
        red[..., 0] = np.clip(error/.5, 0, 1)
        holes = rgb.copy()
        holes[mask & (alpha < _ALPHA_THRESHOLD)] = [1, 0, 1]
        depth = np.full_like(rgb, .15)
        depth[labels & (alpha < _ALPHA_THRESHOLD)] = [1, 0, 1]
        valid = labels & (alpha >= _ALPHA_THRESHOLD)
        depth[valid] = 0
        depth[..., 0][valid] = np.clip(np.abs(z[valid]-values['reference_depth_m'][valid]), 0, 1)
        delta = np.abs(values['single_rgb']-query).mean(axis=2)-error
        gain = np.zeros_like(rgb)
        gain[..., 0] = np.clip(-delta/.2, 0, 1)
        gain[..., 1] = np.clip(delta/.2, 0, 1)
        panels = [query, values['single_rgb'], rgb, red, holes, depth, gain]
        tiles = []
        for title, panel in zip(titles, panels):
            panel = panel.copy()
            panel[~mask] = 0
            image = Image.fromarray(np.rint(np.clip(panel, 0, 1)*255).astype('u1'))
            image.thumbnail((320, 256))
            tile = Image.new('RGB', (320, 280), 'white')
            tile.paste(image, (0, 24))
            ImageDraw.Draw(tile).text((3, 5), label+': '+title, fill='black')
            tiles.append(tile)
        rows.append(tiles)
    sheet = Image.new('RGB', (7*320, 2*280), 'white')
    for row, tiles in enumerate(rows):
        for column, tile in enumerate(tiles):
            sheet.paste(tile, (column*320, row*280))
    sheet.save(path)


def compare(baseline, candidate, output, *, allow_coverage_ratio_change=False,
             allow_coverage_sampling_change=False, frozen_parameter=None,
             reduced_learning_rate=None, allow_seed_distribution_change=False):
    _require(frozen_parameter is None or
             (isinstance(frozen_parameter, str) and frozen_parameter in ('means', 'log_scales')),
             'Frozen parameter must be means or log_scales')
    _require(reduced_learning_rate is None or
             (isinstance(reduced_learning_rate, str) and reduced_learning_rate in ('means', 'log_scales')),
             'Reduced learning rate must be means or log_scales')
    _require(sum((bool(allow_coverage_ratio_change), bool(allow_coverage_sampling_change), bool(allow_seed_distribution_change),
                  frozen_parameter is not None, reduced_learning_rate is not None)) <= 1,
             'Only one setting may vary in a controlled comparison')
    parameter = frozen_parameter if frozen_parameter is not None else reduced_learning_rate
    baseline, candidate, output = Path(baseline).resolve(strict=True), Path(candidate).resolve(strict=True), Path(output).absolute()
    _require(not output.exists() and not output.is_symlink() and output.parent.is_dir(), 'Output must be new')
    resolved = output.resolve()
    _require(all(not resolved.is_relative_to(root) and not root.is_relative_to(resolved)
                 for root in (baseline, candidate)), 'Comparison output overlaps inputs')
    a, split, a_digest = _load(baseline)
    b, other, b_digest = _load(candidate)
    controls = _CONTROLS
    if allow_coverage_ratio_change or allow_coverage_sampling_change or allow_seed_distribution_change or parameter is not None:
        configurations = [verify_configuration(root, report) for root, report in ((baseline, a), (candidate, b))]
        # Nested learning-rate removal must not mutate the loaded reports.
        options = [copy.deepcopy(normalized_training_options(report)) for report in (a, b)]
        for index, document in enumerate(configurations):
            # Detach the edited path so YAML aliases elsewhere remain controls.
            document = dict(document)
            document['gaussian'] = dict(document['gaussian'])
            worker = dict(document['gaussian'].get('worker', {}))
            document['gaussian']['worker'] = worker
            configurations[index] = document
            if allow_coverage_ratio_change or allow_coverage_sampling_change:
                worker.setdefault('coverage_sampling', 'resample')
                options[index].setdefault('coverage_sampling', 'resample')
            if allow_seed_distribution_change:
                worker.setdefault('seed_distribution', 'flat')
        if parameter is not None:
            learning_rates = dict(parameter=parameter,
                baseline=options[0]['learning_rates'][parameter],
                candidate=options[1]['learning_rates'][parameter])
            _require(a['seed_policy'] == b['seed_policy'] and a['seed_policy'] in ('uniform', 'coverage')
                     and learning_rates['baseline'] > 0, 'Rate comparison requires the same policy and a positive baseline')
            if frozen_parameter is not None:
                _require(learning_rates['candidate'] == 0,
                         'Freeze comparison requires a positive learning rate then zero')
            else:
                _require(0 < learning_rates['candidate'] < learning_rates['baseline'],
                         'Reduced learning rate must remain positive and decrease')
        elif allow_seed_distribution_change:
            distributions = dict(zip(('baseline', 'candidate'), (values['seed_distribution'] for values in options)))
            _require(a['seed_policy'] == b['seed_policy'] and a['seed_policy'] in ('uniform', 'coverage') and
                     distributions == {'baseline': 'flat', 'candidate': 'stratified'},
                     'Distribution comparison requires the same policy, flat then stratified')
            varied_key = 'seed_distribution'
        elif allow_coverage_sampling_change:
            samplings = dict(zip(('baseline', 'candidate'), (values['coverage_sampling'] for values in options)))
            _require(a['seed_policy'] == b['seed_policy'] == 'coverage'
                     and samplings == {'baseline': 'resample', 'candidate': 'replace'},
                     'Sampling comparison requires resample then replace')
            varied_key = 'coverage_sampling'
        else:
            ratios = {name: report['training_options']['coverage_uncovered_ratio']
                      for name, report in (('baseline', a), ('candidate', b))}
            _require(a['seed_policy'] in ('uniform', 'coverage') and ratios['baseline'] == 0
                     and b['seed_policy'] == 'coverage', 'Ratio comparison requires a zero-ratio baseline')
            varied_key = 'coverage_uncovered_ratio'
        for document, values in zip(configurations, options):
            if parameter is not None:
                worker = document['gaussian']['worker']
                worker['learning_rates'] = dict(worker.get('learning_rates', {}))
                worker['learning_rates'].pop(parameter, None)
                values['learning_rates'].pop(parameter)
            else:
                document['gaussian'].get('worker', {}).pop(varied_key, None)
                values.pop(varied_key)
        _require(_same_configuration(configurations[0], configurations[1])
                 and canonical_json(options[0]) == canonical_json(options[1]),
                 'Other config or training options differ in controlled comparison')
        controls = tuple(key for key in _CONTROLS if key not in ('config_sha256', 'training_options'))
    else:
        _require(a['seed_policy'] == 'uniform' and b['seed_policy'] == 'coverage', 'Expected uniform then coverage')

    def control_values(report):
        return {key: normalized_training_options(report) if key == 'training_options' else report[key]
                for key in controls}

    _require(split.payload == other.payload
             and canonical_json(control_values(a)) == canonical_json(control_values(b)),
             'Controlled experiment inputs or training options differ')
    counts = _check_training(a['groups'], a, b)
    rows = []
    for old, new in zip(a['queries'], b['queries']):
        _require(old['query'] == new['query'], 'Query camera differs')
        frame = split.frames[old['keyframe_id']]
        x, y = _samples(baseline, a, old, frame), _samples(candidate, b, new, frame)
        _require(all(np.array_equal(x[key], y[key]) for key in _REFERENCE), 'Reference samples differ')
        row = dict(keyframe_id=old['keyframe_id'], selected_ids=old['selected_ids'])
        for mode in ('single', 'joint'):
            if old[mode] is None:
                row[mode] = None
                continue
            row[mode] = dict(baseline=old[mode], candidate=new[mode], common_depth=depth_comparison(
                _rendered(x, mode), _rendered(y, mode), x['reference_depth_m'], x['depth_mask'].astype(bool)))
            row[mode]['coverage_transition'] = coverage_transition(
                x[mode+'_alpha'], y[mode+'_alpha'], x['image_mask'])
        rows.append(row)
    for report in (a, b):
        summary = {mode: aggregate_metrics([row[mode] for row in report['queries'] if row[mode] is not None])
                   for mode in ('single', 'joint')}
        _require(report['summary'] == summary, 'Run summary differs from reconstructed metrics')
    report = dict(schema=1, kind='livo_atlas.gaussian_sampling_comparison', status='succeeded',
        split_id=split.split_id, random_seed=a['random_seed'], baseline_report_sha256=a_digest,
        candidate_report_sha256=b_digest, training_calls=a['training_calls'], training_steps=a['training_steps'],
        final_gaussian_counts=counts, queries=rows, diagnostics=[])
    if allow_seed_distribution_change:
        report.update(comparison_scope='seed_distribution', seed_distributions=distributions)
    elif allow_coverage_ratio_change:
        report.update(comparison_scope='coverage_uncovered_ratio', coverage_uncovered_ratios=ratios)
    elif allow_coverage_sampling_change:
        report.update(comparison_scope='coverage_sampling', coverage_sampling=samplings)
    elif parameter is not None:
        report.update(comparison_scope='learning_rates.'+parameter, learning_rates=learning_rates)
    for mode in ('single', 'joint'):
        selected = [row[mode] for row in rows if row[mode] is not None]
        report[mode] = dict(query_count=len(selected), baseline=aggregate_metrics([r['baseline'] for r in selected]),
            candidate=aggregate_metrics([r['candidate'] for r in selected]),
            common_depth=_depth_total([r['common_depth'] for r in selected]))
    _require(shutil.disk_usage(output.parent).free >= 1024**3 + 128*1024**2,
             'Comparison disk reserve is insufficient')
    output.mkdir(mode=0o700)
    (output / 'views').mkdir()
    worst = sorted(rows, key=lambda row: max(row['joint' if row['joint'] else 'single'][run]['rgb_mse']
                                            for run in ('baseline', 'candidate')), reverse=True)[:6]
    by_key_a = {row['keyframe_id']: row for row in a['queries']}
    by_key_b = {row['keyframe_id']: row for row in b['queries']}
    if frozen_parameter is not None:
        labels = ('baseline', frozen_parameter+' fixed')
    elif reduced_learning_rate is not None:
        labels = tuple(f'{parameter} lr {learning_rates[side]:g}' for side in ('baseline', 'candidate'))
    elif allow_seed_distribution_change:
        labels = ('flat', 'stratified')
    elif allow_coverage_sampling_change:
        labels = ('resample', 'replace')
    elif allow_coverage_ratio_change:
        labels = (f"ratio {ratios['baseline']:g}", f"ratio {ratios['candidate']:g}")
    else:
        labels = ('uniform', 'coverage')
    for row in worst:
        key = row['keyframe_id']
        mode = 'joint' if row['joint'] else 'single'
        path = output / f'views/{key}.png'
        _diagnostic(path, _samples(baseline, a, by_key_a[key], split.frames[key]),
                    _samples(candidate, b, by_key_b[key], split.frames[key]), mode, labels)
        report['diagnostics'].append(dict(keyframe_id=key, mode=mode, path=f'views/{key}.png', sha256=file_sha(path)))
    _require(file_sha(baseline / 'report.json') == a_digest and file_sha(candidate / 'report.json') == b_digest,
             'Input report changed during comparison')
    for root, value in ((baseline, a), (candidate, b)):
        verify_artifacts(root, value['artifacts'], value['queries'])
        load_final_poses(root, value, len(split.frames))
        if _requires_configuration(root, value):
            verify_configuration(root, value)
    payload = _json_bytes(report, 2*1024*1024)
    _require(shutil.disk_usage(output).free >= 1024**3 + len(payload),
             'Comparison publication disk reserve is insufficient')
    pending = output / 'report.pending.json'
    _exclusive_write(pending, payload)
    _require(shutil.disk_usage(output).free >= 1024**3,
             'Comparison publication disk reserve fell below budget')
    pending.rename(output / 'report.json')
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('baseline', 'candidate', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    variation = parser.add_mutually_exclusive_group()
    variation.add_argument('--allow-coverage-ratio-change', action='store_true')
    variation.add_argument('--allow-coverage-sampling-change', action='store_true')
    variation.add_argument('--allow-seed-distribution-change', action='store_true')
    variation.add_argument('--frozen-parameter', choices=('means', 'log_scales'),
                           help='Verify that only this learning rate changes from positive to zero')
    variation.add_argument('--reduced-learning-rate', choices=('means', 'log_scales'),
                           help='Verify that only this learning rate decreases and remains positive')
    args = parser.parse_args(argv)
    try:
        compare(args.baseline, args.candidate, args.output,
                allow_coverage_ratio_change=args.allow_coverage_ratio_change,
                allow_coverage_sampling_change=args.allow_coverage_sampling_change,
                allow_seed_distribution_change=args.allow_seed_distribution_change,
                frozen_parameter=args.frozen_parameter, reduced_learning_rate=args.reduced_learning_rate)
    except Exception as error:
        print(f'{type(error).__name__}: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
