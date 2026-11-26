"""Verify controlled sampling runs and diagnose holes and submap handoff views."""

import argparse
import hashlib
import math
from pathlib import Path
import shutil
import sys

import numpy as np

from compare_gaussian_holdout import _check_training, _read_json
from evaluate_gaussian_sampling import load_final_poses, verify_artifacts, verify_configuration
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


def _load(root):
    _real_directory(root)
    report, digest = _read_json(root / 'report.json')
    _require(report.get('kind') == 'livo_atlas.gaussian_sampling_evaluation'
             and report.get('status') == 'succeeded', 'Expected complete controlled sampling report')
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
    if 'config_file' in report:
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


def compare(baseline, candidate, output, *, allow_coverage_ratio_change=False):
    baseline, candidate, output = Path(baseline).resolve(strict=True), Path(candidate).resolve(strict=True), Path(output).absolute()
    _require(not output.exists() and not output.is_symlink() and output.parent.is_dir(), 'Output must be new')
    resolved = output.resolve()
    _require(all(not resolved.is_relative_to(root) and not root.is_relative_to(resolved)
                 for root in (baseline, candidate)), 'Comparison output overlaps inputs')
    a, split, a_digest = _load(baseline)
    b, other, b_digest = _load(candidate)
    controls = _CONTROLS
    if allow_coverage_ratio_change:
        ratios = {name: report['training_options']['coverage_uncovered_ratio']
                  for name, report in (('baseline', a), ('candidate', b))}
        _require(a['seed_policy'] in ('uniform', 'coverage') and ratios['baseline'] == 0
                 and b['seed_policy'] == 'coverage', 'Ratio comparison requires a zero-ratio baseline')
        configurations = [verify_configuration(root, report) for root, report in ((baseline, a), (candidate, b))]
        options = [dict(report['training_options']) for report in (a, b)]
        for document, values in zip(configurations, options):
            document['gaussian'].get('worker', {}).pop('coverage_uncovered_ratio', None)
            values.pop('coverage_uncovered_ratio')
        _require(configurations[0] == configurations[1] and options[0] == options[1],
                 'Other config or training options differ in ratio comparison')
        controls = tuple(key for key in _CONTROLS if key not in ('config_sha256', 'training_options'))
    else:
        _require(a['seed_policy'] == 'uniform' and b['seed_policy'] == 'coverage', 'Expected uniform then coverage')
    _require(split.payload == other.payload and all(a[key] == b[key] for key in controls),
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
            mask = x['image_mask'].astype(bool)
            covered_a = x[mode+'_alpha'] >= _ALPHA_THRESHOLD
            covered_b = y[mode+'_alpha'] >= _ALPHA_THRESHOLD
            row[mode]['coverage_transition'] = dict(
                newly_covered=int((mask & ~covered_a & covered_b).sum()),
                newly_uncovered=int((mask & covered_a & ~covered_b).sum()),
                common_holes=int((mask & ~covered_a & ~covered_b).sum()))
        rows.append(row)
    for report in (a, b):
        summary = {mode: aggregate_metrics([row[mode] for row in report['queries'] if row[mode] is not None])
                   for mode in ('single', 'joint')}
        _require(report['summary'] == summary, 'Run summary differs from reconstructed metrics')
    report = dict(schema=1, kind='livo_atlas.gaussian_sampling_comparison', status='succeeded',
        split_id=split.split_id, random_seed=a['random_seed'], baseline_report_sha256=a_digest,
        candidate_report_sha256=b_digest, training_calls=a['training_calls'], training_steps=a['training_steps'],
        final_gaussian_counts=counts, queries=rows, diagnostics=[])
    if allow_coverage_ratio_change:
        report.update(comparison_scope='coverage_uncovered_ratio', coverage_uncovered_ratios=ratios)
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
    for row in worst:
        key = row['keyframe_id']
        mode = 'joint' if row['joint'] else 'single'
        path = output / f'views/{key}.png'
        labels = (f"ratio {ratios['baseline']:g}", f"ratio {ratios['candidate']:g}") if allow_coverage_ratio_change else ('uniform', 'coverage')
        _diagnostic(path, _samples(baseline, a, by_key_a[key], split.frames[key]),
                    _samples(candidate, b, by_key_b[key], split.frames[key]), mode, labels)
        report['diagnostics'].append(dict(keyframe_id=key, mode=mode, path=f'views/{key}.png', sha256=file_sha(path)))
    _require(file_sha(baseline / 'report.json') == a_digest and file_sha(candidate / 'report.json') == b_digest,
             'Input report changed during comparison')
    for root, value in ((baseline, a), (candidate, b)):
        verify_artifacts(root, value['artifacts'], value['queries'])
        load_final_poses(root, value, len(split.frames))
        if 'config_file' in value:
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
    parser.add_argument('--allow-coverage-ratio-change', action='store_true')
    args = parser.parse_args(argv)
    try:
        compare(args.baseline, args.candidate, args.output,
                allow_coverage_ratio_change=args.allow_coverage_ratio_change)
    except Exception as error:
        print(f'{type(error).__name__}: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
