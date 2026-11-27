"""Diagnose archived sampling textures on an eight-run fixed coverage region (CPU)."""

import argparse
import hashlib
from pathlib import Path
import shutil
import sys

import numpy as np

from compare_gaussian_sampling import _rendered, _samples
from gaussian_map import _require
from joint_renderer import _ALPHA_THRESHOLD
from render_gaussian_map import _exclusive_write, _json_bytes, _new_output
from study_verification import CONDITIONS, capture_file, load_study_inputs, verify_inputs
from texture_metrics import (COHORTS, GRADIENT_THRESHOLD, aggregate_partitions, aggregate_stats,
                             coverage_partition, gradient_masks, texture_metrics)


MAX_OUTPUT = 64*1024**2
RESERVE_BYTES = 1024**3


def _mask_receipt(mask):
    return dict(pixels=int(mask.sum()), sha256=hashlib.sha256(mask.tobytes()).hexdigest())


def _texture_total(rows):
    return {region: {cohort: aggregate_stats([row[region][cohort] for row in rows]) for cohort in COHORTS}
            for region in ('image', 'common')}


def _analyze(inputs):
    metrics = dict(schema=1, kind='livo_atlas.common_coverage_texture_metrics', masks={}, runs={}, pairs={})
    for scene in inputs['protocol']['scenes']:
        runs = {name: run for name, run in inputs['runs'].items() if run['scene'] == scene}
        metrics['masks'][scene] = []
        for name, run in runs.items():
            metrics['runs'][name] = dict(scene=scene, condition=run['condition'], random_seed=run['seed'], queries=[], summary={})
        for policy in ('uniform', 'coverage'):
            for seed in (0, 1):
                metrics['pairs'][f'{scene}-{policy}-{seed}'] = dict(scene=scene, policy=policy, random_seed=seed,
                    baseline=f'{scene}-flat-{policy}-{seed}', candidate=f'{scene}-stratified-{policy}-{seed}', queries=[], summary={})
        for index, frozen_row in enumerate(next(iter(runs.values()))['report']['queries']):
            key = frozen_row['keyframe_id']
            samples = {}
            for name, run in runs.items():
                row = run['report']['queries'][index]
                _require(row['keyframe_id'] == key, 'Query order differs')
                samples[name] = _samples(run['root'], run['report'], row, run['split'].frames[key])
            first = next(iter(samples.values()))
            reference, image = first['reference_rgb'], first['image_mask'].astype(bool)
            gradients = gradient_masks(reference, image)
            mask_row = dict(keyframe_id=key, image=_mask_receipt(image),
                            gradients={name: _mask_receipt(mask) for name, mask in gradients.items()}, modes={})
            metrics['masks'][scene].append(mask_row)
            for name in runs:
                metrics['runs'][name]['queries'].append(dict(keyframe_id=key, modes={}))
            for pair in (pair for pair in metrics['pairs'].values() if pair['scene'] == scene):
                pair['queries'].append(dict(keyframe_id=key, modes={}))
            for mode in ('single', 'joint'):
                present = [run['report']['queries'][index][mode] is not None for run in runs.values()]
                _require(all(present) or not any(present), 'Rendering modes differ across runs')
                if not any(present):
                    continue
                common = image.copy()
                for values in samples.values():
                    common &= values[mode+'_alpha'] >= _ALPHA_THRESHOLD
                mask_row['modes'][mode] = _mask_receipt(common)
                regions = dict(image=image, common=common)
                for name, values in samples.items():
                    scores = texture_metrics(reference, values[mode+'_rgb'], regions, gradients)
                    old, new = runs[name]['report']['queries'][index][mode], scores['image']['all']
                    for new_key, old_key in (('pixels', 'image_pixels'), ('rgb_absolute_sum', 'rgb_absolute_error_sum'),
                            ('rgb_squared_sum', 'rgb_squared_error_sum'), ('ssim_windows', 'ssim_windows'), ('ssim_sum', 'ssim_sum')):
                        _require(new[new_key] == old[old_key], 'Diagnostic changed the established all-image score')
                    metrics['runs'][name]['queries'][-1]['modes'][mode] = scores
                for pair in (pair for pair in metrics['pairs'].values() if pair['scene'] == scene):
                    pair['queries'][-1]['modes'][mode] = coverage_partition(reference,
                        _rendered(samples[pair['baseline']], mode), _rendered(samples[pair['candidate']], mode), image)
            del samples
        for name in runs:
            run = metrics['runs'][name]
            run['summary'] = {mode: _texture_total([row['modes'][mode] for row in run['queries'] if mode in row['modes']])
                              for mode in ('single', 'joint') if any(mode in row['modes'] for row in run['queries'])}
        for pair in (pair for pair in metrics['pairs'].values() if pair['scene'] == scene):
            pair['summary'] = {mode: aggregate_partitions([row['modes'][mode] for row in pair['queries'] if mode in row['modes']])
                               for mode in ('single', 'joint') if any(mode in row['modes'] for row in pair['queries'])}
    return metrics


def _check_output(output):
    _require(sum(path.stat().st_size for path in output.iterdir() if path.is_file()) <= MAX_OUTPUT,
             'Diagnostic output exceeds byte budget')
    _require(shutil.disk_usage(output).free >= RESERVE_BYTES, 'Insufficient disk reserve')


def diagnose(study, coverage, candidates, output):
    roots = [Path(root).resolve(strict=True) for root in (study, coverage, candidates)]
    output = _new_output(roots[0], roots[1], output)
    _new_output(roots[2], roots[2], output)
    _require(shutil.disk_usage(output.parent).free >= RESERVE_BYTES+MAX_OUTPUT, 'Insufficient diagnostic disk reserve')
    inputs = load_study_inputs(*roots)
    for snapshot in inputs['snapshots'].values():
        _new_output(snapshot.directory, snapshot.keyframes, output)
    output.mkdir()
    try:
        metrics = _analyze(inputs)
        raw = _json_bytes(metrics, MAX_OUTPUT)
        _exclusive_write(output/'metrics.json', raw)
        metric_receipt = dict(bytes=len(raw), sha256=hashlib.sha256(raw).hexdigest())
        report = dict(schema=1, kind='livo_atlas.common_coverage_texture_diagnostic', status='succeeded',
                      new_training_calls=0, new_render_calls=0, runs=len(inputs['runs']), pairs=len(metrics['pairs']),
                      protocol=dict(alpha_threshold=_ALPHA_THRESHOLD, gradient_threshold=GRADIENT_THRESHOLD,
                          gradient='sqrt(sum(four_neighbor_RGB_difference_squared)/6)',
                          common_region='image_mask AND all eight runs covered, per query/mode',
                          ssim='complete 11x11 region window, sigma1.5, gradient cohort at center',
                          pairs='same-policy same-seed flat to stratified',
                          aggregation='sum sufficient statistics, then derive; no addition of dB'),
                      sources=dict(roots=[str(root) for root in roots], receipts=inputs['receipts']),
                      artifacts={'metrics.json': metric_receipt},
                      summary={name: run['summary'] for name, run in metrics['runs'].items()},
                      pair_summary={name: pair['summary'] for name, pair in metrics['pairs'].items()})
        pending = _json_bytes(report, MAX_OUTPUT-len(raw))
        _exclusive_write(output/'report.pending', pending)
        pending_receipt = dict(bytes=len(pending), sha256=hashlib.sha256(pending).hexdigest())
        # Repeat after the long verification too: a late source change cannot publish success.
        verify_inputs(inputs)
        capture_file(output/'metrics.json', {}, MAX_OUTPUT, expected=metric_receipt)
        verify_inputs(inputs)
        _check_output(output)
        capture_file(output/'metrics.json', {}, MAX_OUTPUT, expected=metric_receipt)
        capture_file(output/'report.pending', {}, MAX_OUTPUT, expected=pending_receipt)
        (output/'report.pending').rename(output/'report.json')
        return report
    except Exception as error:
        _exclusive_write(output/'failure.json', _json_bytes(dict(status='failed', error=str(error)), 64*1024))
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('study-directory', 'coverage-directory', 'candidate-directory', 'output'):
        parser.add_argument('--'+name, required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        report = diagnose(args.study_directory, args.coverage_directory, args.candidate_directory, args.output)
        print(f"Diagnosed {report['runs']} archived runs and {report['pairs']} pairs; no training/rendering")
    except Exception as error:
        print(str(error), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
