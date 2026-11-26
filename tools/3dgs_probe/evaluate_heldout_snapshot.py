#!/usr/bin/env python3
"""Score frozen heldout cameras using finalized online weights, without training."""

import argparse
from dataclasses import dataclass
import hashlib
import importlib.metadata
import json
from pathlib import Path
import resource
import shutil
import subprocess
import sys
from tempfile import TemporaryDirectory
import time

import numpy as np

from bundle import load_bundle
from evaluation_metrics import aggregate_metrics
from gaussian_map import _require
from gaussian_state import _open_file
from heldout_query import (load_evaluation_snapshot, render_query,
                           verify_evaluation_snapshot)
from holdout_split import MAX_SPLIT_BYTES
from joint_renderer import infer_blocks
from prepare_holdout_split import file_sha
from render_gaussian_map import _exclusive_write, _json_bytes, _new_output, _png, _sources_report
from worker_config import DEFAULT_CONFIG, load_worker_config


_MIB = 1024 * 1024
_REPORT_BYTES = 2 * _MIB


@dataclass(frozen=True)
class EvaluationSettings:
    max_output_bytes: int
    min_free_disk_bytes: int
    split_file: str
    worker: object
    config_sha256: str


def load_settings(path):
    import yaml

    with _open_file(path, 256 * 1024) as stream:
        raw = stream.read()
    document = yaml.safe_load(raw)
    _require(isinstance(document, dict) and isinstance(document.get('gaussian'), dict), 'Invalid Gaussian config')
    gaussian = document['gaussian']
    evaluation = gaussian.get('evaluation', {})
    _require(isinstance(evaluation, dict) and evaluation.keys() <= {
        'enabled', 'split_file', 'max_output_mib', 'min_free_disk_mib'}, 'Unknown evaluation settings')
    _require(type(evaluation.get('enabled', False)) is bool, 'Invalid evaluation enabled setting')
    policy = gaussian.get('joint_render_policy', 'temporal')
    _require(policy == 'temporal', 'Final scoring currently requires temporal; spatial selection awaits online integration')
    maximum, reserve = evaluation.get('max_output_mib', 256), evaluation.get('min_free_disk_mib', 1024)
    _require(type(maximum) is int and 1 <= maximum <= 4096 and
             type(reserve) is int and 0 <= reserve <= 2**20, 'Invalid evaluation disk budget')
    split_file = evaluation.get('split_file', '')
    _require(isinstance(split_file, str) and '\0' not in split_file, 'Invalid evaluation split_file')
    worker = load_worker_config(path).options
    digest = hashlib.sha256(raw).hexdigest()
    _require(file_sha(path) == digest, 'Evaluation config changed during loading')
    return EvaluationSettings(maximum*_MIB, reserve*_MIB, split_file, worker, digest)


def initialize_cuda(options):
    import torch
    from online_model import checked_cuda_device

    device = checked_cuda_device(options)
    torch.set_num_threads(1)
    software = dict(python=sys.version.split()[0], numpy=np.__version__, torch=str(torch.__version__),
                    cuda=torch.version.cuda, gsplat=importlib.metadata.version('gsplat'),
                    gpu=torch.cuda.get_device_name(device))
    return device, software


def prepare_group_queries(snapshot, owner, executable, output):
    keys = [key for key, frame in snapshot.split.frames.items()
            if frame['query'] and int(frame['submap_id']) == owner]
    # The native bundle format requires a train/validation pair. The first
    # train reference is prepared only to satisfy that format, never optimized.
    reference = next(key for key, frame in snapshot.split.frames.items() if frame['role'] == 'train')
    subprocess.run([str(executable), '--dataset', str(snapshot.keyframes), '--output', str(output),
                    '--train-ids', str(reference), '--validation-ids', ','.join(map(str, keys)),
                    '--source-sha256', snapshot.split.source['camera_sha256'],
                    '--max-pixels', '1000000', '--max-cloud-points', '100000'], check=True, timeout=60)
    views = load_bundle(output)
    _require(sorted(view['keyframe_id'] for view in views) == sorted([reference, *keys]),
             'Prepared evaluation IDs differ from the frozen query plan')
    return {view['keyframe_id']: view for view in views if view['keyframe_id'] in keys}


def _selection(snapshot, owner):
    rows = snapshot.document['entries']
    if 'asset' not in rows[owner]:
        return []
    previous = [int(row['submap_id']) for row in rows[:owner] if 'asset' in row]
    return [previous[-1], owner] if previous else [owner]


def _estimate(snapshot):
    size = _REPORT_BYTES + len(snapshot.split.payload) + 256 * 1024
    scratch = 0
    for frame in snapshot.split.frames.values():
        if not frame['query']:
            continue
        selected = _selection(snapshot, int(frame['submap_id']))
        if not selected:
            continue
        pixels = frame['width'] * frame['height']
        variants = len(selected)
        # Float reference RGB/z + masks; each render RGB/alpha/z; RGB PNG
        # worst case plus per-member ZIP/PNG overhead. No JPEG quantization.
        size += pixels * (18 + 20*variants + 3*(1+variants)) + 64 * 1024
        scratch = max(scratch, 3 * pixels * 12 + 64 * 1024)
    return size, scratch


def _save_samples(output, value):
    query, single, joint = value['query'], value['single'], value['joint']
    folder = output / 'views' / str(query['keyframe_id'])
    folder.mkdir(parents=True)
    arrays = dict(reference_rgb=query['rgb'], reference_depth_m=query['depth_m'],
                  image_mask=query['image_mask'].astype('u1'), depth_mask=query['depth_mask'].astype('u1'))
    _png(folder / 'reference.png', query['rgb'])
    for name, result in (('single', single), ('joint', joint)):
        if result is None:
            continue
        for field in ('rgb', 'alpha', 'depth_m'):
            _require(result[field].dtype == np.dtype('f4'), 'Scoring artifacts require exact float32 renders')
            arrays[name+'_'+field] = result[field]
        _png(folder / (name+'.png'), result['rgb'])
    with (folder / 'samples.npz').open('xb') as stream:
        np.savez(stream, **arrays)
    return {path.relative_to(output).as_posix(): dict(bytes=path.stat().st_size, sha256=file_sha(path))
            for path in sorted(folder.iterdir())}


def _counts(queries):
    return dict(planned=len(queries), **{key: sum(row['status'] == key for row in queries)
                                        for key in ('succeeded', 'skipped', 'failed')})


class EvaluationPreflightError(ValueError):
    def __init__(self, error, split):
        super().__init__(str(error))
        queries = [dict(keyframe_id=key, owner_submap_id=int(frame['submap_id']),
                        phase='final_snapshot', status='failed', reason='preflight_failed')
                   for key, frame in split.frames.items() if frame['query']]
        self.report = dict(schema=1, kind='livo_atlas.gaussian_snapshot_evaluation',
                           phase='final_snapshot', stage='preflight', status='failed',
                           plan_verified=True, snapshot_verified=False,
                           split_id=split.split_id, training_performed=False,
                           error=str(error)[:1024], error_type=type(error).__name__,
                           queries=queries, counts=_counts(queries), summary=None, artifacts={})


def _preflight(args):
    output = _new_output(args.map_directory, args.keyframes, args.output)
    settings = load_settings(args.config)
    snapshot = load_evaluation_snapshot(args.map_directory, args.keyframes)
    if settings.split_file:
        with _open_file(Path(settings.split_file), MAX_SPLIT_BYTES) as stream:
            from holdout_split import HoldoutSplit
            _require(HoldoutSplit(stream.read()).split_id == snapshot.split.split_id,
                     'Configured split differs from the archived online split')
    executable = args.prepare_executable.resolve(strict=True)
    preparer_digest = file_sha(executable)
    predicted, scratch = _estimate(snapshot)
    _require(predicted <= settings.max_output_bytes, 'Evaluation output budget is insufficient for all queries')
    _require(shutil.disk_usage(output.parent).free >= predicted + scratch + settings.min_free_disk_bytes,
             'Evaluation free disk reserve is insufficient')
    queries = [dict(keyframe_id=key, owner_submap_id=int(frame['submap_id']), phase='final_snapshot', status='pending')
               for key, frame in snapshot.split.frames.items() if frame['query']]
    return output, settings, snapshot, executable, preparer_digest, predicted, scratch, queries


def run(args, *, renderer=infer_blocks):
    started = time.perf_counter()
    try:
        output, settings, snapshot, executable, preparer_digest, predicted, scratch, queries = _preflight(args)
    except Exception as error:
        # A verified archived plan remains enumerable even when states, disk
        # capacity or the preparer fail. The CLI emits this bounded ledger on
        # stderr without creating an output directory or initializing CUDA.
        try:
            from holdout_split import HoldoutSplit
            with _open_file(Path(args.map_directory) / 'split.json', MAX_SPLIT_BYTES) as stream:
                split = HoldoutSplit(stream.read())
        except (ValueError, OSError, TypeError):
            raise error
        raise EvaluationPreflightError(error, split) from error
    report = dict(schema=1, kind='livo_atlas.gaussian_snapshot_evaluation', phase='final_snapshot',
                  status='running', split_id=snapshot.split.split_id, training_performed=False,
                  selection_policy='temporal', query_semantics='retrospective_final_online_weights',
                  sources=_sources_report(snapshot), queries=queries, summary=None,
                  verified_source_files=len(snapshot.source_digests),
                  unselected_heldout_ids=[key for key, frame in snapshot.split.frames.items()
                                          if frame['role'] == 'heldout' and not frame['query']],
                  current_preparer_sha256=preparer_digest, frozen_preparer_sha256=snapshot.split.source['preparer_sha256'],
                  config_sha256=settings.config_sha256,
                  budgets=dict(max_output_bytes=settings.max_output_bytes, predicted_output_bytes=predicted,
                               min_free_disk_bytes=settings.min_free_disk_bytes), artifacts={})
    created = False
    try:
        output.mkdir(mode=0o700)
        created = True
        _exclusive_write(output / 'split.json', snapshot.split.payload)
        scorable = []
        for row in queries:
            if not _selection(snapshot, row['owner_submap_id']):
                row.update(status='skipped', reason='missing_model')
            else:
                scorable.append(row)
        device, software = initialize_cuda(settings.worker) if scorable else (None, {})
        report['software'] = software
        peak_allocated = peak_reserved = 0
        for owner in sorted({row['owner_submap_id'] for row in scorable}):
            _require(shutil.disk_usage(output).free >= settings.min_free_disk_bytes + scratch,
                     'Evaluation free disk reserve fell below its preparation budget')
            with TemporaryDirectory(prefix='.prepare-', dir=output) as temporary:
                prepared = prepare_group_queries(snapshot, owner, executable, Path(temporary) / 'bundle')
                for row in [item for item in scorable if item['owner_submap_id'] == owner]:
                    _require(row['keyframe_id'] in prepared, 'A frozen heldout query was not prepared')
                    value = render_query(snapshot, prepared[row['keyframe_id']], _selection(snapshot, owner),
                                         device, renderer=renderer)
                    row.update(value['report'])
                    report['artifacts'].update(_save_samples(output, value))
                    row['status'] = 'succeeded'
                    for result in (value['single'], value['joint']):
                        if result is not None:
                            peak_allocated = max(peak_allocated, result.get('gpu_peak_allocated_bytes', 0))
                            peak_reserved = max(peak_reserved, result.get('gpu_peak_reserved_bytes', 0))
                    del value
        verify_evaluation_snapshot(snapshot)
        _require(file_sha(args.config) == settings.config_sha256 and file_sha(executable) == preparer_digest,
                 'Evaluation config or preparer changed during scoring')
        successful = [row for row in queries if row['status'] == 'succeeded']
        paired = [row for row in successful if row['joint'] is not None]
        report.update(status='completed_with_skips' if len(successful) != len(queries) else 'succeeded',
                      counts=_counts(queries), summary=dict(single=aggregate_metrics([r['single'] for r in successful]),
                      paired_single=aggregate_metrics([r['single'] for r in paired]),
                      joint=aggregate_metrics([r['joint'] for r in paired]), paired_queries=len(paired)),
                      gpu_peak_allocated_bytes=peak_allocated, gpu_peak_reserved_bytes=peak_reserved,
                      scorer_rss_peak_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024,
                      total_seconds=time.perf_counter()-started)
        used = sum(path.stat().st_size for path in output.rglob('*') if path.is_file())
        payload = _json_bytes(report, _REPORT_BYTES)
        _require(used + len(payload) <= settings.max_output_bytes, 'Evaluation exceeded its output budget')
        _require(shutil.disk_usage(output).free >= settings.min_free_disk_bytes + len(payload),
                 'Evaluation free disk reserve fell below its publication budget')
        _exclusive_write(output / 'report.pending.json', payload)
        verify_evaluation_snapshot(snapshot)
        _require(shutil.disk_usage(output).free >= settings.min_free_disk_bytes,
                 'Evaluation free disk reserve fell below its publication budget')
        (output / 'report.pending.json').rename(output / 'report.json')
        return report
    except (Exception, KeyboardInterrupt) as error:
        for row in queries:
            key, owner = row['keyframe_id'], row['owner_submap_id']
            row.clear()
            row.update(keyframe_id=key, owner_submap_id=owner, phase='final_snapshot',
                       status='failed', reason='evaluation_aborted')
        report.update(status='failed', error=str(error)[:1024], error_type=type(error).__name__,
                      counts=_counts(queries), summary=None, artifacts={}, results_valid=False,
                      total_seconds=time.perf_counter()-started)
        error.report = report
        if not created:
            raise
        try:
            (output / 'report.pending.json').unlink(missing_ok=True)
            _exclusive_write(output / 'report.json', _json_bytes(report, _REPORT_BYTES))
        except OSError as write_error:
            # A full/unwritable disk must still leave the complete failed plan
            # on the exception; main emits it on stderr without another file.
            report['failure_record_write_error'] = str(write_error)[:1024]
            raise error from write_error
        raise



def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-directory', type=Path, required=True)
    parser.add_argument('--keyframes', type=Path, required=True)
    parser.add_argument('--prepare-executable', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args(argv)
    try:
        report = run(args)
    except (Exception, KeyboardInterrupt) as error:
        report = getattr(error, 'report', dict(status='not_started', phase='final_snapshot',
                         plan_verified=False, error=str(error)[:1024], error_type=type(error).__name__))
        sys.stderr.write(_json_bytes(report, 64 * 1024).decode())
        return 130 if isinstance(error, KeyboardInterrupt) else 1
    print(json.dumps(dict(status=report['status'], counts=report['counts'], split_id=report['split_id'])))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
