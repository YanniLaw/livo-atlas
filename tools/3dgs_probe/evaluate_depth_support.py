#!/usr/bin/env python3
"""Offline LiDAR support and depth metrics for finalized heldout score archives."""

import argparse
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import re
import sys
from tempfile import TemporaryDirectory

import numpy as np

from compare_gaussian_holdout import _by_keyframe, _read_json
from compare_gaussian_sampling import _samples
from depth_support import (GROUPS, MINIMUM_TOLERANCE_M, MINIMUM_VOTES, NEIGHBOR_OFFSETS, RADII,
                           RELATIVE_TOLERANCE, VOTE_FIELDS, compare_view, project_support,
                           summarize_mode, support_masks)
from gaussian_map import _require
from gaussian_state import _LIMITS, _open_file, _reject_constant, _unique_object
from heldout_query import EvaluationSnapshot, load_evaluation_snapshot, verify_evaluation_snapshot
from holdout_split import MAX_SPLIT_BYTES
from joint_renderer import _ALPHA_THRESHOLD, _real_directory, _stamp
from render_gaussian_map import _exclusive_write, _json_bytes, _sources_report

_MODES = ('single', 'joint')
_FRONTIER = ('session_id', 'map_sequence', 'pose_version', 'frame_count', 'optimized_frame_count', 'parent_frame')
_REPORT_LIMIT = 16*1024**2


@dataclass(frozen=True)
class ScoredArchive:
    scores: Path
    snapshot: EvaluationSnapshot
    report: dict
    queries: dict
    report_sha256: str
    sources: dict


def load_scored_archive(map_directory, keyframes, scores):
    scores = Path(scores).resolve(strict=True)
    _real_directory(scores)
    report, digest = _read_json(scores / 'report.json')
    _require(report.get('kind') == 'livo_atlas.gaussian_snapshot_evaluation'
             and report.get('phase') == 'final_snapshot' and report.get('training_performed') is False
             and report.get('status') in ('succeeded', 'completed_with_skips')
             and report.get('selection_policy') in ('spatial', 'temporal'),
             'Expected finalized heldout scores')
    snapshot = load_evaluation_snapshot(map_directory, keyframes)
    sources = _sources_report(snapshot)
    _require(all(report['sources'].get(key) == value for key, value in sources.items()
                 if key not in ('map_directory', 'keyframes')), 'Score archive differs from its sealed snapshot')
    with _open_file(scores / 'split.json', MAX_SPLIT_BYTES) as stream:
        _require(stream.read() == snapshot.split.payload and report['split_id'] == snapshot.split.split_id,
                 'Scores differ from the sealed split')
    queries = _by_keyframe(report['queries'], len(snapshot.split.frames), 'heldout query')
    expected = [key for key, frame in snapshot.split.frames.items() if frame['query']]
    _require(list(queries) == expected, 'Score queries differ from the frozen plan')
    return ScoredArchive(scores, snapshot, report, queries, digest, sources)


def _check_query(archive, key):
    row = archive.queries[key]
    snapshot = archive.snapshot
    frame = snapshot.split.frames[key]
    _require(row['status'] == 'succeeded', f'Query {key} has no scores. Exclude it explicitly to compare the other views')
    owner = int(frame['submap_id'])
    selected = row['selected_ids']
    _require(row['phase'] == 'final_snapshot' and row['split_id'] == snapshot.split.split_id
             and row['as_of'] == {name: snapshot.document[name] for name in _FRONTIER},
             'Query differs from the sealed frontier')
    _require(row['owner_submap_id'] == owner and 1 <= len(selected) <= 2
             and all(type(group) is int and 0 <= group < len(snapshot.document['entries']) for group in selected)
             and selected == sorted(set(selected)) and selected[-1] == owner
             and row['single'] is not None and (row['joint'] is not None) == (len(selected) == 2)
             and [source['submap_id'] for source in row['sources']] == selected,
             'Query selection differs from the frozen owner')
    for group, source in zip(selected, row['sources']):
        block = snapshot.document['entries'][group]
        _require(block['availability'] == 'current' and source['asset_id'] == block['asset']['asset_id']
                 and source['manifest_sha256'] == block['asset']['manifest_sha256']
                 and source['binding'] == block['binding'] and source['content'] == block['content']
                 and source['split_id'] == snapshot.split.split_id, 'Query sources differ from selected sealed blocks')
        path = snapshot.directory / 'states' / block['asset']['asset_id'] / 'manifest.json'
        with _open_file(path, _LIMITS['manifest.json']) as stream:
            payload = stream.read()
        _require(hashlib.sha256(payload).hexdigest() == block['asset']['manifest_sha256'], 'Sealed state manifest changed')
        proof = json.loads(payload, object_pairs_hook=_unique_object, parse_constant=_reject_constant)['holdout']
        _require(source['lineage_sha256'] == proof['lineage_sha256']
                 and source['training_lineage_ids'] == [int(item['keyframe_id']) for item in proof['lineage']],
                 'Query training lineage differs from the sealed model')
    camera = row['query']
    body_camera = np.asarray(frame['camera_from_body']).reshape(4, 4)
    materialized = snapshot.poses[key]
    _require(camera['calibration_id'] == int(frame['calibration_id']) and camera['capture_time'] == frame['image_timestamp']
             and camera['width'] == frame['width'] and camera['height'] == frame['height']
             and np.array_equal(np.asarray(camera['K']).ravel(), frame['intrinsics'])
             and np.array_equal(np.asarray(camera['camera_from_body']).reshape(4, 4), body_camera)
             and np.array_equal(np.asarray(camera['parent_from_body']).reshape(4, 4), materialized)
             and np.allclose(np.asarray(camera['camera_from_parent']).reshape(4, 4),
                             body_camera @ np.linalg.inv(materialized), atol=1e-9, rtol=1e-12),
             'Query camera differs from the frozen calibration or pose')
    return row


def _view(archives, key, pairs, stamps):
    samples, first = {}, None
    for label, archive in archives.items():
        row = _check_query(archive, key)
        if first is None:
            first = row
        _require(row['query'] == first['query'] and row['selected_ids'] == first['selected_ids'],
                 'Scored archives use different query cameras or submaps')
        path = archive.scores / f'views/{key}/samples.npz'
        stamps[path] = _stamp(path)
        samples[label] = _samples(archive.scores, archive.report, row, archive.snapshot.split.frames[key])
    archive = next(iter(archives.values()))
    reference = next(iter(samples.values()))
    image, labels = (reference[field].astype(bool) for field in ('image_mask', 'depth_mask'))
    z = reference['reference_depth_m']
    neighbors, votes = project_support(archive.snapshot, key, first['query'], z, image, labels)
    result = dict(query=first['query'], owner_submap_id=first['owner_submap_id'], selected_ids=first['selected_ids'],
                  neighbors=neighbors, reference_pixels=int(labels.sum()), modes={},
                  support={str(radius): {name: int(mask.sum()) for name, mask in support_masks(value).items()}
                           for radius, value in votes.items()})
    for mode in _MODES:
        if first[mode] is None:
            continue
        keep = labels & np.logical_and.reduce([value[mode+'_alpha'] >= _ALPHA_THRESHOLD for value in samples.values()])
        errors = {name: value[mode+'_depth_m'][labels].astype('f8')-z[labels] for name, value in samples.items()}
        result['modes'][mode] = compare_view(errors, pairs, keep[labels], votes)
        result['modes'][mode]['depth_mask_sha256'] = hashlib.sha256(keep.tobytes()).hexdigest()
    return result


def _settings(run_specs, compare_pairs, excluded, output):
    _require(1 <= len(run_specs) <= 4, 'Supply one to four scored snapshots')
    specs = {}
    for name, *paths in run_specs:
        _require(re.fullmatch(r'[A-Za-z0-9_.-]{1,32}', name) and name not in specs, 'Invalid or duplicate run name')
        specs[name] = tuple(Path(path).resolve(strict=True) for path in paths)
    pairs = {}
    for baseline, candidate in compare_pairs:
        label = baseline + ':' + candidate
        _require(baseline in specs and candidate in specs and baseline != candidate and label not in pairs,
                 'Invalid or duplicate comparison pair')
        pairs[label] = [baseline, candidate]
    _require(len(excluded) == len(set(excluded)) and all(key >= 0 for key in excluded), 'Invalid excluded query IDs')
    output = Path(output).absolute()
    _require(not os.path.lexists(output) and output.parent.is_dir(), 'Output file must be new with an existing parent')
    _require(not any(output.resolve().is_relative_to(path) for paths in specs.values() for path in paths),
             'Input and output paths must be separate')
    return specs, pairs, output


def run(args):
    specs, pairs, output = _settings(args.run, args.pair, args.exclude_query, args.output)
    archives = {label: load_scored_archive(*paths) for label, paths in specs.items()}
    baseline = next(iter(archives.values()))
    for archive in archives.values():
        _require(archive.snapshot.split.payload == baseline.snapshot.split.payload
                 and np.array_equal(archive.snapshot.poses, baseline.snapshot.poses),
                 'Compared archives must share their frozen split and materialized poses')
    _require(set(args.exclude_query) <= baseline.queries.keys(), 'Excluded query is not in the frozen plan')
    keys = [key for key in baseline.queries if key not in args.exclude_query]
    _require(keys, 'No heldout queries remain')
    stamps = {}
    views = {str(key): _view(archives, key, pairs, stamps) for key in keys}
    modes = {}
    for mode in _MODES:
        rows = [row['modes'][mode] for row in views.values() if mode in row['modes']]
        if rows:
            modes[mode] = summarize_mode(rows, pairs)
    runs = {name: dict(scores=str(archive.scores), scores_sha256=archive.report_sha256, source=archive.sources,
                       selection_policy=archive.report['selection_policy'])
            for name, archive in archives.items()}
    result = dict(schema=1, kind='livo_atlas.gaussian_depth_support_evaluation', status='succeeded',
                  split_id=baseline.snapshot.split.split_id, runs=runs, pairs=pairs, excluded_queries=sorted(args.exclude_query),
                  alpha_threshold=_ALPHA_THRESHOLD, radii_px=list(RADII), vote_fields=list(VOTE_FIELDS), groups=list(GROUPS),
                  neighbor_offsets=list(NEIGHBOR_OFFSETS), min_support_frames=MINIMUM_VOTES,
                  min_tolerance_m=MINIMUM_TOLERANCE_M, relative_tolerance=RELATIVE_TOLERANCE,
                  views=views, modes=modes)
    payload = _json_bytes(result, _REPORT_LIMIT)
    with TemporaryDirectory(prefix='.depth-support-', dir=output.parent) as temporary:
        pending = Path(temporary) / 'report.json'
        _exclusive_write(pending, payload)
        for archive in archives.values():
            verify_evaluation_snapshot(archive.snapshot)
            _require(_read_json(archive.scores / 'report.json')[1] == archive.report_sha256, 'Scores changed during evaluation')
            with _open_file(archive.scores / 'split.json', MAX_SPLIT_BYTES) as stream:
                _require(stream.read() == archive.snapshot.split.payload, 'Score split changed during evaluation')
        _require(all(_stamp(path) == expected for path, expected in stamps.items()), 'Samples changed during evaluation')
        os.link(pending, output)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='append', nargs=4, required=True, metavar=('NAME', 'MAP', 'KEYFRAMES', 'SCORES'))
    parser.add_argument('--pair', action='append', nargs=2, default=[], metavar=('BASELINE', 'CANDIDATE'))
    parser.add_argument('--exclude-query', action='append', type=int, default=[], metavar='ID')
    parser.add_argument('--output', required=True, help='Path to a new JSON report')
    try:
        run(parser.parse_args(argv))
    except (KeyError, TypeError, ValueError, OSError, OverflowError, np.linalg.LinAlgError) as error:
        print(f'Depth support evaluation failed: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
