"""Independent frozen heldout cameras for finalized, proven online assets."""

from dataclasses import dataclass
import copy
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from evaluation_metrics import measure
from gaussian_map import _inspect_gaussian_map, _project, _require, inspect_gaussian_finalization
from gaussian_state import _open_file, _reject_constant, _unique_object, _unsigned, read_state
from holdout_split import HoldoutSplit, MAX_SPLIT_BYTES
from joint_renderer import (_check_frontier, _fingerprint, _real_directory,
                            _small_digest, _stamp, infer_blocks, verify_snapshot)
from prepare_holdout_split import file_sha


@dataclass
class EvaluationSnapshot:
    directory: Path
    keyframes: Path
    document: dict
    finalization: dict
    map_report: dict
    poses: list
    split: HoldoutSplit
    fingerprint: dict
    source_digests: dict


def verify_evaluation_snapshot(snapshot):
    verify_snapshot(snapshot)


def load_evaluation_snapshot(directory, keyframes):
    directory, keyframes = Path(directory).resolve(strict=True), Path(keyframes).resolve(strict=True)
    path = directory / 'map.json'
    with _open_file(path, 512 * 1024) as stream:
        raw = stream.read()
    document = json.loads(raw, object_pairs_hook=_unique_object, parse_constant=_reject_constant)
    _require(isinstance(document, dict), 'Invalid evaluation map header')
    frames, version = (_unsigned(document[key]) for key in ('frame_count', 'pose_version'))
    fingerprint = _fingerprint(directory, keyframes, document)
    _require(fingerprint['digests'][str(path)] == hashlib.sha256(raw).hexdigest(),
             'Evaluation map changed during capture')
    split_path = directory / 'split.json'
    fingerprint['stamps'][str(split_path)] = _stamp(split_path)
    with _open_file(split_path, MAX_SPLIT_BYTES) as stream:
        payload = stream.read()
    split = HoldoutSplit(payload)
    _require(len(split.frames) == frames, 'Frozen split differs from the final frontier')
    _check_frontier(keyframes, frames, version)
    for row in document['entries']:
        _real_directory(keyframes / 'online_submaps/records' / row['submap_id'])
    sources = {keyframes / 'dataset.yml': split.source['camera_sha256'],
               keyframes / 'online_submaps/dataset.yml': split.source['dataset_sha256']}
    for source in split.source['assets']:
        folder = keyframes / 'frames' / source['keyframe_id']
        for name, key in (('image.png', 'image_sha256'), ('cloud.pcd', 'cloud_sha256'),
                          ('frame.yml', 'record_sha256')):
            sources[folder / name] = source[key]
    for source, digest in sources.items():
        stamp = _stamp(source)
        _require(stamp is not None and 0 < stamp[3] <= 16 * 1024 * 1024,
                 'Evaluation source exceeds its byte budget')
        fingerprint['stamps'][str(source)] = stamp
        _require(file_sha(source) == digest, 'Evaluation source differs from frozen split: ' + str(source))
    summary, data, poses, _ = _inspect_gaussian_map(
        directory, keyframes, version, frames, state_validator=split.check_state)
    _require(data == document, 'Evaluation map changed during verification')
    for row in data['entries']:
        first, count = int(row['anchor_keyframe_id']), row['primary_count']
        _require(all(split.frames[key]['submap_id'] == row['submap_id']
                     for key in range(first, first+count)),
                 'Frozen ownership differs from the complete final primary partition')
    finalization = inspect_gaussian_finalization(directory, summary)
    snapshot = EvaluationSnapshot(directory, keyframes, data, finalization, summary, poses,
                                  split, fingerprint, {str(p): digest for p, digest in sources.items()})
    verify_evaluation_snapshot(snapshot)
    return snapshot


def canonical_query(split, view):
    query = dict(view)
    # Hashing and scoring use the same owned canonical values. A caller's f8
    # label must not pass an f4 digest yet change the reported error afterward.
    for name in ('rgb', 'depth_m'):
        query[name] = np.asarray(query[name]).astype('<f4', copy=True)
    for name in ('image_mask', 'depth_mask'):
        mask = np.asarray(query[name])
        _require(mask.dtype in (np.dtype('bool'), np.dtype('u1')) and np.all(mask <= 1),
                 'Invalid frozen query mask')
        query[name] = mask.astype(bool, copy=True)
    if 'camera_from_body' not in query:
        query['camera_from_body'] = query['camera_from_parent'] @ _project(query['parent_from_body'])
    key = split.check_query(query)
    frame = split.frames[key]
    query['K'] = np.asarray(frame['intrinsics'], dtype='f8').reshape(3, 3).copy()
    query['camera_from_body'] = np.asarray(frame['camera_from_body'], dtype='f8').reshape(4, 4).copy()
    return query


def prepare_query(snapshot, view):
    verify_evaluation_snapshot(snapshot)
    query = canonical_query(snapshot.split, view)
    key = query['keyframe_id']
    owner = int(snapshot.split.frames[key]['submap_id'])
    query['camera_from_parent'] = query['camera_from_body'] @ np.linalg.inv(snapshot.poses[key])
    return query, owner, query['camera_from_parent']


def _block(snapshot, group, camera):
    _require(type(group) is int and 0 <= group < len(snapshot.document['entries']), 'Unknown evaluation submap')
    row = snapshot.document['entries'][group]
    _require('asset' in row and row['availability'] == 'current', 'Evaluation submap has no current model')
    path = snapshot.directory / 'states' / row['asset']['asset_id']
    _require(_small_digest(path / 'manifest.json') == row['asset']['manifest_sha256'],
             'Evaluation asset differs from its snapshot')
    state = read_state(path)
    snapshot.split.check_state(state)
    proof = state.metadata['holdout']
    source = dict(submap_id=group, asset_id=row['asset']['asset_id'],
                  manifest_sha256=row['asset']['manifest_sha256'], content=dict(row['content']),
                  binding=copy.deepcopy(row['binding']), split_id=proof['split_id'],
                  lineage_sha256=proof['lineage_sha256'],
                  training_lineage_ids=[int(item['keyframe_id']) for item in proof['lineage']])
    parameters = state.parameters
    del state
    transform = camera @ np.asarray(row['binding']['parent_from_submap']).reshape(4, 4)
    return (group, parameters, transform), source


def render_query(snapshot, view, selected_ids, device, *, renderer=infer_blocks):
    started = time.perf_counter()
    query, owner, camera = prepare_query(snapshot, view)
    selected = sorted(selected_ids)
    _require(1 <= len(selected) <= 2 and len(set(selected)) == len(selected) and owner in selected,
             'Evaluation requires one or two unique blocks including its owner')
    loaded = [_block(snapshot, group, camera) for group in selected]
    blocks, sources = [x[0] for x in loaded], [x[1] for x in loaded]
    verify_evaluation_snapshot(snapshot)
    height, width = query['rgb'].shape[:2]
    own = next(block for block in blocks if block[0] == owner)
    single = renderer([own], query['K'], width, height, device)
    single_metrics = measure(single, query)
    joint = renderer(blocks, query['K'], width, height, device) if len(selected) == 2 else None
    joint_metrics = measure(joint, query) if joint is not None else None
    verify_evaluation_snapshot(snapshot)
    report = dict(keyframe_id=query['keyframe_id'], owner_submap_id=owner, phase='final_snapshot',
                  split_id=snapshot.split.split_id, selected_ids=selected, sources=sources,
                  as_of={key: snapshot.document[key] for key in ('session_id', 'map_sequence',
                      'pose_version', 'frame_count', 'optimized_frame_count', 'parent_frame')},
                  query=dict(calibration_id=query['calibration_id'], width=width, height=height,
                             K=query['K'].tolist(), camera_from_body=query['camera_from_body'].ravel().tolist(),
                             parent_from_body=snapshot.poses[query['keyframe_id']].ravel().tolist(),
                             camera_from_parent=camera.ravel().tolist(),
                             capture_time=query['image_timestamp']),
                  single=single_metrics, joint=joint_metrics, evaluation_seconds=time.perf_counter()-started)
    return dict(report=report, query=query, single=single, joint=joint)
