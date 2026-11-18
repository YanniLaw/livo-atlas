"""Read-only final snapshots and bounded joint Gaussian inference.

CPU validation and geometry stay importable without the optional GPU runtime.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
from pathlib import Path

import numpy as np

from gaussian_map import (_inspect_gaussian_map, _json, _require,
                          inspect_gaussian_finalization)
from gaussian_state import _open_file, _unsigned, read_state


@dataclass
class RenderSnapshot:
    directory: Path
    keyframes: Path
    document: dict
    finalization: dict
    map_report: dict
    poses: list[np.ndarray]
    query: dict
    query_submap_id: int
    selected_ids: tuple[int, ...]
    camera_from_parent: np.ndarray
    fingerprint: dict


def _id(value):
    _require(type(value) is int and 0 <= value < 2**64, 'Invalid uint64 ID')
    return value


def _stamp(path):
    try:
        info = path.lstat()
    except FileNotFoundError:
        return None
    return (info.st_dev, info.st_ino, info.st_mode, info.st_size,
            info.st_mtime_ns, info.st_ctime_ns)


def _small_digest(path):
    with _open_file(path, 512 * 1024) as stream:
        return hashlib.sha256(stream.read()).hexdigest()


def _fingerprint(directory, keyframes, document):
    """Track the bounded inputs and directories that can change membership."""
    frames = _unsigned(document['frame_count'])
    rows = document['entries']
    _require(frames <= 512 and isinstance(rows, list) and len(rows) <= 512,
             'Snapshot frontier exceeds its budget')
    paths = {directory, directory / 'states', keyframes, keyframes / 'frames',
             keyframes / 'online_submaps', keyframes / 'online_submaps/records',
             keyframes / 'online_submaps/poses'}
    small = [directory / name for name in ('map.json', 'finalization.json', 'status.json')]
    small.append(keyframes / 'dataset.yml')
    small.extend(keyframes / 'frames' / str(i) / 'frame.yml' for i in range(frames))
    version = _unsigned(document['pose_version'])
    if version:
        small.append(keyframes / 'online_submaps/poses' / (str(version) + '.yml'))
    for row in rows:
        submap = _unsigned(row['submap_id'])
        records = keyframes / 'online_submaps/records' / str(submap)
        paths.add(records)
        if records.is_dir():
            small.append(records / (str(_unsigned(row['membership_revision'])) + '.yml'))
        if 'asset' in row:
            asset = directory / 'states' / str(_unsigned(row['asset']['asset_id']))
            paths.add(asset)
            paths.update(asset / name for name in ('manifest.json', 'gaussians.npz', 'observations.npz'))
    paths.update(small)
    return {'stamps': {str(path): _stamp(path) for path in paths},
            'digests': {str(path): _small_digest(path) for path in small}}


def verify_snapshot(snapshot: RenderSnapshot) -> None:
    for name, expected in snapshot.fingerprint['stamps'].items():
        _require(_stamp(Path(name)) == expected, 'Snapshot source changed: ' + name)
    for name, expected in snapshot.fingerprint['digests'].items():
        _require(_small_digest(Path(name)) == expected, 'Snapshot source changed: ' + name)


def load_snapshot(directory: Path, keyframes: Path, keyframe_id: int,
                  submap_ids: list[int]) -> RenderSnapshot:
    keyframe_id = _id(keyframe_id)
    _require(isinstance(submap_ids, (list, tuple)) and 1 <= len(submap_ids) <= 2,
             'Select one or two submaps')
    selected = tuple(sorted(_id(value) for value in submap_ids))
    _require(len(set(selected)) == len(selected), 'Duplicate submap IDs')
    directory, keyframes = Path(directory).resolve(strict=True), Path(keyframes).resolve(strict=True)
    document = _json(directory / 'map.json')
    frames, version = (_unsigned(document[key]) for key in ('frame_count', 'pose_version'))
    _require(keyframe_id < frames <= 512, 'Query frame is outside the snapshot frontier')
    fingerprint = _fingerprint(directory, keyframes, document)
    summary, data, poses, query = _inspect_gaussian_map(directory, keyframes, version, frames,
                                                       query_id=keyframe_id)
    finalization = inspect_gaussian_finalization(directory, summary)
    _require(summary['verified_assets'] > 0, 'Snapshot has no confirmed content')
    for submap in selected:
        _require(submap < len(data['entries']) and 'asset' in data['entries'][submap] and
                 data['entries'][submap]['availability'] == 'current', 'Selected submap is missing')
    _require(query is not None, 'Query frame is not retained in the snapshot')
    owner, view = query
    # The saved camera_from_parent is local to its capture submap. Keep the
    # final global camera separately so a caller cannot accidentally reuse it.
    view.pop('camera_from_parent')
    camera = view['camera_from_body'] @ np.linalg.inv(poses[keyframe_id])
    snapshot = RenderSnapshot(directory, keyframes, data, finalization, summary, poses, view,
                              owner, selected, camera, fingerprint)
    verify_snapshot(snapshot)
    return snapshot


def load_block(snapshot: RenderSnapshot, submap_id: int) -> tuple:
    submap_id = _id(submap_id)
    _require(submap_id < len(snapshot.document['entries']), 'Unknown submap')
    verify_snapshot(snapshot)
    row = snapshot.document['entries'][submap_id]
    _require('asset' in row and row['availability'] == 'current', 'Submap has no current asset')
    path = snapshot.directory / 'states' / row['asset']['asset_id']
    _require(_small_digest(path / 'manifest.json') == row['asset']['manifest_sha256'],
             'Reloaded asset differs from its snapshot reference')
    state = read_state(path)
    parameters = state.parameters
    # Only the small parameter arrays survive; views never reach CUDA.
    del state
    camera_from_submap = snapshot.camera_from_parent @ np.asarray(
        row['binding']['parent_from_submap'], dtype=np.float64).reshape(4, 4)
    verify_snapshot(snapshot)
    return submap_id, parameters, camera_from_submap
