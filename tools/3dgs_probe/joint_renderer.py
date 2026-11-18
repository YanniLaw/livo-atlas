"""Read-only final snapshots and bounded joint Gaussian inference.

CPU validation and geometry stay importable without the optional GPU runtime.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import stat
import time

import numpy as np

from gaussian_map import (_inspect_gaussian_map, _require,
                          inspect_gaussian_finalization)
from gaussian_state import (_PARAMETERS, _array, _open_file, _pose,
                            _reject_constant, _unique_object, _unsigned, read_state)


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


def _real_directory(path):
    identity = _stamp(path)
    _require(identity is not None and stat.S_ISDIR(identity[2]),
             'Required snapshot directory is missing or linked: ' + str(path))


def _check_frontier(keyframes, frames, version):
    """Compare the two supplied inputs, rather than trusting the map's own target."""
    frame_root = keyframes / 'frames'
    _real_directory(frame_root)
    paths = list(frame_root.iterdir())
    _require(len(paths) == frames and {_unsigned(path.name) for path in paths} == set(range(frames)),
             'Keyframe directory differs from the final map frontier')
    for path in paths:
        _real_directory(path)
    pose_root = keyframes / 'online_submaps/poses'
    latest = 0
    if _stamp(pose_root) is not None:
        _real_directory(pose_root)
        for path in pose_root.iterdir():
            _require(path.suffix == '.yml' and stat.S_ISREG(path.lstat().st_mode),
                     'Invalid pose history file')
            number = _unsigned(path.stem)
            _require(number > 0, 'Pose history versions must be positive')
            latest = max(latest, number)
    _require(latest == version, 'Keyframe pose version differs from the final map frontier')


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
    map_path = directory / 'map.json'
    with _open_file(map_path, 512 * 1024) as stream:
        raw = stream.read()
    document = json.loads(raw.decode('utf-8'), object_pairs_hook=_unique_object,
                          parse_constant=_reject_constant)
    frames, version = (_unsigned(document[key]) for key in ('frame_count', 'pose_version'))
    _require(keyframe_id < frames <= 512, 'Query frame is outside the snapshot frontier')
    fingerprint = _fingerprint(directory, keyframes, document)
    _require(fingerprint['digests'][str(map_path)] == hashlib.sha256(raw).hexdigest(),
             'Snapshot map changed while collecting its file identities')
    _check_frontier(keyframes, frames, version)
    # The legacy inspector also serves older exports without grouping records.
    # Rendering needs the authoritative directories so membership is monitored.
    records = keyframes / 'online_submaps/records'
    _real_directory(records)
    for row in document['entries']:
        _real_directory(records / str(_unsigned(row['submap_id'])))
    summary, data, poses, query = _inspect_gaussian_map(directory, keyframes, version, frames,
                                                       query_id=keyframe_id)
    _require(data == document, 'Snapshot map changed during validation')
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


def _rotation_quaternion(rotation):
    """Convert a proper rotation to wxyz, including rotations near 180 degrees."""
    r = rotation
    terms = [1 + np.trace(r), 1 + r[0, 0] - r[1, 1] - r[2, 2],
             1 - r[0, 0] + r[1, 1] - r[2, 2],
             1 - r[0, 0] - r[1, 1] + r[2, 2]]
    index = int(np.argmax(terms))
    value = 0.5 * np.sqrt(terms[index])
    divisor = 4 * value
    if index == 0:
        return np.array([value, (r[2, 1] - r[1, 2]) / divisor,
                         (r[0, 2] - r[2, 0]) / divisor,
                         (r[1, 0] - r[0, 1]) / divisor])
    if index == 1:
        return np.array([(r[2, 1] - r[1, 2]) / divisor, value,
                         (r[0, 1] + r[1, 0]) / divisor,
                         (r[0, 2] + r[2, 0]) / divisor])
    if index == 2:
        return np.array([(r[0, 2] - r[2, 0]) / divisor,
                         (r[0, 1] + r[1, 0]) / divisor, value,
                         (r[1, 2] + r[2, 1]) / divisor])
    return np.array([(r[1, 0] - r[0, 1]) / divisor,
                     (r[0, 2] + r[2, 0]) / divisor,
                     (r[1, 2] + r[2, 1]) / divisor, value])


def transform_parameters(parameters: dict, camera_from_submap: np.ndarray) -> dict:
    """Own a temporary SH0 parameter set in camera coordinates; leave raw data intact."""
    _require(isinstance(parameters, dict) and parameters.keys() == _PARAMETERS.keys(),
             'Unknown or missing Gaussian parameters')
    means = parameters['means']
    _require(isinstance(means, np.ndarray) and means.ndim == 2 and
             1 <= len(means) <= 10_000, 'Gaussian count exceeds the block budget')
    count = len(means)
    for name, columns in _PARAMETERS.items():
        _array(parameters[name], '<f4', (count, columns) if columns else (count,))
    _require(np.allclose(np.linalg.norm(parameters['quats'], axis=1), 1,
                         atol=1e-5, rtol=0), 'Gaussian quaternions must be normalized')
    scales = parameters['log_scales']
    _require(np.all(scales >= np.log(.001) - 1e-6) and
             np.all(scales <= np.log(.2) + 1e-6), 'Gaussian scales exceed model bounds')
    pose = _pose(camera_from_submap)
    left, _, right = np.linalg.svd(pose[:3, :3])
    correction = np.diag([1, 1, np.linalg.det(left @ right)])
    rotation = left @ correction @ right
    q = _rotation_quaternion(rotation)
    local = parameters['quats'].astype(np.float64)
    # Hamilton left product rotates both the center and anisotropic covariance.
    quats = np.empty_like(local)
    quats[:, 0] = q[0] * local[:, 0] - local[:, 1:] @ q[1:]
    quats[:, 1:] = (q[0] * local[:, 1:] + local[:, :1] * q[1:] +
                    np.cross(q[1:], local[:, 1:]))
    quats /= np.linalg.norm(quats, axis=1, keepdims=True)
    result = {name: value.copy() for name, value in parameters.items()}
    result['means'] = (means.astype(np.float64) @ rotation.T + pose[:3, 3]).astype('<f4')
    result['quats'] = quats.astype('<f4')
    _require(np.isfinite(result['means']).all(), 'Camera-relative means exceed float32')
    return result


def _infer(blocks: list[tuple], K: np.ndarray, width: int, height: int,
           device, standalone: bool) -> dict:
    """Rasterize one bounded selection together, returning only owned CPU results."""
    started = time.perf_counter()
    _require(isinstance(blocks, (list, tuple)) and 1 <= len(blocks) <= 2,
             'Select one or two Gaussian blocks')
    _require(type(width) is int and type(height) is int and width > 0 and height > 0
             and width * height <= 1_000_000, 'Camera exceeds the pixel budget')
    ids = [_id(block[0]) for block in blocks]
    _require(len(set(ids)) == len(ids), 'Duplicate Gaussian blocks')
    counts = {}
    transformed = []
    for submap_id, raw, camera in sorted(blocks, key=lambda block: block[0]):
        transformed.append(transform_parameters(raw, camera))
        counts[str(submap_id)] = len(raw['means'])
    packed = {key: np.concatenate([block[key] for block in transformed])
              for key in _PARAMETERS}
    del transformed
    _require(sum(counts.values()) <= 20_000, 'Joint Gaussian count exceeds its budget')

    # Importing or validating a snapshot never initializes the optional CUDA stack.
    import torch
    from rasterizer import render, to_gsplat_camera

    device = torch.device(device)
    _require(device == torch.device('cuda:0'), 'Rendering requires the selected CUDA0 device')
    intrinsic, view = to_gsplat_camera(K, np.eye(4))
    torch.cuda.synchronize(device)
    if standalone:
        torch.cuda.reset_peak_memory_stats(device)
    tensors, values = {}, None
    try:
        with torch.no_grad():
            tensors = {key: torch.as_tensor(value, device=device) for key, value in packed.items()}
            del packed
            torch.cuda.synchronize(device)
            uploaded = time.perf_counter()
            values = render(tensors['means'], tensors['quats'], tensors['log_scales'].exp(),
                            tensors['opacity_logits'].sigmoid(), tensors['color_logits'].sigmoid(),
                            view, intrinsic, width, height)
            torch.cuda.synchronize(device)
            rendered = time.perf_counter()
            result = {key: value.cpu().numpy().copy()
                      for key, value in zip(('rgb', 'alpha', 'depth_m'), values)}
            result.update(gaussian_counts=counts, gaussian_count=sum(counts.values()),
                          K_gsplat=intrinsic,
                          timings={'transform_upload_seconds': uploaded - started,
                                   'render_seconds': rendered - uploaded,
                                   'total_seconds': time.perf_counter() - started},
                          gpu_peak_allocated_bytes=torch.cuda.max_memory_allocated(device),
                          gpu_peak_reserved_bytes=torch.cuda.max_memory_reserved(device))
            return result
    finally:
        # Baseline and joint calls run sequentially, including when CUDA raises.
        values = None
        tensors.clear()
        if standalone:
            torch.cuda.empty_cache()


_ALPHA_THRESHOLD = .05


def image_metrics(result: dict, query: dict) -> dict:
    """Measure RGB on valid image pixels and camera z on covered sparse labels."""
    image = query['image_mask']
    covered = image & (result['alpha'] >= _ALPHA_THRESHOLD)
    available_depth = image & query['depth_mask']
    depth = available_depth & covered
    image_count, covered_count, depth_count = map(int, (image.sum(), covered.sum(), depth.sum()))
    error = result['rgb'][image].astype(np.float64) - query['rgb'][image]
    mse = float(np.mean(error**2)) if image_count else None
    z_error = result['depth_m'][depth].astype(np.float64) - query['depth_m'][depth]
    return dict(image_pixels=image_count, covered_pixels=covered_count,
                alpha_threshold=_ALPHA_THRESHOLD,
                alpha_coverage=covered_count / image_count if image_count else None,
                rgb_l1=float(np.mean(np.abs(error))) if image_count else None,
                rgb_mse=mse, rgb_perfect=mse == 0 if mse is not None else False,
                rgb_psnr_db=-10 * math.log10(mse) if mse is not None and mse > 0 else None,
                depth_pixels_available=int(available_depth.sum()), depth_pixels_used=depth_count,
                depth_l1_m=float(np.mean(np.abs(z_error))) if depth_count else None,
                depth_rmse_m=float(np.sqrt(np.mean(z_error**2))) if depth_count else None)


def render_blocks(blocks, K, width, height, device):
    """Standalone offline lifecycle, including its own allocator peak window."""
    return _infer(blocks, K, width, height, device, standalone=True)


def infer_blocks(blocks, K, width, height, device):
    """Serial worker inference; retain its allocator cache and cumulative peak."""
    return _infer(blocks, K, width, height, device, standalone=False)
