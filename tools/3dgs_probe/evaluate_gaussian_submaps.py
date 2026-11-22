"""Evaluate Gaussian submaps on held-out views of a finalized SLAM snapshot."""
import hashlib
import math
from pathlib import Path
import subprocess

import numpy as np

from bundle import load_bundle
from gaussian_map import (_inspect_gaussian_map, _json, _transform, _yaml,
                          inspect_gaussian_finalization)
from gaussian_state import _unsigned
from joint_renderer import _check_frontier, _real_directory


_MAX_FRAMES = 512
_MAX_PRIMARY = 32
_MAX_PIXELS = 1_000_000
_MAX_CLOUD_POINTS = 500_000


def split_primary_ids(first: int, count: int) -> tuple[list[int], list[int]]:
    if (type(first) is not int or type(count) is not int or first < 0 or
            not 3 <= count <= _MAX_PRIMARY or first + count > _MAX_FRAMES):
        raise ValueError(f'Primary group at frame {first} requires 3 to 32 covered frames')
    held_out = [first + 1]
    if count >= 6:
        held_out.append(first + count // 2)
    return [key for key in range(first, first + count) if key not in held_out], held_out


def camera_from_final_parent(view: dict, final_parent_from_body: np.ndarray) -> np.ndarray:
    # The exporter projects depth in the fixed body frame. Only the pose changes.
    camera_from_body = view['camera_from_parent'] @ view['parent_from_body']
    return camera_from_body @ np.linalg.inv(final_parent_from_body)


def file_sha256(path):
    if path.is_symlink() or not path.is_file():
        raise ValueError(f'Expected a regular input file at {path}')
    digest = hashlib.sha256()
    with path.open('rb') as source:
        for block in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def check_split_identity(observations):
    for field in ('keyframe_id', 'capture_time', 'image_sha256'):
        seen = {}
        for observation in observations:
            previous = seen.setdefault(observation[field], observation)
            if previous['split'] != observation['split']:
                raise ValueError(f"Training and held-out keyframes {previous['keyframe_id']} "
                                 f"and {observation['keyframe_id']} share {field}")


def _group_splits(group):
    return {key: split for split, ids in (('train', group['train_ids']),
                                         ('validation', group['held_out_ids'])) for key in ids}


def freeze_split(directory: Path, keyframes: Path, prepare: Path) -> tuple[dict, list]:
    map_hashes = {name: file_sha256(directory / name)
                  for name in ('map.json', 'status.json', 'finalization.json')}
    expected = _json(directory / 'map.json')
    version = _unsigned(expected['pose_version'])
    frames = _unsigned(expected['frame_count'])
    if not 1 <= frames <= _MAX_FRAMES:
        raise ValueError('Expected 1 to 512 frames in the finalized dataset')
    _check_frontier(keyframes, frames, version)
    records = keyframes / 'online_submaps/records'
    _real_directory(keyframes / 'online_submaps')
    _real_directory(records)
    summary, document, poses, _ = _inspect_gaussian_map(directory, keyframes, version, frames)
    if document != expected:
        raise ValueError('Gaussian map changed during input validation')
    inspect_gaussian_finalization(directory, summary)

    paths = [Path('dataset.yml')]
    if version:
        paths.append(Path('online_submaps/poses') / f'{version}.yml')
    groups, observations = [], []
    for row in document['entries']:
        submap_id = _unsigned(row['submap_id'])
        anchor = _unsigned(row['anchor_keyframe_id'])
        revision = _unsigned(row['membership_revision'])
        train, held = split_primary_ids(anchor, row['primary_count'])
        _real_directory(records / str(submap_id))
        paths.append(Path('online_submaps/records') / str(submap_id) / f'{revision}.yml')
        group = dict(submap_id=submap_id, anchor_id=anchor, membership_revision=revision,
                     frame_count=frames, train_ids=train, held_out_ids=held)
        groups.append(group)
        for key, split in sorted(_group_splits(group).items()):
            frame = Path('frames') / str(key)
            metadata = _yaml(keyframes / frame / 'frame.yml')
            if metadata['has_image'] != 1:
                raise ValueError(f'Keyframe {key} has no image')
            capture_time = float(metadata['image_timestamp'])
            if not math.isfinite(capture_time):
                raise ValueError(f'Keyframe {key} has an invalid capture time')
            if _unsigned(metadata['cloud_points']) > _MAX_CLOUD_POINTS:
                raise ValueError(f'Keyframe {key} exceeds the point budget')
            paths.extend(frame / name for name in ('frame.yml', 'image.png', 'cloud.pcd'))
            observations.append(dict(keyframe_id=key, split=split, capture_time=capture_time))

    frame_hashes = {str(path): file_sha256(keyframes / path) for path in sorted(paths)}
    for row in observations:
        row['image_sha256'] = frame_hashes[f"frames/{row['keyframe_id']}/image.png"]
    check_split_identity(observations)
    split = dict(schema=1, session_id=document['session_id'], pose_version=version,
                 parent_frame=document['parent_frame'], frame_count=frames,
                 optimized_frame_count=_unsigned(document['optimized_frame_count']),
                 map_sequence=summary['map_sequence'], groups=groups, observations=observations,
                 keyframe_sha256=frame_hashes, gaussian_sha256=map_hashes,
                 prepare_executable_sha256=file_sha256(prepare))
    verify_split_sources(split, directory, keyframes, prepare)
    return split, poses


def verify_split_sources(split, directory, keyframes, prepare):
    for root, hashes in ((directory, split['gaussian_sha256']), (keyframes, split['keyframe_sha256'])):
        for name, digest in hashes.items():
            if file_sha256(root / name) != digest:
                raise ValueError(f'Input changed during evaluation at {root / name}')
    if file_sha256(prepare) != split['prepare_executable_sha256']:
        raise ValueError('Preparation executable changed during evaluation')


def read_prepared_group(keyframes, output, group, source_sha256):
    manifest = _json(output / 'bundle.json')
    entries = manifest.get('views')
    expected = _group_splits(group)
    if (manifest.get('source_manifest_sha256') != source_sha256 or
            not isinstance(entries, list) or len(entries) != len(expected) or
            len(entries) > _MAX_PRIMARY):
        raise ValueError(f"Prepared bundle differs from submap {group['submap_id']} split")
    seen = set()
    for entry in entries:
        key = _unsigned(entry['keyframe_id'])
        width, height = entry['width'], entry['height']
        if (key in seen or key not in expected or entry['split'] != expected[key] or
                _unsigned(entry['frame_count']) != group['frame_count'] or
                entry['pose_version'] != '0' or 'parent_from_body' not in entry or
                type(width) is not int or type(height) is not int or
                width <= 0 or height <= 0 or width * height > _MAX_PIXELS):
            raise ValueError(f'Prepared view {key} differs from the frozen split or raw pose frontier')
        seen.add(key)

    views = {view['keyframe_id']: view for view in load_bundle(output)}
    for key, view in views.items():
        raw = _yaml(keyframes / 'frames' / str(key) / 'frame.yml')
        if (view['image_timestamp'] != float(raw['image_timestamp']) or
                view['calibration_id'] != _unsigned(raw['calibration_id']) or
                not np.allclose(view['parent_from_body'], _transform(raw['odom_from_body']),
                                atol=1e-12, rtol=0)):
            raise ValueError(f'Prepared view {key} does not match its source observation')
        if not view['image_mask'].any() or (view['split'] == 'train' and not view['depth_mask'].any()):
            raise ValueError(f'Keyframe {key} lacks usable image or training depth')
    return views


def prepare_group(prepare, keyframes, output, group, source_sha256):
    subprocess.run([str(prepare), '--dataset', str(keyframes), '--output', str(output),
                    '--train-ids', ','.join(map(str, group['train_ids'])),
                    '--validation-ids', ','.join(map(str, group['held_out_ids'])),
                    '--source-sha256', source_sha256, '--max-pixels', str(_MAX_PIXELS),
                    '--max-cloud-points', str(_MAX_CLOUD_POINTS)], check=True)
    return read_prepared_group(keyframes, output, group, source_sha256)
