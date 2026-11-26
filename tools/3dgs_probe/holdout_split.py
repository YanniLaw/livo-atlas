"""Frozen Gaussian observation identities and complete training provenance (CPU)."""

import hashlib
import json
import math
import re

import numpy as np


MAX_SPLIT_BYTES = 1024 * 1024
CAPABILITY = 'holdout_split_v1'
_TIMES = ('state_timestamp', 'cloud_timestamp', 'image_timestamp', 'inverse_exposure_time')
_HASH = re.compile(r'[0-9a-f]{64}')
_SOURCE = {'dataset_sha256', 'camera_sha256', 'input_sha256', 'preparer_sha256',
           'frontend_profile', 'assets'}
_FIELDS = {'keyframe_id', 'calibration_id', 'submap_id', 'role', 'query',
           'width', 'height', 'intrinsics', 'camera_from_body', 'rgb_sha256',
           'rgb_float_sha256',
           'depth_sha256', 'image_mask_sha256', 'depth_mask_sha256',
           'observation_sha256'} | set(_TIMES)


def canonical_json(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'),
                      ensure_ascii=True, allow_nan=False).encode('ascii')


def _sha(value):
    return hashlib.sha256(value).hexdigest()


def _id(value):
    if (not isinstance(value, str) or not re.fullmatch(r'0|[1-9][0-9]*', value) or
            len(value) > 20 or int(value) >= 2**64):
        raise ValueError('Expected a canonical uint64 identity')
    return int(value)


def _array(value):
    # State views may be GPU tensors; the admission check itself never changes them.
    if hasattr(value, 'detach'):
        value = value.detach().cpu().numpy()
    return np.asarray(value)


def _calibration(value, count, decimals):
    result = np.round(_array(value).reshape(count), decimals)
    # Canonical JSON distinguishes -0.0 from +0.0, although the camera does not.
    result[result == 0] = 0.0
    return result.tolist()


def observation_record(view, submap_id, role):
    rgb = _array(view['rgb'])
    if rgb.ndim != 3 or rgb.shape[2] != 3 or not np.isfinite(rgb).all():
        raise ValueError('Invalid observation RGB')
    height, width = rgb.shape[:2]
    if not 0 < width * height <= 1_000_000 or np.any(rgb < 0) or np.any(rgb > 1):
        raise ValueError('Observation exceeds image bounds')
    result = dict(keyframe_id=str(view['keyframe_id']),
                  calibration_id=str(view['calibration_id']), submap_id=submap_id,
                  role=role, query=False, width=width, height=height)
    for name in _TIMES:
        result[name] = float(view[name])
        if not math.isfinite(result[name]):
            raise ValueError('Nonfinite observation time')
    result['intrinsics'] = _calibration(view['K'], 9, 10)
    result['camera_from_body'] = _calibration(view['camera_from_body'], 16, 8)
    result['rgb_sha256'] = _sha(np.rint(rgb * 255).astype('u1').tobytes())
    result['rgb_float_sha256'] = _sha(rgb.astype('<f4').tobytes())
    for field, dtype in (('depth_m', '<f4'), ('image_mask', 'u1'), ('depth_mask', 'u1')):
        array = _array(view[field])
        if array.shape != (height, width) or not np.isfinite(array).all():
            raise ValueError('Invalid observation array')
        key = 'depth' if field == 'depth_m' else field
        result[key + '_sha256'] = _sha(array.astype(dtype).tobytes())
    identity = {k: v for k, v in result.items() if k not in ('role', 'query', 'submap_id')}
    result['observation_sha256'] = _sha(canonical_json(identity))
    return result


def build_manifest(source, frames):
    # Recompute query selection before hashing; training roles are frozen by ID.
    frames = [dict(frame, query=False) for frame in frames]
    queries = {}
    for frame in frames:
        if frame['role'] == 'heldout':
            count = queries.get(frame['submap_id'], 0)
            frame['query'] = count < 2
            queries[frame['submap_id']] = count + 1
    manifest = dict(schema=1, kind='livo_atlas.gaussian_split', source=dict(source), frames=frames)
    manifest['split_id'] = _sha(canonical_json(manifest))
    HoldoutSplit(canonical_json(manifest))
    return manifest


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('Duplicate JSON field')
        result[key] = value
    return result


class HoldoutSplit:
    def __init__(self, payload):
        if not isinstance(payload, bytes) or not 0 < len(payload) <= MAX_SPLIT_BYTES:
            raise ValueError('Split exceeds the manifest budget')
        data = json.loads(payload, object_pairs_hook=_unique_object)
        if (not isinstance(data, dict) or set(data) != {'schema', 'kind', 'source', 'frames', 'split_id'} or
                type(data['schema']) is not int or data['schema'] != 1 or
                data['kind'] != 'livo_atlas.gaussian_split'):
            raise ValueError('Unsupported Gaussian split schema')
        self.split_id = data['split_id']
        if self.split_id != _sha(canonical_json({k: v for k, v in data.items() if k != 'split_id'})):
            raise ValueError('Gaussian split digest mismatch')
        source = data['source']
        if not isinstance(source, dict) or set(source) != _SOURCE:
            raise ValueError('Incomplete split source')
        self.source = source
        if (not isinstance(source['frontend_profile'], str) or not 1 <= len(source['frontend_profile']) <= 256 or
                any(not isinstance(source[k], str) or not _HASH.fullmatch(source[k])
                    for k in _SOURCE - {'frontend_profile', 'assets'})):
            raise ValueError('Invalid split source identity')
        frames = data['frames']
        if not isinstance(frames, list) or not 1 <= len(frames) <= 512:
            raise ValueError('Split must cover 1 to 512 frames')
        assets = source['assets']
        if not isinstance(assets, list) or len(assets) != len(frames):
            raise ValueError('Split requires per-frame source file digests')
        for index, item in enumerate(assets):
            if (not isinstance(item, dict) or set(item) != {'keyframe_id', 'image_sha256',
                    'cloud_sha256', 'record_sha256'} or item['keyframe_id'] != str(index) or
                    any(not isinstance(item[k], str) or not _HASH.fullmatch(item[k])
                        for k in item if k != 'keyframe_id')):
                raise ValueError('Invalid per-frame source file identity')
        self.frames = {}
        source_images = {}
        seen = {field: {} for field in ('keyframe_id', 'image_timestamp', 'rgb_sha256')}
        query_counts, last_group = {}, -1
        for index, frame in enumerate(frames):
            if not isinstance(frame, dict) or set(frame) != _FIELDS or _id(frame['keyframe_id']) != index:
                raise ValueError('Split must be a complete ordered prefix')
            group = _id(frame['submap_id'])
            _id(frame['calibration_id'])
            expected = 'heldout' if index % 8 == 1 else 'train'
            query = expected == 'heldout' and query_counts.get(group, 0) < 2
            if (frame['role'] != expected or type(frame['query']) is not bool or frame['query'] != query or
                    group not in (last_group, last_group + 1)):
                raise ValueError('Invalid frozen role, query or contiguous submap membership')
            last_group = group
            raw_image = assets[index]['image_sha256']
            if raw_image in source_images and source_images[raw_image] != expected:
                raise ValueError('Train/heldout source images overlap')
            source_images[raw_image] = expected
            if expected == 'heldout':
                query_counts[group] = query_counts.get(group, 0) + 1
            for field in ('width', 'height'):
                if type(frame[field]) is not int or frame[field] <= 0:
                    raise ValueError('Invalid split image dimensions')
            if frame['width'] * frame['height'] > 1_000_000:
                raise ValueError('Split image budget exceeded')
            for field in _TIMES:
                if type(frame[field]) not in (int, float) or not math.isfinite(frame[field]):
                    raise ValueError('Invalid split timestamp')
            for field, count in (('intrinsics', 9), ('camera_from_body', 16)):
                if (not isinstance(frame[field], list) or len(frame[field]) != count or
                        any(type(x) not in (int, float) or not math.isfinite(x) for x in frame[field])):
                    raise ValueError('Invalid split calibration')
            for field in _FIELDS:
                if field.endswith('_sha256') and (not isinstance(frame[field], str) or
                                                  not _HASH.fullmatch(frame[field])):
                    raise ValueError('Invalid observation digest')
            identity = {k: v for k, v in frame.items()
                        if k not in ('role', 'query', 'submap_id', 'observation_sha256')}
            if frame['observation_sha256'] != _sha(canonical_json(identity)):
                raise ValueError('Observation digest mismatch')
            for field, values in seen.items():
                value = frame[field]
                if value in values and values[value] != expected:
                    raise ValueError('Train/heldout identities overlap: ' + field)
                values[value] = expected
            self.frames[index] = frame
        self.payload = canonical_json(data)

    def _check_observation(self, view, role, selected=False):
        key = view['keyframe_id']
        frame = self.frames.get(key)
        if frame is None or frame['role'] != role or (selected and not frame['query']):
            raise ValueError('Gaussian observation has an invalid role or is not_in_scope')
        if 'submap_id' in view and view['submap_id'] != frame['submap_id']:
            raise ValueError('Gaussian observation has a different primary group')
        actual = observation_record(view, frame['submap_id'], role)
        if actual['observation_sha256'] != frame['observation_sha256']:
            raise ValueError('Gaussian observation differs from frozen split')
        return key

    def check_view(self, view):
        return self._check_observation(view, 'train')

    def check_query(self, view):
        return self._check_observation(view, 'heldout', selected=True)

    def check_target(self, target):
        frontier = _id(target['frame_count'])
        expected = [str(key) for key, frame in self.frames.items()
                    if key < frontier and frame['submap_id'] == target['submap_id']]
        actual = [entry['keyframe_id'] for entry in target['primary']]
        if (not 0 < frontier <= len(self.frames) or not expected or actual != expected or
                target['anchor_keyframe_id'] != expected[0]):
            raise ValueError('Gaussian target differs from frozen primary membership')

    def proof(self, ids):
        lineage = []
        for key in sorted(set(ids)):
            frame = self.frames.get(key)
            if type(key) is not int or frame is None or frame['role'] != 'train':
                raise ValueError('Training lineage contains heldout or unknown observations')
            lineage.append(dict(keyframe_id=str(key), observation_sha256=frame['observation_sha256']))
        if not lineage:
            raise ValueError('A trained state requires a nonempty lineage')
        return dict(split_id=self.split_id, lineage=lineage,
                    lineage_sha256=_sha(canonical_json(lineage)))

    def check_proof(self, proof):
        if not isinstance(proof, dict) or set(proof) != {'split_id', 'lineage', 'lineage_sha256'}:
            raise ValueError('Missing Gaussian training lineage proof')
        if not isinstance(proof['lineage'], list) or not 1 <= len(proof['lineage']) <= 512:
            raise ValueError('Invalid Gaussian training lineage')
        expected = self.proof([_id(entry['keyframe_id']) for entry in proof['lineage']])
        if proof != expected:
            raise ValueError('Gaussian training lineage does not match the frozen split')

    def check_state(self, state):
        if state.metadata.get('schema') != 2:
            raise ValueError('Legacy Gaussian state has no holdout proof')
        proof = state.metadata.get('holdout')
        self.check_proof(proof)
        ids = {int(entry['keyframe_id']) for entry in proof['lineage']}
        if not set(map(int, state.origin_ids)) <= ids:
            raise ValueError('Gaussian origins exceed the proven training lineage')
        for view in state.views:
            if self.check_view(view) not in ids:
                raise ValueError('Retained observation is absent from training lineage')
        self.check_target(state.metadata)
