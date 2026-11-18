"""Bounded, content-bound descriptions for CPU spatial render selection."""
import math

import numpy as np

from online_protocol import _SESSION, _integer, _number, _pose, _unsigned


_CONTENT = {'session_id', 'submap_id', 'source_generation', 'content_job_id'}
_CAMERA = {'keyframe_id', 'calibration_id', 'width', 'height', 'K',
           'camera_from_body', 'capture_time'}


def _require(condition, detail):
    if not condition:
        raise ValueError(detail)


def _keys(value, keys):
    _require(isinstance(value, dict) and value.keys() == keys, 'Invalid spatial metadata fields')


def validate_metadata(metadata, content, frame_ids):
    """Check identity and coverage before consuming bounded numeric arrays."""
    _keys(metadata, {'content', 'extents', 'camera'})
    actual = metadata['content']
    _keys(actual, _CONTENT)
    _require(actual == content and isinstance(actual['session_id'], str) and
             _SESSION.fullmatch(actual['session_id']), 'Spatial metadata content differs')
    _unsigned(actual['submap_id'], 'metadata submap')
    for key in ('source_generation', 'content_job_id'):
        _require(_unsigned(actual[key], key) > 0, 'Spatial content identity must be positive')
    _require(isinstance(frame_ids, (list, tuple)) and 1 <= len(frame_ids) <= 10 and
             all(type(key) is int and 0 <= key < 512 for key in frame_ids) and
             list(frame_ids) == sorted(set(frame_ids)), 'Invalid spatial retained coverage')
    extents = metadata['extents']
    _require(isinstance(extents, list) and len(extents) == len(frame_ids),
             'Spatial origins must cover every retained frame')
    for extent, origin in zip(extents, frame_ids):
        _keys(extent, {'origin_keyframe_id', 'min', 'max', 'max_scale'})
        _require(_unsigned(extent['origin_keyframe_id'], 'origin') == origin,
                 'Spatial origins differ from retained coverage')
        for key in ('min', 'max'):
            _require(isinstance(extent[key], list) and len(extent[key]) == 3,
                     'Expected a three-dimensional Gaussian bound')
            for value in extent[key]:
                _number(value, 'Gaussian bound')
        _require(all(low <= high for low, high in zip(extent['min'], extent['max'])) and
                 _number(extent['max_scale'], 'maximum scale') > 0, 'Invalid Gaussian extent')
    camera = metadata['camera']
    _keys(camera, _CAMERA)
    _require(_unsigned(camera['keyframe_id'], 'query') == frame_ids[-1] and
             _unsigned(camera['calibration_id'], 'calibration') == 0,
             'Spatial query must be the latest retained camera')
    width = _integer(camera['width'], 'width', 1, 1_000_000)
    height = _integer(camera['height'], 'height', 1, 1_000_000)
    _require(width * height <= 1_000_000, 'Spatial camera exceeds its pixel budget')
    intrinsic = camera['K']
    _require(isinstance(intrinsic, list) and len(intrinsic) == 9, 'Invalid spatial K shape')
    for value in intrinsic:
        _number(value, 'camera intrinsic')
    _require(intrinsic[0] > 0 and intrinsic[4] > 0 and
             all(intrinsic[i] == 0 for i in (1, 3, 6, 7)) and intrinsic[8] == 1,
             'Expected the original pinhole K')
    _pose(camera['camera_from_body'], 'retained body camera')
    _number(camera['capture_time'], 'camera capture time')


def extract_metadata(state, content):
    """Read the candidate CPU copy; never scan observations or copy a model."""
    frames = [view['keyframe_id'] for view in state.views]
    _require(1 <= len(frames) <= 10 and frames == sorted(set(frames)),
             'Invalid retained metadata views')
    _require(content['session_id'] == state.metadata['origin_session_id'] and
             content['submap_id'] == state.metadata['submap_id'] and
             content['source_generation'] == state.metadata['generation'],
             'CPU state differs from spatial content')
    means, scales = (state.parameters[key] for key in ('means', 'log_scales'))
    origins = state.origin_ids
    count = len(origins)
    _require(1 <= count <= 10_000 and means.shape == scales.shape == (count, 3) and
             means.dtype == scales.dtype == np.dtype('<f4') and origins.shape == (count,) and
             origins.dtype == np.dtype('<u8') and np.isfinite(means).all() and
             np.isfinite(scales).all(), 'Invalid candidate parameters for metadata')
    counts = dict(zip(*np.unique(origins, return_counts=True)))
    _require(set(counts) == set(frames) and all(1 <= value <= 1000 for value in counts.values()),
             'Gaussian origins disagree with retained metadata views')
    extents = []
    for origin in frames:
        selected = origins == origin
        points = means[selected]
        try:
            maximum = math.exp(float(scales[selected].max()))
        except OverflowError as error:
            raise ValueError('Invalid candidate Gaussian scale') from error
        extents.append(dict(origin_keyframe_id=str(origin), min=points.min(axis=0).tolist(),
                            max=points.max(axis=0).tolist(), max_scale=maximum))
    view = state.views[-1]
    camera = dict(keyframe_id=str(view['keyframe_id']), calibration_id=str(view['calibration_id']),
                  width=view['rgb'].shape[1], height=view['rgb'].shape[0],
                  K=view['K'].ravel().tolist(), camera_from_body=view['camera_from_body'].ravel().tolist(),
                  capture_time=float(view['image_timestamp']))
    result = dict(content=dict(content), extents=extents, camera=camera)
    validate_metadata(result, content, frames)
    return result
