"""Opt-in, bounded read-only rendering messages for persistent schema 2."""
import copy
import math
import re

import numpy as np

from online_protocol import (_SESSION, _dimensions, _integer, _number, _pose,
                             _unsigned)

_ROOT = {'schema', 'type', 'session_id', 'job_id', 'generation', 'pose_version',
         'map_sequence', 'frame_count', 'optimized_frame_count', 'parent_frame',
         'query_keyframe_id', 'query_submap_id', 'neighbor_reason', 'blocks'}
_TARGET = {'generation', 'pose_version', 'submap_id', 'membership_revision',
           'frame_count', 'optimized_frame_count', 'anchor_keyframe_id',
           'primary_count', 'submap_state', 'parent_frame', 'parent_from_submap'}
_CONTENT = {'submap_id', 'source_generation', 'content_job_id'}
_ASSET = _CONTENT | {'asset_id', 'manifest_sha256', 'manifest_bytes',
                     'gaussian_bytes', 'observation_bytes'}
_CAMERA = {'training_view', 'width', 'height', 'calibration_id', 'fx', 'fy', 'cx', 'cy',
           'parent_from_body', 'camera_from_body', 'camera_from_parent',
           'image_timestamp', 'state_timestamp', 'cloud_timestamp', 'inverse_exposure_time'}
_COUNTS = {'image_pixels', 'covered_pixels', 'depth_pixels_available', 'depth_pixels_used'}
_METRICS = {'rgb_l1', 'rgb_mse', 'rgb_psnr_db', 'alpha_coverage', 'depth_l1_m', 'depth_rmse_m'}
_FLAGS = {'image_valid', 'depth_valid', 'psnr_valid', 'rgb_perfect'}
_MEMORY = {'gpu_peak_allocated_bytes', 'gpu_peak_reserved_bytes', 'gpu_allocated_bytes',
           'gpu_reserved_bytes', 'worker_peak_rss_bytes'}


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _keys(value, required, optional=frozenset()):
    _require(isinstance(value, dict) and required <= value.keys() <= required | optional,
             'Missing or unknown joint render fields')


def _positive(value, name):
    number = _unsigned(value, name)
    _require(number > 0, 'Expected positive ' + name)
    return number


def _root_fields(header):
    return _ROOT | ({'policy', 'selection'} if 'policy' in header or 'selection' in header else set())


def _selection(header, camera):
    spatial = 'policy' in header
    if not spatial:
        return
    _require(header['policy'] == 'spatial', 'Unknown spatial wire policy')
    value = header['selection']
    _keys(value, {'candidate_count', 'candidate_pixels', 'projection_uncertain', 'selection_seconds'})
    count = _integer(value['candidate_count'], 'candidate count', 0, 31)
    pixels = _integer(value['candidate_pixels'], 'candidate pixels', 0, camera['width'] * camera['height'])
    uncertain = _integer(value['projection_uncertain'], 'uncertain', 0, 1)
    dual = len(header['blocks']) == 2
    _require((count > 0) == dual and (pixels > 0) == dual and
             (not uncertain or pixels == camera['width'] * camera['height']) and
             _number(value['selection_seconds'], 'selection time') >= 0, 'Invalid spatial selection summary')


def _header(header):
    """Bound every count before decoding even the first primary body array."""
    from submap_protocol import _frame_ids

    spatial = 'policy' in header
    _require(not spatial or header['policy'] == 'spatial', 'Unknown render policy')

    _require(type(header.get('schema')) is int and header['schema'] == 2 and
             isinstance(header.get('session_id'), str) and
             _SESSION.fullmatch(header['session_id']), 'Invalid render session')
    for key in ('job_id', 'generation', 'map_sequence'):
        _positive(header.get(key), key)
    version = _unsigned(header.get('pose_version'), 'pose version')
    frontier = _positive(header.get('frame_count'), 'frame count')
    optimized = _unsigned(header.get('optimized_frame_count'), 'optimized frame count')
    query = _unsigned(header.get('query_keyframe_id'), 'query frame')
    owner = _unsigned(header.get('query_submap_id'), 'query submap')
    _require(query < frontier <= 512 and optimized <= frontier and
             bool(version) == bool(optimized) and
             header.get('parent_frame') == ('map' if version else 'odom'), 'Invalid render frontier')
    blocks = header.get('blocks')
    _require(isinstance(blocks, list) and 1 <= len(blocks) <= 2, 'Select one or two render blocks')
    ids, residents, primary_count = [], 0, 0
    for block in blocks:
        fields = {'target', 'content', 'resident', 'frame_ids', 'gaussian_count'}
        if spatial:
            fields.add('render_metadata')
        _keys(block, fields, {'asset'})
        target, content = block['target'], block['content']
        _keys(target, _TARGET)
        _keys(content, _CONTENT)
        submap = _unsigned(target['submap_id'], 'submap')
        ids.append(submap)
        _require(_positive(target['generation'], 'target generation') <= int(header['generation']),
                 'Render binding generation exceeds the session fence')
        count = _integer(target['primary_count'], 'primary count', 1, 512)
        anchor = _unsigned(target['anchor_keyframe_id'], 'anchor')
        sealed = target['submap_state'] == 'sealed'
        _require(target['submap_state'] in ('active', 'sealed') and anchor < frontier and
                 count <= frontier - anchor and
                 _unsigned(target['membership_revision'], 'revision') == count + int(sealed),
                 'Invalid primary membership')
        _require(all(target[key] == header[key] for key in
                     ('pose_version', 'frame_count', 'optimized_frame_count', 'parent_frame')),
                 'Block target differs from request frontier')
        _pose(target['parent_from_submap'], 'parent from submap')
        _require(_unsigned(content['submap_id'], 'content submap') == submap,
                 'Mismatched source submap')
        _positive(content['source_generation'], 'source generation')
        _positive(content['content_job_id'], 'content job')
        frame_ids = _frame_ids(block)
        _require(all(anchor <= key < anchor + count for key in frame_ids), 'Uncovered retained frame')
        if spatial:
            from spatial_metadata import validate_metadata
            validate_metadata(block['render_metadata'], dict(session_id=header['session_id'], **content), frame_ids)
        resident = _integer(block['resident'], 'resident', 0, 1)
        residents += resident
        _require(('asset' in block) == (not resident), 'Invalid resident source fields')
        if not resident:
            asset = block['asset']
            _keys(asset, _ASSET)
            _require(all(asset[key] == content[key] for key in _CONTENT), 'Asset content mismatch')
            _positive(asset['asset_id'], 'asset ID')
            _require(isinstance(asset['manifest_sha256'], str) and
                     re.fullmatch('[0-9a-f]{64}', asset['manifest_sha256']), 'Invalid manifest hash')
            for key, limit in (('manifest_bytes', 512 * 1024), ('gaussian_bytes', 1024 * 1024),
                               ('observation_bytes', 192 * 1024 * 1024)):
                _require(_positive(asset[key], key) <= limit, 'Asset exceeds budget')
        primary_count += count
    _require(residents <= 1 and ids == sorted(set(ids)) and ids[-1] == owner,
             'Invalid selected render blocks')
    reason = header.get('neighbor_reason')
    if len(ids) == 2:
        _require(owner > 0 and ids[0] < owner and (spatial or ids[0] == owner - 1) and reason == '',
                 'Invalid selected neighbor')
    else:
        expected = 'no_spatial_candidate' if spatial else ('no_previous_group' if owner == 0 else 'neighbor_unavailable')
        _require(reason == expected,
                 'Missing single-block reason')
    _require(str(query) == blocks[-1]['frame_ids'][-1], 'Query is not the last retained owner camera')
    if spatial:
        _selection(header, blocks[-1]['render_metadata']['camera'])
    return primary_count


def validate_render_header(header):
    """Validate request identity without pretending its body arrays are available."""
    _keys(header, _root_fields(header))
    _require(header['type'] == 'joint_render', 'Expected render request')
    return _header(header)


def decode_render_request(header, body):
    count = validate_render_header(header)
    _require(isinstance(body, bytes) and len(body) == 128 * (1 + count) and len(body) <= 131200,
             'Incorrect joint render body size')
    from submap_protocol import _primary_context, _same_pose

    parent = _pose(np.frombuffer(body, dtype='<f8', count=16).tolist(), 'query body pose')
    decoded, offset = [], 128
    for block in header['blocks']:
        size = 128 * block['target']['primary_count']
        flat = dict(block['target'], session_id=header['session_id'])
        target = _primary_context(flat, body[offset:offset + size], 0)
        decoded.append(dict(header=copy.deepcopy(block), target=target))
        offset += size
    owner = decoded[-1]['target']
    relative = next(entry for entry in owner['primary'] if entry['keyframe_id'] == header['query_keyframe_id'])
    composed = (np.asarray(owner['parent_from_submap']).reshape(4, 4) @
                np.asarray(relative['submap_from_body']).reshape(4, 4))
    _require(_same_pose(parent, composed), 'Query body differs from the complete primary binding')
    return dict(header=copy.deepcopy(header), parent_from_body=parent.copy(), blocks=decoded)


def _metrics(metrics, pixels):
    _keys(metrics, _COUNTS | _METRICS | _FLAGS)
    for key in _COUNTS:
        _integer(metrics[key], key, 0, pixels)
    for key in _FLAGS:
        _integer(metrics[key], key, 0, 1)
    for key in _METRICS:
        _number(metrics[key], key)
    image, covered = metrics['image_pixels'], metrics['covered_pixels']
    available, used = metrics['depth_pixels_available'], metrics['depth_pixels_used']
    mse = metrics['rgb_mse']
    _require(covered <= image and used <= min(available, covered) and available <= image,
             'Invalid metric pixel subsets')
    _require(metrics['image_valid'] == int(image > 0) and metrics['depth_valid'] == int(used > 0) and
             metrics['psnr_valid'] == int(image > 0 and mse > 0) and
             metrics['rgb_perfect'] == int(image > 0 and mse == 0), 'Inconsistent metric validity')
    _require(all(0 <= metrics[key] <= 1 for key in ('rgb_l1', 'rgb_mse', 'alpha_coverage')) and
             metrics['depth_l1_m'] >= 0 and metrics['depth_rmse_m'] >= 0, 'Invalid metric range')
    _require(abs(metrics['alpha_coverage'] - (covered / image if image else 0)) < 1e-9,
             'Incorrect alpha coverage')
    for valid, keys in ((image > 0, ('rgb_l1', 'rgb_mse', 'alpha_coverage')),
                        (used > 0, ('depth_l1_m', 'depth_rmse_m')),
                        (mse > 0 and image > 0, ('rgb_psnr_db',))):
        _require(valid or all(metrics[key] == 0 for key in keys), 'Invalid metric must use finite zero')
    if image > 0 and mse > 0:
        _require(abs(metrics['rgb_psnr_db'] + 10 * math.log10(mse)) < 1e-6, 'Incorrect PSNR')


def validate_render_message(header, body):
    kind = header.get('type')
    if kind == 'joint_render':
        decode_render_request(header, body)
        return
    extra = ({'code', 'error'} if kind == 'joint_render_error' else
             _CAMERA | _MEMORY | {'single', 'joint', 'changed_pixels', 'single_gaussian_count',
                                  'gaussian_count', 'worker_seconds'})
    _keys(header, _root_fields(header) | extra)
    _header(header)
    _require(isinstance(body, bytes), 'Invalid render body')
    if kind == 'joint_render_error':
        error = header['error']
        _require(header['code'] == 'oom' and not body and isinstance(error, str) and
                 0 < len(error.encode('utf-8')) <= 4096, 'Invalid render error')
        return
    _require(kind == 'joint_render_ok' and type(header['training_view']) is int and
             header['training_view'] == 1, 'Expected retained training camera')
    width, height, pixels = _dimensions(header)
    _require(len(body) == 3 * pixels, 'Incorrect RGB body size')
    _require(_unsigned(header['calibration_id'], 'calibration') == 0, 'Unknown calibration')
    for key in ('fx', 'fy', 'cx', 'cy', 'image_timestamp', 'state_timestamp', 'cloud_timestamp',
                'inverse_exposure_time', 'worker_seconds'):
        value = _number(header[key], key)
        _require(key not in ('fx', 'fy', 'worker_seconds') or value >= (0 if key == 'worker_seconds' else 1e-300),
                 'Invalid camera or timing')
    from submap_protocol import _same_pose
    parent = _pose(header['parent_from_body'], 'body pose')
    camera = _pose(header['camera_from_body'], 'camera body pose')
    _require(_same_pose(_pose(header['camera_from_parent'], 'global camera'), camera @ np.linalg.inv(parent)),
             'Incorrect query camera composition')
    if 'policy' in header:
        saved = header['blocks'][-1]['render_metadata']['camera']
        K = saved['K']
        _require(header['width'] == saved['width'] and header['height'] == saved['height'] and
                 header['calibration_id'] == saved['calibration_id'] and
                 [header[key] for key in ('fx', 'fy', 'cx', 'cy')] == [K[0], K[4], K[2], K[5]] and
                 header['image_timestamp'] == saved['capture_time'] and
                 np.array_equal(camera, _pose(saved['camera_from_body'], 'spatial query camera')),
                 'Render replaced the retained spatial query camera')
    for key in ('single', 'joint'):
        _metrics(header[key], pixels)
    _integer(header['changed_pixels'], 'changed pixels', 0, pixels)
    count = sum(block['gaussian_count'] for block in header['blocks'])
    _require(type(header['gaussian_count']) is int and header['gaussian_count'] == count and
             type(header['single_gaussian_count']) is int and
             header['single_gaussian_count'] == header['blocks'][-1]['gaussian_count'], 'Incorrect rendered count')
    for key in _MEMORY:
        _unsigned(header[key], key)


def encode_render_reply(request, result):
    header = copy.deepcopy(request['header'])
    header['type'] = 'joint_render_ok'
    header.update(result['metadata'])
    header.update(parent_from_body=request['parent_from_body'].ravel().tolist())
    for name in ('single', 'joint'):
        metrics = result[name]
        header[name] = {key: int(metrics[key]) for key in _COUNTS}
        header[name].update({key: metrics[key] if metrics[key] is not None else 0 for key in _METRICS})
        header[name].update(image_valid=int(metrics['image_pixels'] > 0),
            depth_valid=int(metrics['depth_pixels_used'] > 0),
            psnr_valid=int(metrics['rgb_psnr_db'] is not None), rgb_perfect=int(metrics['rgb_perfect']))
    body = result['rgb'].tobytes()
    validate_render_message(header, body)
    return header, body
