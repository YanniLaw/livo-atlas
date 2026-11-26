"""Opt-in heldout messages; source capture is distinct from preview queries."""

import re

import numpy as np

from joint_render_protocol import (_header, _keys, _positive, _root_fields,
                                    _selection, decode_source_request)
from online_protocol import _dimensions, _number, _pose, _unsigned, _validate_view


CAPABILITY = 'heldout_evaluation_v1'
_BASE = {'schema', 'type', 'session_id', 'job_id', 'split_id', 'keyframe_id', 'map_sequence'}
_QUERY = {'width', 'height', 'keyframe_id', 'calibration_id', 'fx', 'fy', 'cx', 'cy',
          'state_timestamp', 'cloud_timestamp', 'image_timestamp', 'inverse_exposure_time',
          'camera_from_body'}


def validate_evaluation_message(header, body):
    kind = header.get('type')
    if kind == 'heldout_evaluate':
        _keys(header, {'schema', 'type', 'session_id', 'job_id', 'split_id', 'sources', 'query',
                       'sealed_at_unix_seconds', 'dispatched_at_unix_seconds'})
        source, query = header['sources'], header['query']
        _keys(source, _root_fields(source))
        _keys(query, _QUERY)
        if source['type'] != 'gaussian_sources' or any(header[k] != source[k] for k in
                                                      ('schema', 'session_id', 'job_id')):
            raise ValueError('Evaluation source envelope differs')
        if query['keyframe_id'] != source['query_keyframe_id']:
            raise ValueError('Evaluation camera differs from source query')
        count = _header(source, independent=True)
        offset = 128 * (1 + count)
        if not isinstance(body, bytes) or len(body) != offset + 9 * _dimensions(query)[2]:
            raise ValueError('Incorrect evaluation body size')
        decode_source_request(source, body[:offset])
        if source.get('policy') == 'spatial':
            retained = source['blocks'][-1]['render_metadata']['camera']
            if any(query[key] != retained[key] for key in ('width', 'height')):
                raise ValueError('Evaluation camera dimensions differ from spatial owner')
        _selection(source, query)
        camera = _pose(query['camera_from_body'], 'heldout body camera')
        flat = dict(query, **{k: source[k] for k in ('frame_count', 'optimized_frame_count',
                                                    'pose_version', 'parent_frame')},
                    camera_from_submap=camera.ravel().tolist(), parent_from_submap=np.eye(4).ravel().tolist())
        _validate_view(flat, body[offset:])
        if (_number(header['sealed_at_unix_seconds'], 'seal time') <= 0 or
                _number(header['dispatched_at_unix_seconds'], 'dispatch time') < header['sealed_at_unix_seconds']):
            raise ValueError('Invalid evaluation event times')
    else:
        _keys(header, _BASE | ({'report_sha256', 'report_bytes'} if kind == 'heldout_evaluation_ok'
                              else {'code', 'message'}))
        if kind not in ('heldout_evaluation_ok', 'heldout_evaluation_error') or body:
            raise ValueError('Invalid evaluation reply')
        _unsigned(header['keyframe_id'], 'query ID')
        _positive(header['map_sequence'], 'map sequence')
        if kind == 'heldout_evaluation_ok':
            if (not re.fullmatch('[0-9a-f]{64}', header['report_sha256']) or
                    not 0 < _positive(header['report_bytes'], 'report bytes') <= 2*1024*1024):
                raise ValueError('Invalid evaluation report identity')
        elif (header['code'] not in ('oom', 'source', 'storage', 'budget') or
              not isinstance(header['message'], str) or len(header['message'].encode()) > 2048):
            raise ValueError('Invalid evaluation error')
    if (type(header['schema']) is not int or header['schema'] != 2 or
            not isinstance(header['session_id'], str) or not re.fullmatch('[0-9a-f]{32}', header['session_id']) or
            not isinstance(header['split_id'], str) or not re.fullmatch('[0-9a-f]{64}', header['split_id'])):
        raise ValueError('Invalid evaluation session or split')
    _positive(header['job_id'], 'job ID')


def decode_evaluation(header, body):
    validate_evaluation_message(header, body)
    source = header['sources']
    count = sum(block['target']['primary_count'] for block in source['blocks'])
    offset = 128 * (1 + count)
    query = dict(header['query'])
    width, height, pixels = _dimensions(query)
    raw = body[offset:]
    query.update(keyframe_id=int(query['keyframe_id']), calibration_id=int(query['calibration_id']), split='heldout',
                 rgb=np.frombuffer(raw, 'u1', 3*pixels).reshape(height, width, 3)[..., ::-1].astype('f4')/255,
                 image_mask=np.frombuffer(raw, 'u1', pixels, 3*pixels).reshape(height, width).astype(bool),
                 depth_mask=np.frombuffer(raw, 'u1', pixels, 4*pixels).reshape(height, width).astype(bool),
                 depth_m=np.frombuffer(raw, '<f4', pixels, 5*pixels).reshape(height, width).copy(),
                 K=np.array([[query['fx'], 0, query['cx']], [0, query['fy'], query['cy']], [0, 0, 1]], 'f8'),
                 camera_from_body=_pose(query['camera_from_body'], 'heldout body camera'))
    return dict(header=header, sources=decode_source_request(source, body[:offset]), query=query)
