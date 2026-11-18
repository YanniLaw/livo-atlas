import copy
import importlib.util
from pathlib import Path
import socket
import tempfile
import threading
import struct
import unittest
from unittest import mock

import numpy as np

from joint_render_protocol import decode_render_request, validate_render_message
from online_protocol import encode_message, read_message, write_message
from render_fixtures import render_request_fixture, render_reply_fixture, spatial_request_fixture
from state_fixtures import pose
from submap_fixtures import SESSION


class JointRenderProtocolTest(unittest.TestCase):
    def test_spatial_nonadjacent_request_has_exact_bounded_metadata(self):
        header, body = spatial_request_fixture()
        result = decode_render_request(header, body)
        self.assertEqual([block['header']['content']['submap_id'] for block in result['blocks']], ['0', '2'])
        self.assertEqual(result['header']['selection']['candidate_pixels'], 48)
        self.assertEqual(len(body), 640)
        edits = [lambda h: h.update(policy='temporal'),
                 lambda h: h['selection'].update(candidate_count=32),
                 lambda h: h['selection'].update(candidate_pixels=49),
                 lambda h: h['blocks'][0].pop('render_metadata'),
                 lambda h: h['blocks'][0]['render_metadata']['content'].update(content_job_id='11')]
        for edit in edits:
            changed = copy.deepcopy(header)
            edit(changed)
            with self.subTest(edit=edit), self.assertRaises(ValueError):
                decode_render_request(changed, body)

    def test_query_must_be_the_last_retained_owner_camera(self):
        header, body = render_request_fixture()
        header['query_keyframe_id'] = '4'
        body = pose(4).astype('<f8').tobytes() + body[128:]
        with self.assertRaisesRegex(ValueError, 'latest|last'):
            decode_render_request(header, body)

    def test_complete_owned_request_keeps_big_job_and_exact_body_poses(self):
        header, body = render_request_fixture()
        self.assertEqual(len(body), 640)
        result = decode_render_request(header, body)
        self.assertEqual(result['header']['job_id'], '9007199254740993')
        np.testing.assert_array_equal(result['parent_from_body'], pose(5))
        self.assertEqual(result['blocks'][1]['target']['primary'][-1]['keyframe_id'], '5')
        self.assertEqual(result['blocks'][1]['target']['primary'][-1]['submap_from_body'][3], 1)
        header['blocks'][1]['target']['parent_from_submap'][3] = 999
        self.assertEqual(result['header']['blocks'][1]['target']['parent_from_submap'][3], 4)

    def test_resident_binding_generation_is_independent_of_session_fence(self):
        header, body = render_request_fixture()
        block = header['blocks'][-1]
        block.update(resident=1)
        block.pop('asset')
        header.update(blocks=[block], neighbor_reason='neighbor_unavailable')
        # A discarded repair advanced the session but restored generation3.
        decoded = decode_render_request(header, body[:128] + body[-256:])
        self.assertEqual(decoded['header']['generation'], '4')
        self.assertEqual(decoded['blocks'][0]['target']['generation'], '3')

    def test_bad_budgets_fail_before_numpy_body_allocation(self):
        edits = [lambda h: h.update(frame_count='513'),
                 lambda h: h.update(query_keyframe_id='6'),
                 lambda h: h.update(job_id='18446744073709551616'),
                 lambda h: h.update(generation=True),
                 lambda h: h['blocks'].append(copy.deepcopy(h['blocks'][0])),
                 lambda h: h['blocks'][0]['target'].update(primary_count=513),
                 lambda h: h['blocks'][0]['target'].update(anchor_keyframe_id='18446744073709551615')]
        for edit in edits:
            header, body = render_request_fixture()
            edit(header)
            with self.subTest(header=header), mock.patch('joint_render_protocol.np.frombuffer',
                                                        side_effect=AssertionError('unbounded allocation')):
                with self.assertRaises(ValueError):
                    decode_render_request(header, body)

    def test_shapes_camera_relation_and_source_identity_are_checked(self):
        header, body = render_request_fixture()
        for payload in (body[:-1], body + b'x'):
            with self.assertRaises(ValueError):
                decode_render_request(header, payload)
        raw = np.frombuffer(body, dtype='<f8').copy()
        for offset, value in ((3, 0), (16, np.nan), (16, 2)):
            bad = raw.copy()
            bad[offset] = value
            with self.subTest(offset=offset, value=value), self.assertRaises(ValueError):
                decode_render_request(header, bad.tobytes())
        for edit in (lambda h: h['blocks'][0].update(resident=1),
                     lambda h: h['blocks'][0]['asset'].update(manifest_sha256='bad'),
                     lambda h: h['blocks'][0]['content'].update(source_generation='0'),
                     lambda h: h['blocks'][1]['target'].update(submap_id='1'),
                     lambda h: h['blocks'][1].update(frame_ids=['4', '4'])):
            changed = copy.deepcopy(header)
            edit(changed)
            with self.assertRaises(ValueError):
                decode_render_request(changed, body)

    def test_valid_reply_requires_exact_rgb_and_finite_consistent_metrics(self):
        header, body = render_reply_fixture()
        validate_render_message(header, body)
        for edit in (lambda h: h.update(width=1000001),
                     lambda h: h['joint'].update(depth_l1_m=None),
                     lambda h: h['joint'].update(depth_valid=0),
                     lambda h: h['joint'].update(alpha_coverage=.9),
                     lambda h: h['joint'].update(rgb_psnr_db=float('inf')),
                     lambda h: h.update(changed_pixels=49),
                     lambda h: h.update(gaussian_count=3),
                     lambda h: h.update(training_view=0)):
            changed = copy.deepcopy(header)
            edit(changed)
            with self.subTest(edit=edit), self.assertRaises(ValueError):
                validate_render_message(changed, body)
        with self.assertRaises(ValueError):
            validate_render_message(header, body[:-1])

    def test_empty_and_perfect_metrics_use_wire_validity_instead_of_null(self):
        header, body = render_reply_fixture()
        header['changed_pixels'] = 0
        for name in ('single', 'joint'):
            header[name] = {key: 0 for key in header[name]}
        validate_render_message(header, body)
        for name in ('single', 'joint'):
            header[name].update(image_valid=1, image_pixels=48, rgb_perfect=1)
        validate_render_message(header, body)
        header['joint']['rgb_perfect'] = 0
        with self.assertRaises(ValueError):
            validate_render_message(header, body)

    def test_capabilities_are_opt_in_without_changing_legacy_hello_bytes(self):
        header = dict(schema=2, type='hello', session_id=SESSION, state_directory='/tmp/render-test/states')
        old = ('{"schema":2,"type":"hello","session_id":"' + SESSION +
               '","state_directory":"/tmp/render-test/states"}').encode()
        self.assertEqual(encode_message(header, b''), struct.pack('<IQ', len(old), 0) + old)
        header['capabilities'] = ['joint_render_v1']
        self.assertIn(b'joint_render_v1', encode_message(header, b''))
        for value in ([], ['unknown'], ['joint_render_v1'] * 2, 'joint_render_v1'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                encode_message(dict(header, capabilities=value), b'')

    def test_sensorless_ros_entry_peer_negotiates_only_requested_capability(self):
        path = Path(__file__).resolve().parents[2] / 'tests/gaussian/fake_worker.py'
        spec = importlib.util.spec_from_file_location('render_entry_peer', path)
        peer = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(peer)
        for enabled in (False, True):
            with tempfile.TemporaryDirectory() as root:
                local, remote = socket.socketpair()
                with local, remote:
                    hello = dict(schema=2, type='hello', session_id=SESSION,
                                 state_directory=str(Path(root) / 'states'))
                    if enabled:
                        hello['capabilities'] = ['joint_render_v1']
                    thread = threading.Thread(target=peer.serve_persistent, args=(remote, hello))
                    thread.start()
                    try:
                        response, _ = read_message(local, 2)
                        self.assertEqual(response.get('capabilities'),
                                         ['joint_render_v1'] if enabled else None)
                    finally:
                        stop = dict(schema=2, type='stop', session_id=SESSION,
                                    job_id='1', generation='0', pose_version='0', map_sequence='0')
                        write_message(local, stop, b'', 2)
                        read_message(local, 2)
                        thread.join(2)
                    self.assertFalse(thread.is_alive())


if __name__ == '__main__':
    unittest.main()
