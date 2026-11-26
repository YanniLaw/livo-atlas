import copy
import os
import socket
import subprocess
import unittest

import numpy as np

from evaluation_protocol import decode_evaluation, validate_evaluation_message
from online_protocol import encode_message, read_message, write_message
from test_submap_protocol import SubmapProtocolTest
from render_fixtures import render_request_fixture
from test_online_protocol import view_fixture
from test_holdout_split import sample_manifest
from holdout_split import canonical_json


def evaluation_fixture():
    source, poses = render_request_fixture()
    source.update(type='gaussian_sources', query_keyframe_id='4')
    poses = np.eye(4, dtype='<f8').tobytes() + poses[128:]
    # The owner anchor is frame4 with absolute body tx(4).
    matrix = np.eye(4, dtype='<f8'); matrix[0, 3] = 4
    poses = matrix.tobytes() + poses[128:]
    source['blocks'][-1]['frame_ids'] = ['5']
    source['blocks'][-1]['gaussian_count'] = 1
    view, pixels = view_fixture()
    query = {k: view[k] for k in ('width', 'height', 'calibration_id', 'fx', 'fy', 'cx', 'cy',
        'state_timestamp', 'cloud_timestamp', 'image_timestamp', 'inverse_exposure_time')}
    query['keyframe_id'] = '4'
    query['camera_from_body'] = np.eye(4).ravel().tolist()
    header = dict(schema=2, type='heldout_evaluate', session_id=source['session_id'],
                  job_id=source['job_id'], split_id='a'*64, sources=source, query=query,
                  sealed_at_unix_seconds=1., dispatched_at_unix_seconds=2.)
    return header, poses + pixels


class EvaluationProtocolTest(unittest.TestCase):
    def test_capability_requires_holdout_and_explicit_budgets(self):
        manifest = sample_manifest()
        header = dict(schema=2, type='hello', session_id='a'*32, state_directory='/tmp/evaluation/states',
                      split_id=manifest['split_id'], capabilities=['joint_render_v1', 'holdout_split_v1', 'heldout_evaluation_v1'],
                      evaluation_limits=dict(max_output_bytes=str(256*1024*1024), min_free_disk_bytes='0'))
        encode_message(header, canonical_json(manifest))
        bad = copy.deepcopy(header); bad.pop('evaluation_limits')
        with self.assertRaises(ValueError):
            encode_message(bad, canonical_json(manifest))
        bad = copy.deepcopy(header); bad['capabilities'].remove('holdout_split_v1')
        with self.assertRaises(ValueError):
            encode_message(bad, canonical_json(manifest))
    @unittest.skipUnless(os.environ.get('LIVO_WIRE_PEER'), 'C++ wire peer is not configured')
    def test_cpp_echoes_independent_requests_and_both_receipts(self):
        header, body = evaluation_fixture()
        SubmapProtocolTest.echo(self, header, body)
        base = dict(schema=2, session_id=header['session_id'], job_id=header['job_id'],
                    split_id=header['split_id'], keyframe_id='4', map_sequence=header['sources']['map_sequence'])
        SubmapProtocolTest.echo(self, dict(base,type='heldout_evaluation_ok',report_sha256='b'*64,report_bytes='123'))
        SubmapProtocolTest.echo(self, dict(base,type='heldout_evaluation_error',code='oom',message='controlled'))

    @unittest.skipUnless(os.environ.get('LIVO_WIRE_PEER'), 'C++ wire peer is not configured')
    def test_cpp_generated_query_decodes_and_accepts_python_receipt(self):
        local, remote = socket.socketpair()
        with local, remote:
            process = subprocess.Popen([os.environ['LIVO_WIRE_PEER'],str(remote.fileno()),'--evaluation'],
                                       pass_fds=(remote.fileno(),),stderr=subprocess.PIPE,text=True)
            remote.close()
            try:
                header, body = read_message(local, 2)
                query = decode_evaluation(header, body)
                self.assertEqual(query['query']['keyframe_id'], 4)
                write_message(local, dict(schema=2,type='heldout_evaluation_ok',
                    session_id=header['session_id'],job_id=header['job_id'],split_id=header['split_id'],
                    keyframe_id='4',map_sequence=header['sources']['map_sequence'],
                    report_sha256='b'*64,report_bytes='123'), b'', 2)
            finally:
                _, error = process.communicate(timeout=5)
            self.assertEqual(process.returncode,0,error)


    def test_independent_camera_is_not_required_in_training_window(self):
        header, body = evaluation_fixture()
        validate_evaluation_message(header, body)
        value = decode_evaluation(header, body)
        self.assertEqual(value['query']['keyframe_id'], 4)
        self.assertEqual(value['sources']['blocks'][-1]['header']['frame_ids'], ['5'])
        encode_message(header, body)

    @unittest.skipUnless(os.environ.get('LIVO_WIRE_PEER'), 'C++ wire peer is not configured')
    def test_full_primary_binding_proof_uses_a_bounded_map_ack_body(self):
        header = dict(schema=2,type='map_committed',session_id='a'*32,job_id='3',generation='1',
                      pose_version='0',map_sequence='1',asset_ids=[],
                      evaluation_bindings=[dict(submap_id='0',primary_count=1)])
        body = np.eye(4,dtype='<f8').tobytes()
        SubmapProtocolTest.echo(self,header,body)
        with self.assertRaises(ValueError):
            encode_message(header,body[:-1])

    def test_strict_fields_and_boundaries(self):
        header, body = evaluation_fixture()
        for edit in (lambda h: h.update(extra=1), lambda h: h.update(split_id='bad'),
                     lambda h: h['query'].update(keyframe_id='5'),
                     lambda h: h['sources'].update(job_id='1')):
            bad = copy.deepcopy(header); edit(bad)
            with self.subTest(edit=edit), self.assertRaises(ValueError):
                validate_evaluation_message(bad, body)
        with self.assertRaises(ValueError):
            validate_evaluation_message(header, body[:-1])


if __name__ == '__main__':
    unittest.main()
