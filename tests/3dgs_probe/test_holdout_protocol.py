import unittest
import os
import socket
import threading
from unittest import mock

from online_protocol import encode_message
from holdout_split import CAPABILITY, canonical_json
from submap_fixtures import SESSION
from test_holdout_split import sample_manifest


class HoldoutProtocolTest(unittest.TestCase):
    def test_worker_rejects_tampered_manifest_before_initializing_gpu(self):
        from online_protocol import read_message, write_message
        from online_worker import serve_connection

        manifest = sample_manifest()
        header = dict(schema=2, type='hello', session_id=SESSION,
                      state_directory='/tmp/holdout/states', capabilities=[CAPABILITY],
                      split_id=manifest['split_id'])
        manifest['frames'][1]['role'] = 'train'
        factory = mock.Mock()
        local, remote = socket.socketpair()
        with local, remote:
            worker = threading.Thread(target=serve_connection, args=(remote, factory, 1))
            worker.start()
            try:
                write_message(local, header, canonical_json(manifest), 1)
                reply, _ = read_message(local, 2)
                self.assertEqual(reply['type'], 'error')
            finally:
                worker.join(timeout=3)
            self.assertFalse(worker.is_alive())
        factory.assert_not_called()

    @unittest.skipUnless(os.environ.get('LIVO_WIRE_PEER'), 'C++ wire peer is not configured')
    def test_cpp_accepts_the_negotiated_split_and_matching_schema(self):
        from test_submap_protocol import SubmapProtocolTest

        manifest = sample_manifest()
        for capabilities in ([CAPABILITY], ['joint_render_v1', CAPABILITY],
                             ['joint_render_v1', 'joint_render_spatial_v1', CAPABILITY]):
            header = dict(schema=2, type='hello', session_id=SESSION,
                          state_directory='/tmp/holdout/states', capabilities=capabilities,
                          split_id=manifest['split_id'])
            SubmapProtocolTest().echo(header, canonical_json(manifest))
    def test_explicit_capability_carries_bounded_manifest(self):
        manifest = sample_manifest()
        header = dict(schema=2, type='hello', session_id=SESSION,
                      state_directory='/tmp/holdout/states', capabilities=[CAPABILITY],
                      split_id=manifest['split_id'])
        encode_message(header, canonical_json(manifest))
        for caps in ([], ['joint_render_v1'], [CAPABILITY, 'joint_render_v1']):
            with self.subTest(caps=caps), self.assertRaises(ValueError):
                encode_message(dict(header, capabilities=caps), canonical_json(manifest))
        for body in (b'', b'x' * (1024 * 1024 + 1)):
            with self.subTest(size=len(body)), self.assertRaises(ValueError):
                encode_message(header, body)

    def test_default_hello_keeps_original_strict_shape(self):
        header = dict(schema=2, type='hello', session_id=SESSION,
                      state_directory='/tmp/holdout/states')
        encode_message(header, b'')
        with self.assertRaises(ValueError):
            encode_message(header, canonical_json(sample_manifest()))
        with self.assertRaises(ValueError):
            encode_message(dict(header, split_id='a' * 64), b'')


if __name__ == '__main__':
    unittest.main()
