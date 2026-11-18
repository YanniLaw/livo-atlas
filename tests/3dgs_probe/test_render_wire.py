"""The installed C++ codec and Python validator exchange independent records."""
import os
import socket
import subprocess
import unittest

import numpy as np

from joint_render_protocol import decode_render_request
from online_protocol import read_message, write_message
from render_fixtures import render_request_fixture, render_reply_fixture
import test_submap_protocol


class RenderWireTest(unittest.TestCase):
    def test_python_records_are_validated_and_echoed_by_cpp(self):
        for fixture in (render_request_fixture, render_reply_fixture):
            header, body = fixture()
            test_submap_protocol.SubmapProtocolTest.echo(self, header, body)

    def test_cpp_request_is_decoded_then_cpp_decodes_python_reply(self):
        local, remote = socket.socketpair()
        with local, remote:
            process = subprocess.Popen([os.environ['LIVO_WIRE_PEER'], str(remote.fileno()), '--render'],
                                       pass_fds=(remote.fileno(),), stderr=subprocess.PIPE, text=True)
            remote.close()
            try:
                header, body = read_message(local, 2)
                decoded = decode_render_request(header, body)
                self.assertEqual(header['job_id'], '9007199254740993')
                self.assertEqual(decoded['parent_from_body'][0, 3], 5)
                self.assertEqual(decoded['blocks'][0]['header']['content']['content_job_id'], '11')
                reply, rgb = render_reply_fixture()
                # Both independent fixtures intentionally specify the same source.
                self.assertEqual(reply['blocks'], header['blocks'])
                write_message(local, reply, rgb, 2)
            finally:
                _, error = process.communicate(timeout=5)
            self.assertEqual(process.returncode, 0, error)

    def test_cpp_spatial_request_and_python_reply_preserve_nonadjacent_metadata(self):
        local, remote = socket.socketpair()
        with local, remote:
            process = subprocess.Popen(
                [os.environ['LIVO_WIRE_PEER'], str(remote.fileno()), '--spatial-render'],
                pass_fds=(remote.fileno(),), stderr=subprocess.PIPE, text=True)
            remote.close()
            try:
                header, body = read_message(local, 2)
                decoded = decode_render_request(header, body)
                self.assertEqual(header['policy'], 'spatial')
                self.assertEqual([b['content']['submap_id'] for b in header['blocks']], ['0', '2'])
                reply, rgb = render_reply_fixture()
                reply.update(header, type='joint_render_ok')
                camera = header['blocks'][-1]['render_metadata']['camera']
                reply['camera_from_body'] = camera['camera_from_body']
                reply['image_timestamp'] = camera['capture_time']
                camera_from_parent = np.asarray(camera['camera_from_body']).reshape(4, 4) @ np.linalg.inv(
                    decoded['parent_from_body'])
                reply['camera_from_parent'] = camera_from_parent.ravel().tolist()
                write_message(local, reply, rgb, 2)
            finally:
                _, error = process.communicate(timeout=5)
            self.assertEqual(process.returncode, 0, error)


if __name__ == '__main__':
    unittest.main()
