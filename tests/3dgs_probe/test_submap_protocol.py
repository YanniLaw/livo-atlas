import json
import os
import socket
import struct
import subprocess
import sys
import unittest
from unittest import mock

import numpy as np

import online_protocol
from online_protocol import decode_view, encode_message, read_message, write_message
from submap_fixtures import (SESSION, candidate_fixture, control_fixture, target_fixture,
                             view_v2_fixture)


class SubmapProtocolTest(unittest.TestCase):
    def echo(self, header, body=b""):
        local, remote = socket.socketpair()
        with local, remote:
            process = subprocess.Popen([os.environ["LIVO_WIRE_PEER"], str(remote.fileno())],
                                       pass_fds=(remote.fileno(),), stdout=subprocess.PIPE,
                                       stderr=subprocess.PIPE, text=True)
            remote.close()
            try:
                write_message(local, header, body, 2)
                received = read_message(local, 2)
            finally:
                if process.poll() is None:
                    process.wait(timeout=3)
                out, error = process.communicate(timeout=2)
            self.assertEqual(process.returncode, 0, out + error)
        self.assertEqual(received, (header, body))
        return received

    def test_cpp_generated_full_primary_body_roundtrips_losslessly(self):
        local, remote = socket.socketpair()
        with local, remote:
            process = subprocess.Popen([os.environ["LIVO_WIRE_PEER"], str(remote.fileno()), "--target"],
                                       pass_fds=(remote.fileno(),), stdout=subprocess.PIPE,
                                       stderr=subprocess.PIPE, text=True)
            remote.close()
            try:
                header, body = read_message(local, 2)
                self.assertEqual(header["job_id"], "9007199254740993")
                self.assertEqual(len(body), 512 * 128)
                context = online_protocol.decode_target(header, body)
                self.assertEqual(context["primary"][-1]["keyframe_id"], "511")
                self.assertEqual(context["primary"][-1]["submap_from_body"][3], 511)
                self.assertEqual(context["parent_from_submap"],
                                 [0, -1, 0, 4, 1, 0, 0, -2, 0, 0, 1, 1, 0, 0, 0, 1])
                self.assertLess(len(json.dumps(header)), 64 * 1024)
                write_message(local, header, body, 2)
            finally:
                if process.poll() is None:
                    process.wait(timeout=3)
                out, error = process.communicate(timeout=2)
            self.assertEqual(process.returncode, 0, out + error)

    def test_view_preserves_pixels_depth_intrinsics_and_independent_body_camera(self):
        header, body = view_v2_fixture(frame=1)
        received, payload = self.echo(header, body)
        view = decode_view(received, payload)
        context = online_protocol.decode_target(received, payload, 9 * 48)
        self.assertEqual(context["capture_kind"], "online_confirmed")
        self.assertEqual(view["keyframe_id"], 1)
        np.testing.assert_allclose(view["rgb"][3, 4], [30/255, 20/255, 10/255], rtol=0, atol=1e-8)
        self.assertEqual(view["depth_m"][3, 4], 4)
        np.testing.assert_array_equal(view["K"], [[4, 0, 3.5], [0, 4, 2.5], [0, 0, 1]])
        np.testing.assert_array_equal(view["camera_from_body"],
                                     [[0, -1, 0, 1], [1, 0, 0, 2], [0, 0, 1, 3], [0, 0, 0, 1]])
        self.assertEqual(context["primary"][1]["submap_from_body"][3], 1)

    def test_reference_acknowledgement_preserves_large_asset_ids(self):
        header = control_fixture("map_committed", job=2**53+1, map_sequence="2",
                                 asset_ids=["1", "9007199254740993", "18446744073709551615"])
        self.echo(header)
        for kind in ("map_committed", "map_committed_ok"):
            self.echo(control_fixture(kind, map_sequence="3", asset_ids=[]))

    def test_bounds_and_shapes_fail_before_primary_numpy_allocation(self):
        for change in ({"primary_count": 513}, {"primary_count": 2**64},
                       {"anchor_keyframe_id": "18446744073709551615"},
                       {"frame_count": "513"}, {"membership_revision": "99"}):
            with self.subTest(change=change):
                header, body = target_fixture()
                header.update(change)
                with mock.patch("online_protocol.np.frombuffer", side_effect=AssertionError("allocated")):
                    with self.assertRaises(ValueError):
                        encode_message(header, body)

    def test_invalid_target_and_extra_fields_are_rejected(self):
        header, body = target_fixture()
        edits = [dict(header, unknown=1), dict(header, submap_state="anything"),
                 dict(header, action="reuse", asset_id="../elsewhere"),
                 dict(header, schema=True), dict(header, schema=1),
                 dict(header, generation="01")]
        for invalid in edits:
            with self.subTest(header=invalid):
                with self.assertRaises(ValueError):
                    encode_message(invalid, body)
        for payload in (body[:-1], body + b"x"):
            with self.assertRaises(ValueError):
                encode_message(header, payload)
        poses = np.frombuffer(body, dtype="<f8").copy().reshape(2, 4, 4)
        poses[0, 0, 3] = 1
        with self.assertRaises(ValueError):
            encode_message(header, poses.tobytes())
        poses[0] = np.eye(4)
        poses[1, 0, 0] = np.nan
        with self.assertRaises(ValueError):
            encode_message(header, poses.tobytes())

    def test_view_body_camera_relation_is_checked_independently(self):
        header, body = view_v2_fixture(frame=1)
        header["camera_from_body"][3] += 0.1
        with self.assertRaises(ValueError):
            encode_message(header, body)

    def test_reference_set_budget_order_and_path_are_checked(self):
        for values in ([str(i) for i in range(33)], ["2", "1"], ["1", "1"], ["01"]):
            with self.subTest(values=values):
                with self.assertRaises(ValueError):
                    encode_message(control_fixture("map_committed", map_sequence="1", asset_ids=values), b"")
        for path in ("relative/states", "/tmp/../states", "/tmp/other", "/tmp/states/"):
            with self.subTest(path=path):
                with self.assertRaises(ValueError):
                    encode_message({"schema": 2, "type": "hello", "session_id": SESSION,
                                    "state_directory": path}, b"")

    def test_zero_step_reuse_is_distinct_from_training(self):
        header = candidate_fixture()
        self.echo(header, bytes(3 * 48))
        for operation, steps in (("view", 0), ("rebuild", 20), ("reuse", 20)):
            with self.subTest(operation=operation):
                with self.assertRaises(ValueError):
                    encode_message(dict(header, operation=operation, steps=steps), bytes(3 * 48))
        self.echo(dict(header, operation="rebuild", steps=40), bytes(3 * 48))

    def test_candidate_depth_budget_and_operation_types_are_checked(self):
        cases = ({"operation": "view", "steps": 20, "supervised_depth_pixels": 20_000_001},
                 {"operation": "reuse", "steps": 0, "supervised_depth_pixels": 1},
                 {"operation": "rebuild", "steps": 40, "supervised_depth_pixels": 40_000_001},
                 {"operation": ["reuse"]})
        for fields in cases:
            with self.subTest(fields=fields):
                with self.assertRaises(ValueError):
                    encode_message(dict(candidate_fixture(), **fields), bytes(3 * 48))

    def test_both_languages_reject_nonidentity_anchor_noise(self):
        for scale, translation in ((1.0000002, [0, 0, 0]), (1.0, [0.75e-9] * 3)):
            with self.subTest(scale=scale, translation=translation):
                header, body = target_fixture()
                poses = np.frombuffer(body, dtype="<f8").copy().reshape(2, 4, 4)
                poses[0, 0, 0] = scale
                poses[0, :3, 3] = translation
                payload = poses.tobytes()
                with self.assertRaises(ValueError):
                    encode_message(header, payload)
                raw = json.dumps(header).encode()
                local, remote = socket.socketpair()
                with local, remote:
                    process = subprocess.Popen([os.environ["LIVO_WIRE_PEER"], str(remote.fileno())],
                                               pass_fds=(remote.fileno(),), stdout=subprocess.PIPE,
                                               stderr=subprocess.PIPE, text=True)
                    remote.close()
                    local.sendall(struct.pack("<IQ", len(raw), len(payload)) + raw + payload)
                    out, error = process.communicate(timeout=3)
                    self.assertNotEqual(process.returncode, 0, out + error)

    def test_cpu_target_decode_does_not_import_gpu_runtime(self):
        script = """
import sys
class BlockTorch:
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] == 'torch':
            raise AssertionError('protocol imported torch')
sys.meta_path.insert(0, BlockTorch())
from submap_fixtures import target_fixture
from online_protocol import decode_target
h,b = target_fixture()
assert decode_target(h,b)['primary'][1]['keyframe_id'] == '1'
"""
        result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
