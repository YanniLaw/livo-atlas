import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import threading
import unittest
from unittest import mock

import numpy as np

from online_protocol import decode_view, encode_message, read_message, write_message


def view_fixture():
    width, height = 8, 6
    header = {
        "schema": 1,
        "type": "view",
        "session_id": "a" * 32,
        "job_id": "2147483649",
        "generation": "1",
        "submap_id": "0",
        "pose_version": "0",
        "membership_revision": "1",
        "frame_count": "2147483650",
        "optimized_frame_count": "0",
        "keyframe_id": "2147483649",
        "calibration_id": "0",
        "parent_frame": "odom",
        "state_timestamp": 100.0,
        "cloud_timestamp": 99.99,
        "image_timestamp": 100.01,
        "inverse_exposure_time": 1.0,
        "width": width,
        "height": height,
        "fx": 4.0,
        "fy": 4.0,
        "cx": 3.5,
        "cy": 2.5,
        "camera_from_submap": [0, -1, 0, 1, 1, 0, 0, 2,
                               0, 0, 1, 3, 0, 0, 0, 1],
        "parent_from_submap": [1, 0, 0, 0, 0, 1, 0, 0,
                               0, 0, 1, 0, 0, 0, 0, 1],
    }
    bgr = np.empty((height, width, 3), dtype=np.uint8)
    bgr[:] = [10, 20, 30]
    image_mask = np.ones((height, width), dtype=np.uint8)
    depth_mask = np.zeros((height, width), dtype=np.uint8)
    depth_mask[3, 4] = 1
    depth = np.zeros((height, width), dtype="<f4")
    depth[3, 4] = 4.0
    return header, b"".join((bgr.tobytes(), image_mask.tobytes(),
                             depth_mask.tobytes(), depth.tobytes()))


class OnlineProtocolTest(unittest.TestCase):
    def round_trip(self, header, body):
        local, peer = socket.socketpair()
        self.addCleanup(local.close)
        self.addCleanup(peer.close)
        executable = Path(os.environ["LIVO_WIRE_PEER"])
        process = subprocess.Popen([str(executable), str(peer.fileno())],
                                   pass_fds=(peer.fileno(),),
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, text=True)
        peer.close()
        try:
            write_message(local, header, body, 2.0)
            actual_header, actual_body = read_message(local, 2.0)
        finally:
            if process.poll() is None:
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
            stdout, stderr = process.communicate(timeout=2)
        self.assertEqual(process.returncode, 0, stdout + stderr)
        self.assertEqual(actual_header, header)
        self.assertEqual(actual_body, body)
        return actual_header, actual_body

    def test_real_socket_cross_language_view_round_trip(self):
        header, body = view_fixture()
        actual_header, actual_body = self.round_trip(header, body)
        view = decode_view(actual_header, actual_body)
        self.assertEqual(view["keyframe_id"], 2147483649)
        np.testing.assert_allclose(view["rgb"][3, 4], [30/255, 20/255, 10/255])
        self.assertEqual(view["depth_m"][3, 4], 4)
        self.assertTrue(view["depth_mask"][3, 4])
        self.assertTrue(view["image_mask"][3, 4])
        np.testing.assert_array_equal(view["K"], [[4, 0, 3.5], [0, 4, 2.5], [0, 0, 1]])
        np.testing.assert_array_equal(
            view["camera_from_parent"],
            [[0, -1, 0, 1], [1, 0, 0, 2], [0, 0, 1, 3], [0, 0, 0, 1]],
        )
        for name in ("rgb", "image_mask", "depth_mask", "depth_m"):
            self.assertTrue(view[name].flags.owndata, name)

    def test_candidate_and_reset_cross_language_round_trip(self):
        header, _ = view_fixture()
        common = ("session_id", "job_id", "generation", "submap_id",
                  "pose_version", "membership_revision", "frame_count",
                  "optimized_frame_count", "width", "height")
        candidate = {name: header[name] for name in common}
        candidate.update({"schema": 1, "type": "candidate",
                          "frame_ids": ["2147483649"], "steps": 20,
                          "rgb_l1": 0.2, "depth_l1_m": 0.1,
                          "supervised_depth_pixels": 1, "gaussian_count": 1,
                          "nonzero_gradient": 1, "parameter_updated": 1,
                          "gpu_peak_allocated_bytes": "20",
                          "gpu_peak_reserved_bytes": "30",
                          "worker_peak_rss_bytes": "40"})
        self.round_trip(candidate, bytes([30, 20, 10]) * 48)
        for key, value in (("nonzero_gradient", 2), ("parameter_updated", True),
                           ("gpu_peak_allocated_bytes", "-1"),
                           ("gpu_peak_reserved_bytes", "bad"),
                           ("worker_peak_rss_bytes", 3)):
            with self.subTest(key=key), self.assertRaises(ValueError):
                encode_message(dict(candidate, **{key: value}), bytes([30, 20, 10]) * 48)
        reset = {"schema": 1, "type": "reset", "session_id": "a" * 32,
                 "generation": "2", "pose_version": "1"}
        self.round_trip(reset, b"")
        reset["submap_id"] = None
        with self.assertRaises(ValueError):
            encode_message(reset, b"")

    def test_utf8_worker_metadata_round_trips_between_languages(self):
        header = {"schema": 1, "type": "hello_ok", "session_id": "a" * 32,
                  "python": "3.10", "torch": "2.3", "gsplat": "1.5.3",
                  "cuda": "12.1", "device": "GPU 测试",
                  "gpu_free_bytes": "4294967296", "gpu_total_bytes": "8589934592"}
        self.round_trip(header, b"")

    def test_unicode_escape_is_rejected_for_opencv_compatibility(self):
        header = {"schema": 1, "type": "hello_ok", "session_id": "a" * 32,
                  "python": "3.10", "torch": "2.3", "gsplat": "1.5.3",
                  "cuda": "12.1", "device": "GPU\x01",
                  "gpu_free_bytes": "4294967296", "gpu_total_bytes": "8589934592"}
        with self.assertRaises(ValueError):
            encode_message(header, b"")
        document = (b'{"schema":1,"type":"hello","session_id":"' + b"a" * 32 +
                    b'","note":"\\u6d4b"}')
        local, peer = socket.socketpair()
        with local, peer:
            peer.sendall(struct.pack("<IQ", len(document), 0) + document)
            with self.assertRaises(ValueError):
                read_message(local, 0.5)

    def test_bad_masks_depth_dimensions_and_session_are_rejected(self):
        header, body = view_fixture()
        pixels = header["width"] * header["height"]
        invalid = bytearray(body)
        invalid[3 * pixels] = 255
        with self.assertRaises(ValueError):
            encode_message(header, bytes(invalid))
        invalid = bytearray(body)
        invalid[3 * pixels + 3 * header["width"] + 4] = 0
        with self.assertRaises(ValueError):
            encode_message(header, bytes(invalid))
        invalid = bytearray(body)
        depth_offset = 5 * pixels + 4 * (3 * header["width"] + 4)
        invalid[depth_offset:depth_offset + 4] = struct.pack("<f", float("nan"))
        with self.assertRaises(ValueError):
            encode_message(header, bytes(invalid))
        invalid_header = dict(header, width=2**63)
        with self.assertRaises(ValueError):
            encode_message(invalid_header, body)
        invalid_header = dict(header, session_id="bad")
        with self.assertRaises(ValueError):
            encode_message(invalid_header, body)

    def test_truncated_and_oversized_frames_fail_before_payload_allocation(self):
        samples = [
            struct.pack("<IQ", 65537, 0),
            struct.pack("<IQ", 1, 16 * 1024 * 1024 + 1),
            struct.pack("<IQ", 10, 4) + b"{}",
            b"\x01\x02",
        ]
        for data in samples:
            with self.subTest(data=data[:12]):
                local, peer = socket.socketpair()
                with local, peer:
                    peer.sendall(data)
                    peer.shutdown(socket.SHUT_WR)
                    with self.assertRaises((OSError, ValueError)):
                        read_message(local, 0.5)

    def test_invalid_json_and_nonfinite_values_are_rejected(self):
        header, body = view_fixture()
        header["fx"] = float("nan")
        with self.assertRaises(ValueError):
            encode_message(header, body)
        local, peer = socket.socketpair()
        with local, peer:
            document = b'{"schema":1,"type":"hello","session_id":NaN}'
            peer.sendall(struct.pack("<IQ", len(document), 0) + document)
            with self.assertRaises(ValueError):
                read_message(local, 0.5)

    def test_invalid_header_type_is_reported_as_invalid_input(self):
        header = {"schema": 1, "type": [], "session_id": "a" * 32}
        with self.assertRaises(ValueError):
            encode_message(header, b"")

    def test_timeout_and_closed_peer_fail(self):
        local, peer = socket.socketpair()
        with local, peer:
            with self.assertRaises((OSError, TimeoutError)):
                read_message(local, 0.05)
        local, peer = socket.socketpair()
        with local:
            peer.close()
            with self.assertRaises(OSError):
                write_message(local, {"schema": 1, "type": "hello",
                                      "session_id": "a" * 32}, b"", 0.5)

    def test_reads_fragmented_frame_and_write_timeout_on_full_socket(self):
        header, body = view_fixture()
        encoded = encode_message(header, body)
        local, peer = socket.socketpair()
        with local, peer:
            def send_parts():
                for byte in encoded:
                    peer.sendall(bytes([byte]))

            writer = threading.Thread(target=send_parts)
            writer.start()
            try:
                self.assertEqual(read_message(local, 2.0), (header, body))
            finally:
                writer.join(timeout=2)
            self.assertFalse(writer.is_alive())
        local, peer = socket.socketpair()
        with local, peer:
            local.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4096)
            local.setblocking(False)
            while True:
                try:
                    local.send(b"x" * 4096)
                except BlockingIOError:
                    break
            with self.assertRaises(TimeoutError):
                write_message(local, {"schema": 1, "type": "hello",
                                      "session_id": "a" * 32}, b"", 0.03)

    def test_deadline_applies_even_if_every_read_makes_progress(self):
        encoded = encode_message({"schema": 1, "type": "hello",
                                  "session_id": "a" * 32}, b"")

        class SlowSocket:
            def gettimeout(self):
                return None

            def setblocking(self, value):
                pass

            def settimeout(self, value):
                pass

            def recv(self, size):
                nonlocal encoded
                part, encoded = encoded[:1], encoded[1:]
                return part

        with mock.patch("online_protocol.time.monotonic", side_effect=[0.0, 0.2]):
            with self.assertRaises(TimeoutError):
                read_message(SlowSocket(), 0.1)


if __name__ == "__main__":
    unittest.main()
