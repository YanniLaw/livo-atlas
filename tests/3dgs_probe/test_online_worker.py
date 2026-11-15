import os
import socket
import subprocess
import sys
import threading
import tempfile
import unittest
from pathlib import Path

import numpy as np

from online_protocol import read_message, write_message
from online_worker import serve, serve_connection
from test_online_protocol import view_fixture


SESSION = "a" * 32
METADATA = {"python": "test", "torch": "test", "gsplat": "test",
            "cuda": "test", "device": "test", "gpu_free_bytes": "5000000000",
            "gpu_total_bytes": "8000000000"}


class FakeModel:
    def __init__(self):
        self.generation = None
        self.frames = []
        self.optimizer = None
        self.pending = False
        self.resets = []

    def reset(self, generation):
        self.generation = generation
        self.resets.append(generation)
        self.frames = []
        self.optimizer = None
        self.pending = False

    def train_candidate(self, view):
        self.frames.append(view["keyframe_id"])
        self.optimizer = object()
        self.pending = True
        image = np.full((*view["rgb"].shape[:2], 3), 40, dtype=np.uint8)
        return image, {"frame_ids": list(self.frames), "steps": 20,
                       "rgb_l1": 0.2, "depth_l1_m": 0.1,
                       "supervised_depth_pixels": 20, "gaussian_count": 1}

    def decide(self, accept):
        if not self.pending:
            raise RuntimeError("No pending candidate")
        if accept:
            self.pending = False
        else:
            self.reset(self.generation)


class OnlineWorkerTest(unittest.TestCase):
    def setUp(self):
        self.client, server = socket.socketpair()
        self.model = FakeModel()
        self.thread = threading.Thread(target=self.run_server, args=(server,), daemon=True)
        self.thread.start()
        self.send("hello")
        self.assertEqual(self.receive()["type"], "hello_ok")

    def run_server(self, server):
        with server:
            serve_connection(server, lambda: (self.model, METADATA), transfer_timeout=1)

    def tearDown(self):
        self.client.close()
        self.thread.join(2)
        self.assertFalse(self.thread.is_alive())

    def send(self, kind, **fields):
        header = {"schema": 1, "type": kind, "session_id": SESSION, **fields}
        write_message(self.client, header, b"", 1)

    def receive(self):
        header, _ = read_message(self.client, 2)
        return header

    def reset(self, generation="1", pose_version="0", submap_id="0"):
        self.send("reset", generation=generation, pose_version=pose_version,
                  submap_id=submap_id)
        self.assertEqual(self.receive()["type"], "reset_ok")

    def view(self, job="1", frame="0", generation="1", pose_version="0"):
        header, body = view_fixture()
        header.update(job_id=job, keyframe_id=frame, generation=generation,
                      frame_count=str(int(frame) + 1), pose_version=pose_version,
                      parent_frame="odom" if pose_version == "0" else "map",
                      optimized_frame_count="0")
        write_message(self.client, header, body, 1)
        return header

    def decision(self, view, action):
        fields = {key: view[key] for key in
                  ("job_id", "generation", "submap_id", "pose_version")}
        self.send("decision", action=action, **fields)

    def test_confirmation_and_idle_reset_clear_model_and_optimizer(self):
        self.reset()
        view = self.view()
        candidate = self.receive()
        self.assertEqual(candidate["type"], "candidate")
        self.assertEqual(candidate["frame_ids"], ["0"])
        self.decision(view, "accept")
        self.assertEqual(self.receive()["type"], "commit_ok")
        self.assertIsNotNone(self.model.optimizer)
        self.reset("3", "2")
        self.assertEqual(self.model.resets, [1, 3])
        self.assertEqual(self.model.frames, [])
        self.assertIsNone(self.model.optimizer)
        self.send("stop")

    def test_discard_clears_all_mutated_state(self):
        self.reset()
        view = self.view()
        self.receive()
        self.decision(view, "discard")
        self.assertEqual(self.receive()["type"], "discard_ok")
        self.assertEqual(self.model.frames, [])
        self.assertIsNone(self.model.optimizer)
        self.send("stop")

    def test_duplicate_job_is_protocol_error(self):
        self.reset()
        view = self.view()
        self.receive()
        self.decision(view, "accept")
        self.receive()
        self.view()
        self.assertEqual(self.receive()["code"], "protocol_error")

    def test_reset_while_candidate_undecided_is_protocol_error(self):
        self.reset()
        self.view()
        self.receive()
        self.send("reset", generation="2", pose_version="1", submap_id="0")
        self.assertEqual(self.receive()["code"], "protocol_error")

    def test_wrong_candidate_identity_is_rejected(self):
        self.reset()
        view = self.view()
        self.receive()
        view["job_id"] = "2"
        self.decision(view, "accept")
        self.assertEqual(self.receive()["code"], "protocol_error")

    def test_wrong_session_is_rejected(self):
        self.reset()
        header = {"schema": 1, "type": "reset", "session_id": "b" * 32,
                  "generation": "2", "pose_version": "1", "submap_id": "0"}
        write_message(self.client, header, b"", 1)
        self.assertEqual(self.receive()["code"], "protocol_error")

    def test_view_must_match_latest_reset(self):
        self.reset()
        self.view(generation="2")
        self.assertEqual(self.receive()["code"], "protocol_error")


class OnlineWorkerStartupTest(unittest.TestCase):
    def test_hidden_gpu_fails_without_falling_back_to_cpu(self):
        client, server = socket.socketpair()
        with client, server:
            environment = dict(os.environ, CUDA_VISIBLE_DEVICES="")
            process = subprocess.Popen(
                [sys.executable, "-c",
                 "import socket,sys; from online_worker import serve_connection; "
                 "sys.exit(0 if serve_connection(socket.socket(fileno=int(sys.argv[1]))) else 1)",
                 str(server.fileno())], pass_fds=(server.fileno(),), env=environment,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            server.close()
            try:
                write_message(client, {"schema": 1, "type": "hello",
                                       "session_id": SESSION}, b"", 5)
                header, _ = read_message(client, 20)
                self.assertEqual(header["code"], "gpu_unavailable")
                _, error = process.communicate(timeout=20)
                self.assertEqual(process.returncode, 1, error)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait(timeout=5)

    def test_initialization_failure_is_reported_to_the_producer(self):
        for text, expected in (("CUDA is required", "gpu_unavailable"),
                               ("gsplat.csrc missing", "extension_unavailable")):
            with self.subTest(expected=expected):
                client, server = socket.socketpair()

                def fail():
                    raise RuntimeError(text)

                thread = threading.Thread(target=serve_connection,
                                          args=(server, fail, 1), daemon=True)
                thread.start()
                with client, server:
                    write_message(client, {"schema": 1, "type": "hello",
                                           "session_id": SESSION}, b"", 1)
                    header, _ = read_message(client, 1)
                    self.assertEqual(header["code"], expected)
                    thread.join(2)
                    self.assertFalse(thread.is_alive())

    def test_existing_path_is_never_removed(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "worker.sock"
            path.write_text("keep")
            with self.assertRaises(FileExistsError):
                serve(path)
            self.assertEqual(path.read_text(), "keep")


if __name__ == "__main__":
    unittest.main()
