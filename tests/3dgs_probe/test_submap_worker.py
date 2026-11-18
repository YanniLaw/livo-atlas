import copy
import gc
import hashlib
import os
from pathlib import Path
import resource
import socket
import sys
import tempfile
import threading
import unittest
from unittest import mock
import weakref

import numpy as np
import torch

from gaussian_state import CommitUncertainError, read_state
from online_model import OnlineGaussianModel, checked_cuda_device
from online_protocol import read_message, write_message
from online_worker import serve_connection
from submap_fixtures import SESSION, control_fixture, target_fixture, view_v2_fixture
from submap_worker import GaussianSubmapWorker
from train import _render_view


METADATA = {"python": sys.version.split()[0], "torch": str(torch.__version__),
            "gsplat": "1.5.3", "cuda": "12.1", "device": "test GPU",
            "gpu_free_bytes": "5000000000", "gpu_total_bytes": "8000000000"}


class SubmapWorkerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.device = checked_cuda_device()
        torch.set_num_threads(1)
        torch.cuda.reset_peak_memory_stats()

    @classmethod
    def tearDownClass(cls):
        print("Submap state GPU peak: allocated={} reserved={} RSS={}".format(
            torch.cuda.max_memory_allocated(), torch.cuda.max_memory_reserved(),
            resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024))

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="livo-submap-worker-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.directory = self.root / "states"
        self.actor = GaussianSubmapWorker(SESSION, self.directory, OnlineGaussianModel(self.device), METADATA)
        self.addCleanup(self.release)
        self.next_job = 1
        self.next_asset = 1
        self.map_sequence = 0
        self.frontier = 0
        self.generation = 0
        self.target = None
        self.target_body = None
        self.references = {}

    def release(self):
        if self.actor.model is not None:
            self.actor.model.reset(0)
        self.actor.confirmed_state = None
        self.actor.pending = None
        gc.collect()

    def job(self):
        value = self.next_job
        self.next_job += 1
        return value

    def activate(self, submap=0, first=0, count=2, action="empty", asset=None,
                 version=0, poses=None, parent=None):
        old = (self.target, self.target_body)
        self.generation += 1
        header, body = target_fixture(count=count, first=first, submap=submap,
                                      generation=self.generation, version=version,
                                      action=action, job=self.job())
        self.frontier = max(self.frontier, first + count)
        header.update(frame_count=str(self.frontier),
                      optimized_frame_count=str(self.frontier if version else 0))
        if poses is not None:
            body = np.asarray(poses, dtype="<f8").tobytes()
        if parent is not None:
            header["parent_from_submap"] = np.asarray(parent).ravel().tolist()
        if action != "empty":
            source = asset if asset is not None else self.actor.content
            header.update(source_generation=source["source_generation"], content_job_id=source["content_job_id"])
            if asset is not None:
                header.update(asset_id=asset["asset_id"], manifest_sha256=asset["manifest_sha256"])
        reply, preview = self.actor.handle(header, body)
        self.target, self.target_body = header, body
        if reply["type"] == "candidate":
            reply["_previous_test_target"] = old
        return reply, preview

    def decide(self, candidate, accept=True):
        fields = {name: candidate[name] for name in
                  ("job_id", "generation", "pose_version", "submap_id", "operation")}
        reply, body = self.actor.handle({"schema": 2, "type": "decision", "session_id": SESSION,
                                         **fields, "action": "accept" if accept else "discard"}, b"")
        self.assertEqual(reply["type"], "commit_ok" if accept else "discard_ok")
        if not accept and "_previous_test_target" in candidate:
            self.target, self.target_body = candidate["_previous_test_target"]
        return reply

    def view(self, frame, accept=True):
        target = self.target
        header, body = view_v2_fixture(frame, target["primary_count"], int(target["anchor_keyframe_id"]),
                                       int(target["submap_id"]), int(target["generation"]),
                                       int(target["pose_version"]), self.job())
        header.update({name: target[name] for name in
                       ("frame_count", "optimized_frame_count", "parent_from_submap")})
        poses = np.frombuffer(self.target_body, dtype="<f8").reshape(-1, 4, 4)
        local = frame - int(target["anchor_keyframe_id"])
        camera_body = np.asarray(header["camera_from_body"]).reshape(4, 4)
        header["camera_from_submap"] = (camera_body @ np.linalg.inv(poses[local])).ravel().tolist()
        reply, preview = self.actor.handle(header, body[:9*48] + self.target_body)
        self.assertEqual(reply["steps"], 20)
        if accept:
            self.decide(reply)
        return reply, preview

    def checkpoint(self, acknowledge=True):
        source = self.actor.content
        header = control_fixture("checkpoint", job=self.job(), generation=int(self.target["generation"]),
                                 version=int(self.target["pose_version"]), submap=int(source["submap_id"]),
                                 asset_id=str(self.next_asset), source_generation=source["source_generation"],
                                 content_job_id=source["content_job_id"])
        self.next_asset += 1
        reply, _ = self.actor.handle(header, b"")
        self.assertEqual(reply["type"], "checkpoint_ok")
        self.references[reply["submap_id"]] = reply["asset_id"]
        if acknowledge:
            self.acknowledge()
        return reply

    def acknowledge(self):
        self.map_sequence += 1
        header = control_fixture("map_committed", job=self.job(), generation=self.generation,
                                 version=int(self.target["pose_version"]), map_sequence=str(self.map_sequence),
                                 asset_ids=sorted(self.references.values(), key=int))
        reply, _ = self.actor.handle(header, b"")
        self.assertEqual(reply["type"], "map_committed_ok")

    def prune(self, asset):
        return self.actor.handle(control_fixture("prune", job=self.job(), generation=self.generation,
                                                version=int(self.target["pose_version"]),
                                                map_sequence=str(self.map_sequence), asset_id=asset["asset_id"]), b"")

    def assert_parameters(self, expected):
        for name, value in expected.items():
            np.testing.assert_array_equal(self.actor.model.parameters[name].detach().cpu().numpy(), value)

    def enable_spatial(self):
        self.actor.model.reset(0)
        self.directory.rmdir()
        self.actor = GaussianSubmapWorker(SESSION, self.directory, OnlineGaussianModel(self.device),
            METADATA, joint_render_enabled=True, spatial_render_enabled=True)

    def test_spatial_metadata_waits_for_accept_and_reuses_one_candidate_capture(self):
        self.enable_spatial()
        self.activate()
        candidate, _ = self.view(0, accept=False)
        self.assertIsNone(self.actor.confirmed_metadata)
        self.assertEqual(candidate['render_metadata']['camera']['keyframe_id'], '0')
        self.decide(candidate, False)
        self.assertIsNone(self.actor.confirmed_metadata)
        self.assertIsNone(self.actor.pending)
        candidate, _ = self.view(0, accept=False)
        captured = self.actor.pending['state']
        self.assertTrue(self.actor.model.candidate_pending)
        self.decide(candidate)
        self.assertIs(self.actor.confirmed_state, captured)
        self.assertEqual(self.actor.confirmed_metadata, candidate['render_metadata'])
        self.assertEqual(self.actor.confirmed_metadata['content']['content_job_id'], candidate['job_id'])

    def test_spatial_metadata_follows_reuse_rebuild_checkpoint_and_prune(self):
        self.enable_spatial()
        self.activate()
        self.view(0)
        self.view(1)
        before = copy.deepcopy(self.actor.confirmed_metadata)
        asset = self.checkpoint()
        self.assertEqual(self.actor.assets[int(asset['asset_id'])]['render_metadata'], before)
        self.assertNotIn('render_metadata', read_state(self.directory / asset['asset_id']).metadata)
        candidate, _ = self.activate(action='reuse', version=1)
        self.assertEqual(candidate['render_metadata'], before)
        self.decide(candidate, False)
        self.assertEqual(self.actor.confirmed_metadata, before)
        candidate, _ = self.activate(action='reuse', version=1)
        self.decide(candidate)
        self.assertEqual(self.actor.confirmed_metadata, before)
        poses = np.frombuffer(self.target_body, dtype='<f8').copy().reshape(2, 4, 4)
        poses[1, 1, 3] = .5
        candidate, _ = self.activate(action='rebuild', version=2, poses=poses)
        self.assertEqual(self.actor.confirmed_metadata, before)
        self.assertEqual(candidate['render_metadata']['content']['source_generation'], candidate['generation'])
        self.decide(candidate)
        self.assertNotEqual(self.actor.confirmed_metadata['content'], before['content'])
        replacement = self.checkpoint()
        self.prune(asset)
        self.assertNotIn(int(asset['asset_id']), self.actor.assets)
        self.assertEqual(self.actor.assets[int(replacement['asset_id'])]['render_metadata'],
                         self.actor.confirmed_metadata)
        self.activate(submap=1, first=2, count=1, version=2)
        self.assertIsNone(self.actor.confirmed_metadata)

    def test_rejected_view_restores_prior_confirmed_parameters(self):
        self.activate()
        self.view(0)
        before = copy.deepcopy(self.actor.confirmed_state)
        with torch.no_grad():
            renders = [value.cpu().numpy().copy() for value in
                       _render_view(self.actor.model.parameters, self.actor.model.views[-1])]
        candidate, _ = self.view(1, accept=False)
        self.assertTrue(self.actor.model.candidate_pending)
        self.assertEqual(self.actor.confirmed_state.origin_ids.tolist(), [0])
        self.decide(candidate, False)
        self.assert_parameters(before.parameters)
        self.assertIsNone(self.actor.model.optimizer)
        with torch.no_grad():
            restored = [value.cpu().numpy().copy() for value in
                        _render_view(self.actor.model.parameters, self.actor.model.views[-1])]
        for index, (left, right) in enumerate(zip(renders, restored)):
            np.testing.assert_allclose(left, right, atol=1e-5 if index == 2 else 1e-6,
                                       rtol=1e-6 if index == 2 else 0)

    def two_stored_blocks(self):
        self.activate(count=1)
        self.view(0)
        a = self.checkpoint()
        self.activate(submap=1, first=1, count=1)
        self.view(1)
        b = self.checkpoint()
        return a, b

    def test_rejected_activation_of_b_restores_a(self):
        a, b = self.two_stored_blocks()
        candidate, _ = self.activate(count=1, action="reuse", asset=a)
        self.decide(candidate)
        expected = copy.deepcopy(self.actor.confirmed_state)
        candidate, _ = self.activate(submap=1, first=1, count=1, action="reuse", asset=b)
        self.decide(candidate, False)
        self.assertEqual(self.actor.content["submap_id"], "0")
        self.assertEqual(self.actor.model.frame_ids, [0])
        self.assert_parameters(expected.parameters)

    def test_rollback_rebinds_accepted_target(self):
        self.activate()
        self.view(0)
        self.checkpoint()
        candidate, _ = self.activate(action="reuse", version=1)
        self.decide(candidate)
        self.assertEqual(self.actor.confirmed_state.metadata["generation"], "1")
        candidate, _ = self.view(1, accept=False)
        self.decide(candidate, False)
        self.assertEqual(self.actor.model.generation, 2)
        self.assertEqual(self.actor.model.views[0]["pose_version"], "1")
        self.assertEqual(self.actor.confirmed_binding["generation"], "2")

    def test_first_checkpoint_requires_map_acknowledgement(self):
        self.activate(count=1)
        self.view(0)
        expected = copy.deepcopy(self.actor.confirmed_state.parameters)
        asset = self.checkpoint(acknowledge=False)
        with self.assertRaises((ValueError, RuntimeError)):
            self.activate(submap=1, first=1, count=1)
        self.assert_parameters(expected)
        self.assertTrue((self.directory / asset["asset_id"] / "manifest.json").is_file())

    def test_rigid_reuse_preserves_arrays_without_rewrite(self):
        self.activate()
        self.view(0)
        asset = self.checkpoint()
        path = self.directory / asset["asset_id"]
        hashes = {file.name: hashlib.sha256(file.read_bytes()).hexdigest() for file in path.iterdir()}
        state = self.actor.confirmed_state
        content = dict(self.actor.content)
        pointers = {name: value.data_ptr() for name, value in self.actor.model.parameters.items()}
        parent = np.asarray(self.target["parent_from_submap"]).reshape(4, 4).copy()
        parent[0, 3] += 10
        candidate, _ = self.activate(action="reuse", version=1, parent=parent)
        self.assertEqual(candidate["steps"], 0)
        self.decide(candidate)
        self.assertIs(self.actor.confirmed_state, state)
        self.assertEqual(self.actor.content, content)
        self.assertEqual({name: value.data_ptr() for name, value in self.actor.model.parameters.items()}, pointers)
        self.assertEqual({file.name: hashlib.sha256(file.read_bytes()).hexdigest() for file in path.iterdir()}, hashes)
        self.assertEqual(len(list(self.directory.iterdir())), 1)

    def test_evicted_member_change_requires_rebuild(self):
        self.activate(count=12)
        for frame in range(12):
            self.view(frame)
        self.assertEqual(self.actor.model.frame_ids, list(range(2, 12)))
        self.checkpoint()
        before = copy.deepcopy(self.actor.confirmed_state)
        poses = np.frombuffer(self.target_body, dtype="<f8").copy().reshape(12, 4, 4)
        poses[1, 1, 3] = 0.5
        candidate, _ = self.activate(count=12, action="rebuild", version=1, poses=poses)
        self.assertEqual(candidate["steps"], 200)
        self.assertEqual(candidate["nonzero_gradient"], 1)
        self.assertEqual(candidate["parameter_updated"], 1)
        self.assertEqual(self.actor.confirmed_state.metadata["pose_version"], "0")
        self.decide(candidate)
        state = self.actor.confirmed_state
        self.assertEqual(state.metadata["pose_version"], "1")
        self.assertEqual(state.metadata["primary"][1]["submap_from_body"][7], 0.5)
        for left, right in zip(before.views, state.views):
            for name in ("rgb", "depth_m", "image_mask", "depth_mask", "K", "camera_from_body"):
                np.testing.assert_array_equal(left[name], right[name])

    def test_rebuild_rejection_never_captures_intermediate_models(self):
        self.activate()
        self.view(0)
        self.view(1)
        self.checkpoint()
        expected = copy.deepcopy(self.actor.confirmed_state.parameters)
        poses = np.frombuffer(self.target_body, dtype="<f8").copy().reshape(2, 4, 4)
        poses[1, 1, 3] = 0.5
        calls = []
        original = OnlineGaussianModel.capture_state

        def capture(model, context):
            calls.append(context)
            return original(model, context)

        with mock.patch.object(OnlineGaussianModel, "capture_state", new=capture):
            candidate, _ = self.activate(action="rebuild", version=1, poses=poses)
            self.assertEqual(calls, [])
            self.decide(candidate, False)
            self.assertEqual(calls, [])
        self.assert_parameters(expected)
        self.assertEqual(self.actor.confirmed_binding["pose_version"], "0")

    def test_release_precedes_gpu_load(self):
        a, _ = self.two_stored_blocks()
        old_parameters = [weakref.ref(value) for value in self.actor.model.parameters.values()]
        original = OnlineGaussianModel.from_state.__func__
        released = []

        def restore(cls, state, device):
            gc.collect()
            released.append(all(value() is None for value in old_parameters))
            return original(cls, state, device)

        with mock.patch.object(OnlineGaussianModel, "from_state", new=classmethod(restore)):
            candidate, _ = self.activate(count=1, action="reuse", asset=a)
            self.decide(candidate)
        self.assertEqual(released, [True])

    def test_corrupt_source_is_rejected_before_releasing_resident_gpu(self):
        a, _ = self.two_stored_blocks()
        pointers = {name: value.data_ptr() for name, value in self.actor.model.parameters.items()}
        (self.directory / a["asset_id"] / "gaussians.npz").write_bytes(b"truncated")
        with self.assertRaises((ValueError, OSError)):
            self.activate(count=1, action="reuse", asset=a)
        self.assertEqual({name: value.data_ptr() for name, value in self.actor.model.parameters.items()}, pointers)
        self.assertEqual(self.actor.content["submap_id"], "1")

    def test_thirty_third_block_fails_before_discarding_confirmed_content(self):
        for submap in range(32):
            self.activate(submap=submap, first=submap, count=1)
            self.view(submap)
            self.checkpoint()
        pointers = {name: value.data_ptr() for name, value in self.actor.model.parameters.items()}
        with self.assertRaises(RuntimeError):
            self.activate(submap=32, first=32, count=1)
        self.assertEqual(len(list(self.directory.iterdir())), 32)
        self.assertEqual(self.actor.content["submap_id"], "31")
        self.assertEqual({name: value.data_ptr() for name, value in self.actor.model.parameters.items()}, pointers)

    def test_training_after_tiny_rigid_tolerance_uses_exact_target_cameras(self):
        self.activate(count=512)
        self.view(0)
        self.view(255)
        self.checkpoint()
        poses = np.frombuffer(self.target_body, dtype="<f8").copy().reshape(512, 4, 4)
        # An allowed half-nanoradian body rotation at 255 m changes the
        # camera translation by more than the asset's 1e-8 consistency bound.
        poses[255, :3, :3] = [[1, 0, 0.5e-9], [0, 1, 0], [-0.5e-9, 0, 1]]
        candidate, _ = self.activate(count=512, action="reuse", version=1, poses=poses)
        self.decide(candidate)
        self.view(256)
        self.assertEqual(self.actor.confirmed_state.metadata["pose_version"], "1")
        self.assertEqual([view["keyframe_id"] for view in self.actor.confirmed_state.views], [0, 255, 256])

    def test_prune_requires_durable_reference_removal(self):
        self.activate()
        self.view(0)
        old = self.checkpoint()
        self.view(1)
        new = self.checkpoint(acknowledge=False)
        with self.assertRaises((ValueError, RuntimeError)):
            self.prune(old)
        self.assertTrue((self.directory / old["asset_id"]).is_dir())
        self.assertTrue((self.directory / new["asset_id"]).is_dir())

    def test_prune_after_ack_removes_only_replaced_asset(self):
        self.activate()
        self.view(0)
        old = self.checkpoint()
        self.view(1)
        new = self.checkpoint()
        reply, _ = self.prune(old)
        self.assertEqual(reply["type"], "prune_ok")
        self.assertFalse((self.directory / old["asset_id"]).exists())
        self.assertEqual(read_state(self.directory / new["asset_id"]).origin_ids.tolist(), [0, 1])

    def test_sync_uncertain_preserves_both_assets(self):
        self.activate()
        self.view(0)
        old = self.checkpoint()
        self.view(1)
        with mock.patch("gaussian_state._sync_directory", side_effect=OSError("Injected directory sync failure")):
            with self.assertRaises(CommitUncertainError):
                self.checkpoint()
        self.assertEqual(read_state(self.directory / old["asset_id"]).origin_ids.tolist(), [0])
        self.assertEqual(read_state(self.directory / "2").origin_ids.tolist(), [0, 1])
        with self.assertRaises(RuntimeError):
            self.prune(old)
        self.assertTrue((self.directory / old["asset_id"]).is_dir())

    def test_schema_two_server_confirms_real_gpu_training(self):
        self.server_roundtrip()

    def test_spatial_server_negotiates_and_transports_candidate_metadata(self):
        self.server_roundtrip(['joint_render_v1', 'joint_render_spatial_v1'])

    def server_roundtrip(self, capabilities=None):
        parent = self.root / "server"
        parent.mkdir()
        client, server = socket.socketpair()
        results = []

        def run():
            with server:
                results.append(serve_connection(server, lambda: (OnlineGaussianModel(self.device), METADATA), 5))

        thread = threading.Thread(target=run, daemon=True)
        thread.start()
        try:
            greeting = {"schema": 2, "type": "hello", "session_id": SESSION,
                        "state_directory": str(parent / "states")}
            if capabilities is not None:
                greeting['capabilities'] = capabilities
            write_message(client, greeting, b"", 5)
            hello, _ = read_message(client, 10)
            self.assertEqual(hello["type"], "hello_ok", hello)
            self.assertEqual(hello["schema"], 2)
            self.assertEqual(hello["mode"], "persistent_submaps")
            if capabilities is None:
                self.assertNotIn('capabilities', hello)
            else:
                self.assertEqual(hello['capabilities'], capabilities)
            header, body = target_fixture(count=1)
            write_message(client, header, body, 5)
            self.assertEqual(read_message(client, 5)[0]["type"], "activated")
            view, body = view_v2_fixture(count=1, job=2)
            write_message(client, view, body, 5)
            candidate, _ = read_message(client, 10)
            self.assertEqual(candidate["type"], "candidate")
            self.assertEqual(candidate["nonzero_gradient"], 1)
            if capabilities is None:
                self.assertNotIn('render_metadata', candidate)
            else:
                metadata = candidate['render_metadata']
                self.assertEqual(metadata['camera']['keyframe_id'], '0')
                self.assertEqual(metadata['camera']['K'][2], 3.5)
                self.assertEqual(metadata['content'], dict(session_id=SESSION, submap_id='0',
                    source_generation='1', content_job_id='2'))
            decision = control_fixture("decision", job=2, submap=0, operation="view", action="accept")
            write_message(client, decision, b"", 5)
            self.assertEqual(read_message(client, 5)[0]["type"], "commit_ok")
            write_message(client, control_fixture("stop", job=3, map_sequence="0", incomplete=1), b"", 5)
            self.assertEqual(read_message(client, 5)[0]["type"], "stopped")
        finally:
            client.close()
            thread.join(10)
        self.assertFalse(thread.is_alive())
        self.assertEqual(results, [True])


if __name__ == "__main__":
    unittest.main()
