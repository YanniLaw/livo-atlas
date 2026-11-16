import copy
import gc
import os
from pathlib import Path
import resource
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
import weakref

import numpy as np
import torch

from gaussian_state import read_state, write_state
from online_model import OnlineGaussianModel, checked_cuda_device
from state_fixtures import context_for_model, make_state
from test_online_model import make_view
from train import _render_view


class ModelStateTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not torch.cuda.is_available():
            if os.environ.get("LIVO_REQUIRE_GPU") == "1":
                raise RuntimeError("CUDA is required")
            raise unittest.SkipTest("CUDA is not available")
        cls.device = checked_cuda_device()
        torch.set_num_threads(1)
        torch.cuda.reset_peak_memory_stats()

    @classmethod
    def tearDownClass(cls):
        print("State GPU test peak bytes: allocated={} reserved={} RSS={}".format(
            torch.cuda.max_memory_allocated(), torch.cuda.max_memory_reserved(),
            resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024))

    def model(self, frames=(0,)):
        model = OnlineGaussianModel(self.device)
        model.reset(1)
        for frame in frames:
            model.train_candidate(make_view(frame))
            model.decide(True)
        return model

    def test_capture_rejects_empty_and_pending_candidates(self):
        model = OnlineGaussianModel(self.device)
        model.reset(1)
        with self.assertRaises(ValueError):
            model.capture_state(context_for_model([0]))
        model.train_candidate(make_view(0))
        with self.assertRaises(ValueError):
            model.capture_state(context_for_model([0]))
        model.decide(True)
        self.assertEqual(model.capture_state(context_for_model([0])).origin_ids.tolist(), [0])
        model.train_candidate(make_view(1))
        model.decide(False)
        with self.assertRaises(ValueError):
            model.capture_state(context_for_model([0]))

    def test_capture_owns_arrays_after_model_training_and_reset(self):
        model = self.model()
        context = context_for_model([0], count=2)
        saved = model.capture_state(context)
        original = copy.deepcopy(saved)
        context["primary"][0]["submap_from_body"][3] = 999
        model.train_candidate(make_view(1))
        model.decide(True)
        model.reset(2)
        self.assertEqual(saved.metadata, original.metadata)
        for name in saved.parameters:
            np.testing.assert_array_equal(saved.parameters[name], original.parameters[name])
        np.testing.assert_array_equal(saved.views[0]["rgb"], original.views[0]["rgb"])

    def test_restore_preserves_raw_parameters_and_renders(self):
        model = self.model((0, 1))
        before = model.render_preview(1)
        with torch.no_grad():
            floating = [value.cpu().numpy().copy() for value in
                        _render_view(model.parameters, model.views[-1])]
        state = model.capture_state(context_for_model([0, 1]))
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "state"
            write_state(path, state)
            old = weakref.ref(model)
            allocated = torch.cuda.memory_allocated()
            del model
            gc.collect()
            torch.cuda.synchronize()
            self.assertIsNone(old())
            self.assertLess(torch.cuda.memory_allocated(), allocated)
            restored = OnlineGaussianModel.from_state(read_state(path), self.device)
        for name, value in state.parameters.items():
            np.testing.assert_array_equal(restored.parameters[name].detach().cpu().numpy(), value)
        self.assertFalse(restored.parameters["quats"].requires_grad)
        np.testing.assert_array_equal(restored.origin_ids.cpu().numpy(), state.origin_ids)
        self.assertLessEqual(np.abs(restored.render_preview(1).astype(int) - before).max(), 1)
        with torch.no_grad():
            after = [value.cpu().numpy() for value in _render_view(restored.parameters, restored.views[-1])]
        for index, (a, b) in enumerate(zip(floating, after)):
            np.testing.assert_allclose(a, b, atol=1e-5 if index == 2 else 1e-6,
                                       rtol=1e-6 if index == 2 else 0)
        print("Cold restore float max differences (RGB, alpha, z):", [
            float(np.max(np.abs(a - b))) for a, b in zip(floating, after)])

    def test_restore_resets_adam_and_continues_bounded_training(self):
        model = OnlineGaussianModel(self.device)
        model.reset(1)
        for frame in range(10):
            model.train_candidate(make_view(frame, width=1201, height=1, dense=True))
            model.decide(True)
        state = model.capture_state(context_for_model(list(range(10)), count=11))
        del model
        gc.collect()
        restored = OnlineGaussianModel.from_state(state, self.device)
        self.assertIsNone(restored.optimizer)
        self.assertFalse(restored.candidate_pending)
        _, metrics = restored.train_candidate(make_view(10, width=1201, height=1, dense=True))
        self.assertEqual(metrics["steps"], 20)
        self.assertEqual(metrics["gaussian_count"], 10_000)
        self.assertEqual(metrics["frame_ids"], list(range(1, 11)))
        self.assertTrue(metrics["nonzero_gradient"])
        self.assertTrue(metrics["parameter_updated"])
        restored.decide(True)
        for state in restored.optimizer.state.values():
            self.assertEqual(state["step"].item(), 20)

    def test_bad_state_is_rejected_before_cuda_allocation(self):
        state = make_state()
        state.parameters["means"][0, 0] = np.inf
        with mock.patch("online_model.checked_cuda_device", side_effect=AssertionError("GPU allocation")):
            with self.assertRaises(ValueError):
                OnlineGaussianModel.from_state(state, self.device)

    def test_missing_gpu_fails_without_cpu_fallback(self):
        script = """
from online_model import OnlineGaussianModel
from state_fixtures import make_state
OnlineGaussianModel.from_state(make_state(), 'cuda:0')
"""
        result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True,
                                env=dict(os.environ, CUDA_VISIBLE_DEVICES=""))
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("CUDA is required", result.stderr)

    def test_capture_checks_independent_camera_relation(self):
        model = self.model((0, 1))
        context = context_for_model([0, 1])
        context["primary"][1]["submap_from_body"][3] = 2
        with self.assertRaises(ValueError):
            model.capture_state(context)

    def test_capture_accepts_older_frontiers_but_rejects_identity_conflicts(self):
        model = self.model((0, 1))
        context = context_for_model([0, 1], count=3)
        saved = model.capture_state(context)
        self.assertEqual(saved.metadata["frame_count"], "3")
        for key, value in (("origin_session_id", "b" * 32), ("generation", "2"),
                           ("submap_id", "1"), ("pose_version", "1")):
            with self.subTest(key=key):
                invalid = copy.deepcopy(context)
                invalid[key] = value
                with self.assertRaises(ValueError):
                    model.capture_state(invalid)
        restored = OnlineGaussianModel.from_state(saved, self.device)
        self.assertEqual([view["frame_count"] for view in restored.views], ["3", "3"])


if __name__ == "__main__":
    unittest.main()
