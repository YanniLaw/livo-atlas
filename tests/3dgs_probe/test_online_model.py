import os
import subprocess
import sys
import unittest
from unittest import mock

import numpy as np
import torch

from online_model import OnlineGaussianModel, _render_view, checked_cuda_device
from online_protocol import decode_view


def make_view(frame_id, width=8, height=6, dense=False):
    header = {
        "schema": 1, "type": "view", "session_id": "a" * 32,
        "job_id": str(frame_id), "generation": "1", "submap_id": "0",
        "pose_version": "0", "membership_revision": str(frame_id + 1),
        "frame_count": str(frame_id + 1), "optimized_frame_count": "0",
        "keyframe_id": str(frame_id), "calibration_id": "0",
        "parent_frame": "odom", "state_timestamp": 100.0 + frame_id,
        "cloud_timestamp": 100.0 + frame_id, "image_timestamp": 100.0 + frame_id,
        "inverse_exposure_time": 1.0, "width": width, "height": height,
        "fx": 4.0, "fy": 4.0, "cx": 3.5 if height == 6 else 0.0,
        "cy": 2.5 if height == 6 else 0.0,
        "camera_from_submap": [0, -1, 0, 1, 1, 0, 0, 2,
                               0, 0, 1, 3, 0, 0, 0, 1],
        "parent_from_submap": [1, 0, 0, 0, 0, 1, 0, 0,
                               0, 0, 1, 0, 0, 0, 0, 1],
    }
    bgr = np.empty((height, width, 3), dtype=np.uint8)
    bgr[:] = [10, 20, 30]
    mask = np.ones((height, width), dtype=np.uint8)
    depth_mask = np.ones((height, width), dtype=np.uint8) if dense else np.zeros(
        (height, width), dtype=np.uint8
    )
    if not dense:
        depth_mask[3, 4] = 1
    depth = np.where(depth_mask, 4.0, 0.0).astype("<f4")
    body = b"".join((bgr.tobytes(), mask.tobytes(), depth_mask.tobytes(),
                     depth.tobytes()))
    return decode_view(header, body)


class OnlineGaussianModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not torch.cuda.is_available():
            if os.environ.get("LIVO_REQUIRE_GPU") == "1":
                raise RuntimeError("CUDA is required for this test")
            raise unittest.SkipTest("CUDA is not available")
        cls.device = checked_cuda_device()
        torch.cuda.reset_peak_memory_stats()
        torch.set_num_threads(1)

    @classmethod
    def tearDownClass(cls):
        torch.cuda.synchronize()
        print("GPU peak bytes: allocated={}, reserved={}".format(
            torch.cuda.max_memory_allocated(), torch.cuda.max_memory_reserved()
        ))

    def test_local_pose_rgb_depth_and_training_order(self):
        model = OnlineGaussianModel(self.device)
        model.reset(1)
        view = make_view(0)
        preview, metrics = model.train_candidate(view)
        self.assertEqual(preview.shape, (6, 8, 3))
        self.assertEqual(preview.dtype, np.uint8)
        self.assertEqual(metrics["steps"], 20)
        self.assertEqual(metrics["training_frame_ids"], [0] * 20)
        self.assertGreater(metrics["supervised_depth_pixels"], 0)
        self.assertTrue(metrics["nonzero_gradient"])
        self.assertTrue(metrics["parameter_updated"])
        self.assertTrue(np.isfinite(metrics["rgb_l1"]))
        np.testing.assert_allclose(view["rgb"][3, 4], [30/255, 20/255, 10/255])
        np.testing.assert_allclose(model.parameters["means"].detach().cpu().numpy()[0],
                                   [-1.5, 0.5, 1.0], atol=0.011, rtol=0)
        model.decide(True)
        _, metrics = model.train_candidate(make_view(1))
        self.assertEqual(metrics["training_frame_ids"], [1] * 10 + [0] * 10)
        for state in model.optimizer.state.values():
            self.assertEqual(state["step"].item(), 20)
        model.decide(True)

    def test_bounded_window_evicts_oldest_origins_and_resets_adam(self):
        model = OnlineGaussianModel(self.device)
        model.reset(1)
        for frame_id in range(11):
            _, metrics = model.train_candidate(
                make_view(frame_id, width=1201, height=1, dense=True)
            )
            self.assertEqual(metrics["steps"], 20)
            self.assertLessEqual(metrics["gaussian_count"], 10_000)
            self.assertLessEqual(len(model.frame_ids), 10)
            model.decide(True)
        self.assertEqual(model.frame_ids, list(range(1, 11)))
        origins = model.origin_ids.detach().cpu().numpy()
        self.assertEqual(len(origins), 10_000)
        self.assertEqual(set(origins), set(range(1, 11)))
        for frame_id in range(1, 11):
            self.assertEqual(int((origins == frame_id).sum()), 1000)
        for state in model.optimizer.state.values():
            self.assertEqual(state["step"].item(), 20)

    def test_coverage_skips_render_for_empty_model_or_adequate_budget(self):
        model = OnlineGaussianModel(self.device)
        model.reset(1)
        with mock.patch("online_model._MAX_PER_VIEW", 4):
            for view, count in ((make_view(0, dense=True), 48), (make_view(1), 1)):
                with mock.patch("online_model._render_view", wraps=_render_view) as render:
                    _, metrics = model.train_candidate(view, seed_policy="coverage")
                self.assertEqual(render.call_count, 21)
                self.assertEqual(metrics["seed_selection"], {
                    "prepass_seconds": None, "depth_candidates": count,
                    "uncovered_candidates": None, "reserved_uncovered": 0,
                })
                model.decide(True)

    def test_coverage_prepass_uses_retained_origins_without_mutating_model(self):
        model = OnlineGaussianModel(self.device)
        model.reset(1)
        for frame_id in range(10):
            _, metrics = model.train_candidate(make_view(frame_id))
            self.assertNotIn("seed_selection", metrics)
            model.decide(True)
        view = make_view(10, dense=True)
        with self.assertRaisesRegex(ValueError, "seed policy"):
            model.train_candidate(view, seed_policy="other")
        parameters = model.parameters
        optimizer = model.optimizer
        origins = model.origin_ids.clone()
        prior = {name: tensor.detach().clone() for name, tensor in parameters.items()}
        keep = origins != 0

        with mock.patch("online_model._render_view") as render:
            with self.assertRaisesRegex(ValueError, "training view"):
                model.train_candidate(dict(view, split="validation"), seed_policy="coverage")
            render.assert_not_called()

        def prepass(render_parameters, render_view):
            self.assertIs(render_view, view)
            self.assertFalse(torch.is_grad_enabled())
            for name, value in render_parameters.items():
                torch.testing.assert_close(value, prior[name][keep])
            alpha = torch.ones((6, 8), device=self.device)
            alpha[0, :3] = torch.tensor([0., .049, .05], device=self.device)
            return None, alpha, None

        with mock.patch("online_model._MAX_PER_VIEW", 4):
            with mock.patch("online_model._render_view", side_effect=RuntimeError("prepass failed")):
                with self.assertRaisesRegex(RuntimeError, "prepass failed"):
                    model.train_candidate(view, seed_policy="coverage")
            self.assertEqual(model.frame_ids, list(range(10)))
            self.assertFalse(model.candidate_pending)
            self.assertIs(model.parameters, parameters)
            self.assertIs(model.optimizer, optimizer)
            torch.testing.assert_close(model.origin_ids, origins)
            for name, value in model.parameters.items():
                torch.testing.assert_close(value, prior[name])

            def render(parameters, observation):
                if observation is view:
                    return prepass(parameters, observation)
                return _render_view(parameters, observation)

            with mock.patch("online_model._render_view", side_effect=render) as mocked:
                _, metrics = model.train_candidate(view, seed_policy="coverage")
            self.assertEqual(mocked.call_count, 22)
        selection = metrics["seed_selection"]
        self.assertGreaterEqual(selection["prepass_seconds"], 0)
        self.assertEqual(selection["depth_candidates"], 48)
        self.assertEqual(selection["uncovered_candidates"], 2)
        self.assertEqual(selection["reserved_uncovered"], 2)
        self.assertEqual(model.frame_ids, list(range(1, 11)))
        self.assertEqual(model.origin_ids.tolist(), list(range(1, 10)) + [10] * 4)
        model.decide(True)

    def test_no_depth_black_white_and_discard_clear_everything(self):
        model = OnlineGaussianModel(self.device)
        model.reset(1)
        empty = make_view(0)
        empty["depth_mask"][:] = False
        empty["depth_m"][:] = 0
        with self.assertRaises(ValueError):
            model.train_candidate(empty)
        self.assertEqual(model.frame_ids, [])
        view = make_view(0)
        view["rgb"][:] = 0
        view["rgb"][3, 4] = 1
        _, metrics = model.train_candidate(view)
        self.assertTrue(np.isfinite(metrics["rgb_l1"]))
        for tensor in model.parameters.values():
            self.assertTrue(torch.isfinite(tensor).all().item())
        model.decide(False)
        self.assertEqual(model.frame_ids, [])
        self.assertIsNone(model.optimizer)
        self.assertIsNone(model.parameters)
        model.reset(2)
        view["generation"] = "2"
        _, metrics = model.train_candidate(view)
        self.assertEqual(metrics["frame_ids"], [0])
        model.decide(True)

    def test_missing_gpu_does_not_fall_back_to_cpu(self):
        environment = dict(os.environ, CUDA_VISIBLE_DEVICES="")
        result = subprocess.run(
            [sys.executable, "-c",
             "from online_model import checked_cuda_device; checked_cuda_device()"],
            env=environment, capture_output=True, text=True, timeout=10,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("CUDA is required", result.stderr)

    def test_resource_and_extension_failures_are_explicit(self):
        with mock.patch("torch.cuda.mem_get_info", return_value=(3 * 2**30, 8 * 2**30)):
            with self.assertRaisesRegex(RuntimeError, "4 GiB"):
                checked_cuda_device()
        with mock.patch("online_model.importlib.import_module", side_effect=ImportError):
            with self.assertRaisesRegex(RuntimeError, "prebuilt gsplat.csrc"):
                checked_cuda_device()

    def test_requires_decision_and_rejects_out_of_order_views(self):
        model = OnlineGaussianModel(self.device)
        model.reset(1)
        model.train_candidate(make_view(0))
        with self.assertRaises(RuntimeError):
            model.train_candidate(make_view(1))
        self.assertEqual(model.frame_ids, [0])
        model.decide(True)
        with self.assertRaises(ValueError):
            model.train_candidate(make_view(0))
        stale = make_view(1)
        stale["generation"] = "0"
        with self.assertRaises(ValueError):
            model.train_candidate(stale)
        self.assertEqual(model.frame_ids, [0])




class CoverageRatioGpuTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from dataclasses import replace
        from worker_config import DEFAULT_WORKER_OPTIONS
        if not torch.cuda.is_available():
            if os.environ.get('LIVO_REQUIRE_GPU') == '1':
                raise RuntimeError('CUDA is required')
            raise unittest.SkipTest('CUDA is not available')
        # These two tests retain at most eight Gaussians on 8x6 images.
        # Use an explicit test budget without changing the production default.
        cls.options = replace(DEFAULT_WORKER_OPTIONS, min_free_gpu_memory_mib=1024,
                              gpu_memory_fraction=.2)
        cls.device = checked_cuda_device(cls.options)
        torch.set_num_threads(1)

    def test_zero_ratio_uses_uniform_training_without_prepass(self):
        from dataclasses import replace
        from worker_config import DEFAULT_WORKER_OPTIONS
        options = replace(self.options, coverage_uncovered_ratio=0.)
        models = []
        schedules = []
        for policy in ('uniform', 'coverage'):
            model = OnlineGaussianModel(self.device, options=options)
            model.reset(1)
            actual = []
            with mock.patch('online_model._MAX_PER_VIEW', 4):
                for key in (0, 1):
                    with mock.patch('online_model._render_view', wraps=_render_view) as render:
                        _, metrics = model.train_candidate(make_view(key, dense=True), seed_policy=policy)
                    self.assertEqual(render.call_count, 21)
                    actual.append(metrics['training_frame_ids'])
                    if policy == 'coverage':
                        self.assertIsNone(metrics['seed_selection']['prepass_seconds'])
                        self.assertEqual(metrics['seed_selection']['reserved_uncovered'], 0)
                    model.decide(True)
            models.append(model)
            schedules.append(actual)
        self.assertEqual(schedules[0], schedules[1])
        torch.testing.assert_close(models[0].origin_ids, models[1].origin_ids, rtol=0, atol=0)
        for name in models[0].parameters:
            torch.testing.assert_close(models[0].parameters[name], models[1].parameters[name], rtol=1e-6, atol=1e-7)

    def test_quarter_ratio_reaches_prepass_and_actual_initialization(self):
        from dataclasses import replace
        from worker_config import DEFAULT_WORKER_OPTIONS
        options = replace(self.options, coverage_uncovered_ratio=.25,
                          learning_rates=replace(DEFAULT_WORKER_OPTIONS.learning_rates, means=0.))
        model = OnlineGaussianModel(self.device, options=options)
        model.reset(1)
        model.train_candidate(make_view(0))
        model.decide(True)
        view = make_view(1, dense=True)
        def render(parameters, observation):
            if observation is view:
                alpha = torch.ones((6, 8), device=self.device)
                alpha[0, :2] = 0
                return None, alpha, None
            return _render_view(parameters, observation)
        with mock.patch('online_model._MAX_PER_VIEW', 4), mock.patch('online_model._render_view', side_effect=render):
            _, metrics = model.train_candidate(view, seed_policy='coverage')
        self.assertEqual(metrics['seed_selection']['reserved_uncovered'], 1)
        actual = model.parameters['means'][model.origin_ids == 1].detach().cpu().numpy()
        np.testing.assert_array_equal(actual, [[-4.5, 4.5, 1], [-4.5, 3.5, 1], [-2.5, 4.5, 1], [-.5, 4.5, 1]])


if __name__ == "__main__":
    unittest.main()
