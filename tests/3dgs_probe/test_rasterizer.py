import os
import sys
import unittest
from unittest import mock

import numpy as np
import torch

from rasterizer import render, to_gsplat_camera


def camera_fixture():
    intrinsic = np.array(
        [[4.0, 0.0, 3.5], [0.0, 4.0, 2.5], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    camera_from_parent = np.array(
        [[0.0, -1.0, 0.0, 1.0],
         [1.0, 0.0, 0.0, 2.0],
         [0.0, 0.0, 1.0, 3.0],
         [0.0, 0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    return intrinsic, camera_from_parent


class CameraAdapterTest(unittest.TestCase):
    def test_converts_only_principal_point_without_mutating_inputs(self):
        intrinsic, pose = camera_fixture()
        original_intrinsic = intrinsic.copy()
        original_pose = pose.copy()
        result_intrinsic, result_pose = to_gsplat_camera(intrinsic, pose)
        expected = original_intrinsic.copy()
        expected[0, 2] += 0.5
        expected[1, 2] += 0.5
        np.testing.assert_array_equal(result_intrinsic, expected)
        np.testing.assert_array_equal(result_pose, original_pose)
        np.testing.assert_array_equal(intrinsic, original_intrinsic)
        np.testing.assert_array_equal(pose, original_pose)

    def test_rejects_invalid_camera_and_pose(self):
        intrinsic, pose = camera_fixture()
        invalid = intrinsic.copy()
        invalid[0, 0] = 0
        with self.assertRaises(ValueError):
            to_gsplat_camera(invalid, pose)
        invalid = intrinsic.copy()
        invalid[0, 2] = np.nan
        with self.assertRaises(ValueError):
            to_gsplat_camera(invalid, pose)
        invalid_pose = pose.copy()
        invalid_pose[3, 0] = 1
        with self.assertRaises(ValueError):
            to_gsplat_camera(intrinsic, invalid_pose)


class GaussianRasterizationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not torch.cuda.is_available():
            if os.environ.get("LIVO_REQUIRE_GPU") == "1":
                raise RuntimeError("CUDA is required for this test")
            raise unittest.SkipTest("CUDA is not available")
        free, _ = torch.cuda.mem_get_info()
        if free < 4 * 2**30:
            raise RuntimeError("At least 4 GiB of free GPU memory is required")
        torch.cuda.set_per_process_memory_fraction(0.4)
        torch.cuda.reset_peak_memory_stats()
        torch.set_num_threads(1)

    @classmethod
    def tearDownClass(cls):
        torch.cuda.synchronize()
        print("GPU peak bytes: allocated={}, reserved={}".format(
            torch.cuda.max_memory_allocated(), torch.cuda.max_memory_reserved()
        ))

    def test_known_pose_pixel_rgb_depth_and_gradient(self):
        intrinsic, pose = camera_fixture()
        intrinsic, pose = to_gsplat_camera(intrinsic, pose)
        device = torch.device("cuda:0")
        means = torch.tensor([[-1.5, 0.5, 1.0]], device=device)
        quats = torch.tensor([[1.0, 0.0, 0.0, 0.0]], device=device)
        scales = torch.tensor([[0.1, 0.1, 0.1]], device=device)
        opacities = torch.tensor([0.8], device=device)
        colors = torch.tensor([[0.9, 0.2, 0.1]], device=device,
                              requires_grad=True)

        rgb, alpha, depth = render(
            means, quats, scales, opacities, colors, pose, intrinsic, 8, 6
        )
        torch.cuda.synchronize()
        self.assertEqual(tuple(rgb.shape), (6, 8, 3))
        self.assertEqual(tuple(alpha.shape), (6, 8))
        self.assertEqual(tuple(depth.shape), (6, 8))
        self.assertGreater(alpha[3, 4].item(), 0.05)
        self.assertGreater(alpha[3, 4].item(), alpha[3, 3].item() + 0.05)
        self.assertGreater(alpha[3, 4].item(), alpha[2, 4].item() + 0.05)
        self.assertAlmostEqual(depth[3, 4].item(), 4.0, places=4)
        torch.testing.assert_close(rgb[3, 4] / alpha[3, 4], colors[0],
                                   rtol=1e-5, atol=1e-5)

        optimizer = torch.optim.SGD([colors], lr=0.05)
        target = torch.tensor([0.1, 0.7, 0.6], device=device)
        before = colors.detach().clone()
        loss = (rgb[3, 4] - target).square().sum()
        loss.backward()
        self.assertTrue(torch.isfinite(colors.grad).all().item())
        self.assertGreater(colors.grad.abs().max().item(), 0.0)
        optimizer.step()
        torch.cuda.synchronize()
        self.assertFalse(torch.equal(colors.detach(), before))

    def test_missing_prebuilt_extension_does_not_enter_renderer(self):
        intrinsic, pose = to_gsplat_camera(*camera_fixture())
        device = torch.device("cuda:0")
        means = torch.tensor([[-1.5, 0.5, 1.0]], device=device)
        quats = torch.tensor([[1.0, 0.0, 0.0, 0.0]], device=device)
        scales = torch.tensor([[0.1, 0.1, 0.1]], device=device)
        opacities = torch.tensor([0.8], device=device)
        colors = torch.tensor([[0.9, 0.2, 0.1]], device=device)
        with mock.patch.dict(sys.modules, {"gsplat.csrc": None}), mock.patch(
                "rasterizer.rasterization",
                side_effect=AssertionError("Renderer must not be entered")):
            with self.assertRaisesRegex(RuntimeError, "prebuilt"):
                render(means, quats, scales, opacities, colors, pose, intrinsic,
                       8, 6)


if __name__ == "__main__":
    unittest.main()
