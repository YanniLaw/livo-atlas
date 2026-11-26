import copy
import unittest

import numpy as np
import torch

from train import (create_parameters, initialize_gaussians, masked_losses,
                   training_schedule)


def make_view(keyframe_id, split, width=8, height=6):
    return {
        "keyframe_id": keyframe_id,
        "split": split,
        "rgb": np.full((height, width, 3), 0.25, dtype=np.float32),
        "image_mask": np.ones((height, width), dtype=bool),
        "depth_mask": np.zeros((height, width), dtype=bool),
        "depth_m": np.zeros((height, width), dtype=np.float32),
        "K": np.array([[4, 0, 3.5], [0, 4, 2.5], [0, 0, 1]], dtype=float),
        "camera_from_parent": np.array(
            [[0, -1, 0, 1], [1, 0, 0, 2], [0, 0, 1, 3], [0, 0, 0, 1]],
            dtype=float,
        ),
    }


class InitializationTest(unittest.TestCase):
    def test_configured_coverage_ratio_preserves_total_budget_and_exact_seeds(self):
        view = make_view(0, 'train', width=4, height=3)
        valid = [0, 1, 2, 3, 4, 6, 7, 10, 11]
        view['depth_mask'].flat[valid] = True
        view['depth_m'].flat[valid] = 4
        view['rgb'][:, :, 0] = np.arange(12).reshape(3, 4) / 12
        holes = np.zeros((3, 4), dtype=bool)
        holes.flat[[6, 7, 11]] = True
        for ratio, pixels in ((0, [0, 2, 4, 7]), (.25, [0, 2, 6, 7]),
                              (.5, [0, 3, 6, 7]), (1, [0, 6, 7, 11])):
            with self.subTest(ratio=ratio):
                result = initialize_gaussians([view], 4, uncovered_mask=holes, uncovered_ratio=ratio)
                np.testing.assert_array_equal(result['colors'][:, 0], view['rgb'][:, :, 0].flat[pixels])
                self.assertEqual(result['means'].shape, (4, 3))
        holes[:] = False
        holes.flat[[1, 2, 3, 4, 6, 7]] = True
        result = initialize_gaussians([view], 4, uncovered_mask=holes, uncovered_ratio=1)
        np.testing.assert_array_equal(result['colors'][:, 0], view['rgb'][:, :, 0].flat[[1, 2, 4, 6]])

    def test_initializer_rejects_invalid_ratios_and_zero_matches_uniform(self):
        view = make_view(0, 'train')
        view['depth_mask'][:] = True
        view['depth_m'][:] = 4
        uniform = initialize_gaussians([view], 4)
        zero = initialize_gaussians([view], 4, uncovered_mask=np.ones((6, 8), bool), uncovered_ratio=0)
        for name in uniform:
            np.testing.assert_array_equal(zero[name], uniform[name])
        for ratio in (True, -.01, 1.01, float('nan'), float('inf'), '.5'):
            with self.subTest(ratio=ratio), self.assertRaises(ValueError):
                initialize_gaussians([view], 4, uncovered_ratio=ratio)

    def test_backprojection_and_validation_isolation(self):
        first = make_view(2, "train")
        first["depth_mask"][3, 4] = True
        first["depth_m"][3, 4] = 4
        first["rgb"][3, 4] = [0.1, 0.2, 0.3]
        second = copy.deepcopy(first)
        second["keyframe_id"] = 0
        held_out = copy.deepcopy(first)
        held_out["keyframe_id"] = 1
        held_out["split"] = "validation"
        views = [first, held_out, second]
        result = initialize_gaussians(views, 1000)
        np.testing.assert_allclose(result["means"], [[-1.5, 0.5, 1]] * 2)
        np.testing.assert_allclose(result["colors"], [[0.1, 0.2, 0.3]] * 2)
        np.testing.assert_array_equal(result["quats"], [[1, 0, 0, 0]] * 2)
        np.testing.assert_allclose(result["scales"], np.full((2, 3), 0.03))
        np.testing.assert_allclose(result["opacities"], [0.5, 0.5])
        schedule = training_schedule(views, 5)
        self.assertEqual(schedule, [0, 2, 0, 2, 0])
        held_out["rgb"][:] = 1
        held_out["depth_m"][:] = 99
        changed = initialize_gaussians(views, 1000)
        for name in result:
            np.testing.assert_array_equal(result[name], changed[name])
        self.assertEqual(schedule, training_schedule(views, 5))

    def test_deterministic_row_major_sampling_above_and_below_limit(self):
        view = make_view(0, "train", width=1201, height=1)
        view["depth_mask"][:] = True
        view["depth_m"][:] = 4
        view["rgb"][0, :, 0] = np.arange(1201, dtype=np.float32) / 1200
        result = initialize_gaussians([view], 1000)
        self.assertEqual(result["means"].shape, (1000, 3))
        indices = np.arange(1000) * 1201 // 1000
        np.testing.assert_array_equal(result["colors"][:, 0],
                                      view["rgb"][0, indices, 0])
        np.testing.assert_array_equal(
            result["means"], initialize_gaussians([view], 1000)["means"]
        )
        small = initialize_gaussians([view], 1500)
        self.assertEqual(small["means"].shape, (1201, 3))
        np.testing.assert_array_equal(small["colors"][:, 0], view["rgb"][0, :, 0])

    def test_coverage_reserves_seeds_and_backprojects_in_row_major_order(self):
        view = make_view(0, "train", width=4, height=3)
        valid = [0, 1, 2, 3, 4, 6, 7, 10, 11]
        view["depth_mask"].flat[valid + [5]] = True
        view["image_mask"].flat[5] = False
        view["depth_m"].flat[valid + [5]] = 4
        view["rgb"][:, :, 0] = np.arange(12).reshape(3, 4) / 12
        uncovered = np.zeros((3, 4), dtype=bool)
        uniform = initialize_gaussians([view], 4)
        np.testing.assert_array_equal(uniform["colors"][:, 0],
                                      view["rgb"][:, :, 0].flat[[0, 2, 4, 7]])
        all_seeds = initialize_gaussians([view], len(valid))
        for holes, selected in (([5, 6, 7, 11], [0, 3, 6, 7]),
                                ([5, 6], [0, 2, 6, 7])):
            with self.subTest(holes=holes):
                uncovered[:] = False
                uncovered.flat[holes] = True
                result = initialize_gaussians([view], 4, uncovered_mask=uncovered)
                for name in uniform:
                    np.testing.assert_array_equal(
                        result[name], all_seeds[name][[valid.index(i) for i in selected]]
                    )
        for limit, holes in ((4, []), (4, range(12)), (9, [6]), (1, [6])):
            with self.subTest(limit=limit, holes=holes):
                uncovered[:] = False
                uncovered.flat[list(holes)] = True
                result = initialize_gaussians([view], limit, uncovered_mask=uncovered)
                for name, value in initialize_gaussians([view], limit).items():
                    np.testing.assert_array_equal(result[name], value)

    def test_coverage_requires_a_boolean_mask_for_one_training_view(self):
        view = make_view(0, "train")
        other = make_view(1, "train")
        mask = np.zeros_like(view["depth_mask"])
        for views, bad_mask in (([view, other], mask), ([view], mask.T),
                                ([view], mask.astype(np.uint8))):
            with self.subTest(length=len(views), shape=bad_mask.shape, dtype=bad_mask.dtype):
                with self.assertRaisesRegex(ValueError, "uncovered_mask"):
                    initialize_gaussians(views, 10, uncovered_mask=bad_mask)

    def test_black_and_white_parameters_are_finite(self):
        view = make_view(0, "train", width=2, height=1)
        view["depth_mask"][:] = True
        view["depth_m"][:] = 1
        view["rgb"][0, 0] = 0
        view["rgb"][0, 1] = 1
        values = initialize_gaussians([view], 1000)
        parameters = create_parameters(values, torch.device("cpu"))
        for parameter in parameters.values():
            self.assertTrue(torch.isfinite(parameter).all().item())


class MaskedLossTest(unittest.TestCase):
    def test_rgb_and_alpha_selected_depth(self):
        view = make_view(0, "train", width=2, height=1)
        view["rgb"][:] = [[[0.1, 0.2, 0.3], [0.8, 0.9, 1.0]]]
        view["depth_mask"][:] = True
        view["depth_m"][:] = [[2, 4]]
        rgb = torch.tensor([[[0.4, 0.2, 0.0], [1.0, 0.9, 0.8]]],
                           requires_grad=True)
        depth = torch.tensor([[3.0, 2.0]])
        alpha = torch.tensor([[0.05, 0.049]])
        rgb_loss, depth_loss, count = masked_losses(rgb, depth, alpha, view)
        self.assertAlmostEqual(rgb_loss.item(), 1.0 / 6.0, places=6)
        self.assertAlmostEqual(depth_loss.item(), 1.0, places=6)
        self.assertEqual(count, 1)
        (rgb_loss + depth_loss).backward()
        self.assertTrue(torch.isfinite(rgb.grad).all().item())
        self.assertGreater(rgb.grad.abs().sum().item(), 0)

    def test_empty_depth_or_alpha_still_allows_rgb_gradient(self):
        view = make_view(0, "train", width=2, height=1)
        rgb = torch.zeros((1, 2, 3), requires_grad=True)
        depth = torch.ones((1, 2))
        alpha = torch.zeros((1, 2))
        for has_depth in (False, True):
            view["depth_mask"][:] = has_depth
            view["depth_m"][:] = 1 if has_depth else 0
            rgb_loss, depth_loss, count = masked_losses(rgb, depth, alpha, view)
            self.assertEqual(count, 0)
            self.assertEqual(depth_loss.item(), 0)
            self.assertTrue(torch.isfinite(rgb_loss).item())
            rgb_loss.backward()
            self.assertGreater(rgb.grad.abs().sum().item(), 0)
            rgb.grad = None

    def test_empty_image_mask_is_rejected(self):
        view = make_view(0, "train", width=2, height=1)
        view["image_mask"][:] = False
        with self.assertRaises(ValueError):
            masked_losses(torch.zeros((1, 2, 3)), torch.ones((1, 2)),
                          torch.ones((1, 2)), view)


if __name__ == "__main__":
    unittest.main()
