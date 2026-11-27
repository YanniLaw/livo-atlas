from dataclasses import replace
import os
from pathlib import Path
import tempfile
import unittest

import numpy as np
import yaml

from seed_sampling import stratified_pixels
from train import _seed_pixels, initialize_gaussians
from worker_config import DEFAULT_CONFIG, DEFAULT_WORKER_OPTIONS, load_worker_config


class StratifiedSamplingTest(unittest.TestCase):
    def test_dense_grid_selects_one_valid_center_quantile_per_cell(self):
        np.testing.assert_array_equal(stratified_pixels(np.arange(16), 4, (4, 4)), [4, 6, 12, 14])
        np.testing.assert_array_equal(stratified_pixels(np.array([0, 1, 2, 4, 5, 8]), 4, (4, 4)), [1, 2, 5, 8])

    def test_sparse_skinny_saturated_and_empty_are_exact_without_duplicates(self):
        rng = np.random.default_rng(17)
        for shape in ((1, 127), (127, 1), (17, 19), (512, 640)):
            valid = np.sort(rng.choice(np.prod(shape), size=min(3217, np.prod(shape)), replace=False))
            for n in (0, 1, min(37, len(valid)), len(valid)):
                with self.subTest(shape=shape, n=n):
                    points = stratified_pixels(valid, n, shape)
                    self.assertEqual(len(points), n)
                    self.assertEqual(len(np.unique(points)), n)
                    self.assertTrue(np.isin(points, valid).all())
                    np.testing.assert_array_equal(points, np.sort(points))
                    np.testing.assert_array_equal(points, stratified_pixels(valid, n, shape))
        np.testing.assert_array_equal(stratified_pixels(np.array([], dtype='i8'), 0, (4, 4)), [])

    def test_invalid_budget_shape_or_pixel_list_is_rejected(self):
        for pixels, count, shape in ((np.arange(4), True, (2, 2)), (np.arange(4), 5, (2, 2)),
                (np.array([1, 1]), 1, (2, 2)), (np.array([2, 1]), 1, (2, 2)),
                (np.array([-1]), 1, (2, 2)), (np.array([4]), 1, (2, 2)),
                (np.array([1.]), 1, (2, 2)), (np.arange(4), 1, (0, 4)),
                (np.arange(4), -1, (2, 2)), (np.arange(4), 1, (1001, 1000))):
            with self.subTest(pixels=pixels, count=count, shape=shape), self.assertRaises(ValueError):
                stratified_pixels(pixels, count, shape)

    def test_unsigned_pixels_do_not_overflow_grid_coordinates(self):
        pixels = np.arange(256, dtype='u1')
        np.testing.assert_array_equal(stratified_pixels(pixels, 100, (100, 100)),
                                      stratified_pixels(pixels.astype('i8'), 100, (100, 100)))

    def test_default_flat_and_stratified_replacement_positions(self):
        mask = np.ones((4, 4), bool)
        holes = np.zeros_like(mask)
        holes[0] = True
        np.testing.assert_array_equal(_seed_pixels(mask, 4, None, .5, 'replace'), [0, 4, 8, 12])
        np.testing.assert_array_equal(_seed_pixels(mask, 4, None, .5, 'replace', 'stratified'), [4, 6, 12, 14])
        np.testing.assert_array_equal(_seed_pixels(mask, 4, holes, .5, 'replace', 'stratified'), [1, 3, 4, 12])
        for sampling in ('resample', 'replace'):
            for ratio in (0., .25, 1.):
                for condition in (None, np.zeros_like(mask), mask.copy(), holes):
                    selected = _seed_pixels(mask, 4, condition, ratio, sampling, 'stratified')
                    self.assertEqual(len(selected), 4)
                    self.assertEqual(len(np.unique(selected)), 4)

    def test_yaml_defaults_and_invalid_distributions(self):
        self.assertEqual(load_worker_config(DEFAULT_CONFIG).options.seed_distribution, 'flat')
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)/'config.yaml'
            for value in ('flat', 'stratified', 'other', True, None, 1):
                path.write_text(yaml.safe_dump(dict(gaussian=dict(worker=dict(seed_distribution=value)))))
                if value in ('flat', 'stratified'):
                    self.assertEqual(load_worker_config(path).options.seed_distribution, value)
                else:
                    with self.assertRaisesRegex(ValueError, 'seed_distribution'): load_worker_config(path)
            path.write_text('gaussian: {}\n')
            self.assertEqual(load_worker_config(path).options.seed_distribution, 'flat')

    def test_initializer_rejects_invalid_distribution_and_preserves_flat(self):
        from test_online_model import make_view
        view = make_view(0, width=80, height=80, dense=True)
        flat = initialize_gaussians([view], 1000)
        explicit = initialize_gaussians([view], 1000, seed_distribution='flat')
        for name in flat: np.testing.assert_array_equal(flat[name], explicit[name])
        with self.assertRaisesRegex(ValueError, 'seed_distribution'):
            initialize_gaussians([view], 1000, seed_distribution=True)


class StratifiedGpuTest(unittest.TestCase):
    def test_real_prepass_evidence_and_insertion_use_same_stratified_base(self):
        import torch
        from online_model import OnlineGaussianModel, checked_cuda_device
        from test_online_model import make_view
        if not torch.cuda.is_available():
            if os.environ.get('LIVO_REQUIRE_GPU') == '1': raise RuntimeError('CUDA required')
            self.skipTest('CUDA unavailable')
        options = replace(DEFAULT_WORKER_OPTIONS, seed_distribution='stratified', coverage_sampling='replace',
            coverage_uncovered_ratio=.25, min_free_gpu_memory_mib=128, gpu_memory_fraction=.2,
            learning_rates=replace(DEFAULT_WORKER_OPTIONS.learning_rates, means=0., log_scales=0., color_logits=0., opacity_logits=0.))
        device = checked_cuda_device(options)
        torch.set_num_threads(1)
        model = OnlineGaussianModel(device, options=options)
        model.reset(1)
        for key in range(11):
            view = make_view(key, width=80, height=80, dense=True)
            if key == 10:
                mask, evidence = model._coverage_prepass(view)
                base = _seed_pixels(view['image_mask'] & view['depth_mask'], 1000, None, .25, 'replace', 'stratified')
                self.assertEqual(evidence['uniform_uncovered'], int(mask.ravel()[base].sum()))
                expected = initialize_gaussians([view], 1000, uncovered_mask=mask,
                    uncovered_ratio=.25, coverage_sampling='replace', seed_distribution='stratified')
            _, metrics = model.train_candidate(view, seed_policy='coverage')
            model.decide(True)
        actual = model.parameters['means'].detach().cpu().numpy()[model.origin_ids.cpu().numpy() == 10]
        np.testing.assert_array_equal(actual, expected['means'])
        self.assertEqual(model.frame_ids, list(range(1, 11)))
        self.assertEqual(len(model.origin_ids), 10000)
        self.assertEqual(metrics['steps'], 20)
        self.assertEqual(metrics['seed_selection']['uniform_uncovered'], evidence['uniform_uncovered'])


if __name__ == '__main__':
    unittest.main()
