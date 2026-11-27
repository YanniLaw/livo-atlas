import copy
import math
import unittest

import numpy as np

from texture_metrics import (aggregate_partitions, aggregate_stats, coverage_partition,
                             gradient_masks, texture_metrics)
from evaluation_metrics import _ssim, measure
from test_evaluation_metrics import reference_ssim


class TextureMetricsTest(unittest.TestCase):
    def test_four_states_reconstruct_rgb_error_and_weighted_contributions(self):
        reference = np.zeros((1, 4, 3))
        a = dict(rgb=np.repeat(np.array([[[0.], [.5], [.5], [1.]]]), 3, axis=2),
                 alpha=np.array([[1., 0, 1, 0]]))
        b = dict(rgb=np.repeat(np.array([[[.5], [0.], [1.], [.5]]]), 3, axis=2),
                 alpha=np.array([[1., 1, 0, 0]]))
        result = coverage_partition(reference, a, b, np.ones((1, 4), bool))
        expected = dict(common_covered=(0., .75), gained=(.75, 0.), lost=(.75, 3.), common_holes=(3., .75))
        self.assertEqual(result['image_pixels'], 4)
        self.assertEqual(result['residuals']['baseline']['pixels'], 0)
        self.assertEqual(result['residuals']['candidate']['rgb_squared_sum'], 0.)
        for state, (old, new) in expected.items():
            row = result['states'][state]
            self.assertEqual(row['baseline']['pixels'], 1)
            self.assertEqual(row['baseline']['rgb_squared_sum'], old)
            self.assertEqual(row['candidate']['rgb_squared_sum'], new)
            self.assertAlmostEqual(row['mse_delta_contribution'], (new-old)/12)
        self.assertEqual(result['global']['baseline']['rgb_squared_sum'], 4.5)
        self.assertEqual(result['global']['candidate']['rgb_squared_sum'], 4.5)
        self.assertAlmostEqual(sum(r['mse_delta_contribution'] for r in result['states'].values()), 0)
        pooled = aggregate_partitions([result, result])
        self.assertEqual(pooled['image_pixels'], 8)
        self.assertEqual(pooled['states']['lost']['baseline']['pixels'], 2)
        self.assertEqual(pooled['global']['candidate']['rgb_squared_sum'], 9.)
        self.assertEqual(pooled['states']['lost']['mse_delta_contribution'], .1875)

    def test_reference_gradient_detects_alternating_texture_and_masked_neighbors(self):
        yy, xx = np.indices((13, 13))
        rgb = np.repeat(((xx+yy) % 2)[..., None], 3, axis=2).astype('f8')
        image = np.ones((13, 13), bool)
        g = gradient_masks(rgb, image)
        self.assertEqual(int(g['strong'].sum()), 121)
        self.assertEqual(int(g['weak'].sum()), 0)
        self.assertEqual(int(g['unknown'].sum()), 48)
        image[6, 6] = False
        g = gradient_masks(rgb, image)
        self.assertEqual(int(g['strong'].sum()), 116)
        self.assertEqual(int(g['unknown'].sum()), 52)
        g = gradient_masks(np.full_like(rgb, .5), image)
        self.assertEqual(int(g['weak'].sum()), 116)
        self.assertEqual(int(g['strong'].sum()), 0)

    def test_ssim_uses_complete_region_windows_and_center_gradient_groups(self):
        ref = np.full((13, 13, 3), .5)
        rgb = np.full_like(ref, .25)
        image = np.ones((13, 13), bool)
        common = image.copy()
        common[0, 0] = False  # Removes one of the nine 11x11 windows.
        strong = np.zeros_like(image)
        strong[5, 5] = True
        gradient = dict(strong=strong, weak=~strong, unknown=np.zeros_like(image))
        result = texture_metrics(ref, rgb, dict(image=image, common=common), gradient)
        score = (2*.5*.25+.01**2)/(.5**2+.25**2+.01**2)
        self.assertEqual(result['image']['all']['ssim_windows'], 9)
        self.assertEqual(result['image']['strong']['ssim_windows'], 1)
        self.assertEqual(result['common']['all']['ssim_windows'], 8)
        self.assertEqual(result['common']['strong']['ssim_windows'], 0)
        self.assertIsNone(result['common']['strong']['ssim_rgb'])
        self.assertAlmostEqual(result['image']['all']['ssim_rgb'], score, places=12)
        total, count, reason = _ssim(ref, rgb, image)
        self.assertEqual(result['image']['all']['ssim_sum'], total)
        self.assertEqual(result['image']['all']['ssim_windows'], count)

    def test_texture_scores_match_legacy_full_image_metrics(self):
        rng = np.random.default_rng(7)
        ref = rng.random((17, 19, 3)).astype('f4')
        rgb = np.clip(ref*.7+.1, 0, 1)
        image = np.ones((17, 19), bool)
        image[0, 0] = False
        query = dict(rgb=ref, depth_m=np.zeros(image.shape, dtype='f4'), image_mask=image, depth_mask=np.zeros_like(image))
        rendered = dict(rgb=rgb, alpha=np.ones(image.shape, dtype='f4'), depth_m=np.zeros(image.shape, dtype='f4'))
        legacy = measure(rendered, query)
        actual = texture_metrics(ref, rgb, dict(image=image, common=image), gradient_masks(ref, image))['image']['all']
        naive, windows = reference_ssim(ref, rgb, image)
        self.assertEqual(actual['ssim_windows'], windows)
        self.assertAlmostEqual(actual['ssim_rgb'], naive, places=10)
        for key, old_key in (('pixels', 'image_pixels'), ('rgb_absolute_sum', 'rgb_absolute_error_sum'),
                             ('rgb_squared_sum', 'rgb_squared_error_sum'), ('ssim_sum', 'ssim_sum'), ('ssim_windows', 'ssim_windows')):
            self.assertEqual(actual[key], legacy[old_key])

    def test_empty_perfect_small_images_and_pooled_denominators(self):
        ref = np.zeros((1, 1, 3))
        image = np.ones((1, 1), bool)
        result = texture_metrics(ref, ref, dict(image=image, common=~image), gradient_masks(ref, image))
        self.assertTrue(result['image']['all']['rgb_perfect'])
        self.assertIsNone(result['image']['all']['rgb_psnr_db'])
        self.assertEqual(result['image']['all']['rgb_reason'], 'perfect_rgb')
        self.assertEqual(result['image']['all']['ssim_reason'], 'image_smaller_than_window')
        self.assertIsNone(result['common']['all']['rgb_mse'])
        self.assertFalse(result['common']['all']['rgb_perfect'])
        self.assertEqual(result['common']['all']['rgb_reason'], 'no_selected_pixels')
        bright = texture_metrics(ref, np.ones_like(ref), dict(image=image, common=image), gradient_masks(ref, image))['image']['all']
        pooled = aggregate_stats([result['image']['all'], bright, bright])
        self.assertEqual(pooled['pixels'], 3)
        self.assertAlmostEqual(pooled['rgb_mse'], 2/3)
        self.assertAlmostEqual(pooled['rgb_psnr_db'], -10*math.log10(2/3))

    def test_invalid_arrays_and_overlapping_gradient_groups_are_rejected(self):
        ref = np.zeros((13, 13, 3));image = np.ones((13, 13), bool)
        gradients = gradient_masks(ref, image)
        invalid = copy.deepcopy(gradients);invalid['strong'][3, 3] = True
        with self.assertRaises(ValueError): texture_metrics(ref, ref, dict(image=image, common=image), invalid)
        for rgb, mask in ((ref*np.nan, image), (ref, image.astype('i4')), (ref+2, image), (ref, image[:-1])):
            with self.subTest(rgb=rgb.shape, mask=mask.shape), self.assertRaises(ValueError): gradient_masks(rgb, mask)
        with self.assertRaises(ValueError):
            coverage_partition(ref, dict(rgb=ref, alpha=np.full(image.shape, np.nan)),
                               dict(rgb=ref, alpha=np.ones(image.shape)), image)


if __name__ == '__main__':
    unittest.main()
