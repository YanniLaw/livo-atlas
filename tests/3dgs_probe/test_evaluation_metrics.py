import copy
import math
import unittest

import numpy as np

from evaluation_metrics import aggregate_metrics, measure


def fixture(height=17, width=19):
    rng = np.random.default_rng(10)
    rgb = rng.random((height, width, 3)).astype('f4')
    reference = dict(rgb=rgb, image_mask=np.ones((height, width), bool),
                     depth_mask=np.ones((height, width), bool),
                     depth_m=np.ones((height, width), 'f4'))
    result = dict(rgb=rgb.copy(), alpha=np.ones((height, width), 'f4'),
                  depth_m=np.ones((height, width), 'f4'))
    return reference, result


def reference_ssim(a, b, mask):
    axis = np.arange(-5, 6, dtype='f8')
    kernel = np.exp(-(axis[:, None] ** 2 + axis[None, :] ** 2) / (2 * 1.5**2))
    kernel /= kernel.sum()
    values = []
    for y in range(5, a.shape[0] - 5):
        for x in range(5, a.shape[1] - 5):
            if not mask[y-5:y+6, x-5:x+6].all():
                continue
            channels = []
            for channel in range(3):
                u = a[y-5:y+6, x-5:x+6, channel].astype('f8')
                v = b[y-5:y+6, x-5:x+6, channel].astype('f8')
                mu, mv = (kernel * u).sum(), (kernel * v).sum()
                vu = (kernel * (u-mu)**2).sum()
                vv = (kernel * (v-mv)**2).sum()
                cov = (kernel * (u-mu) * (v-mv)).sum()
                channels.append(((2*mu*mv + .01**2) * (2*cov + .03**2)) /
                                ((mu**2+mv**2+.01**2) * (vu+vv+.03**2)))
            values.append(np.mean(channels))
    return float(np.mean(values)) if values else None, len(values)


class EvaluationMetricsTest(unittest.TestCase):
    def test_identical_and_changed_images(self):
        query, image = fixture()
        perfect = measure(image, query)
        self.assertTrue(perfect['rgb_perfect'])
        self.assertIsNone(perfect['rgb_psnr_db'])
        self.assertAlmostEqual(perfect['ssim_rgb'], 1)
        image['rgb'] *= .5
        changed = measure(image, query)
        expected, count = reference_ssim(query['rgb'], image['rgb'], query['image_mask'])
        self.assertAlmostEqual(changed['ssim_rgb'], expected, places=12)
        self.assertEqual(changed['ssim_windows'], count)
        self.assertLess(changed['ssim_rgb'], 1)
        self.assertAlmostEqual(changed['rgb_psnr_db'], -10*math.log10(changed['rgb_mse']))

    def test_empty_coverage_still_counts_rgb_and_reports_missing_depth(self):
        query, image = fixture()
        image['alpha'][:] = 0
        image['rgb'][:] = 0
        metrics = measure(image, query)
        self.assertGreater(metrics['rgb_mse'], 0)
        self.assertEqual(metrics['image_pixels'], 17 * 19)
        self.assertEqual(metrics['depth_pixels_available'], 17 * 19)
        self.assertEqual(metrics['depth_pixels_used'], 0)
        self.assertEqual(metrics['depth_reason'], 'no_covered_reference_depth')
        self.assertIsNone(metrics['depth_rmse_m'])

    def test_mask_requires_whole_ssim_window(self):
        query, image = fixture(25, 25)
        query['image_mask'][0, 0] = False
        query['image_mask'][12, 12] = False
        query['depth_mask'] &= query['image_mask']
        image['rgb'][0, 0] = 0
        expected, count = reference_ssim(query['rgb'], image['rgb'], query['image_mask'])
        actual = measure(image, query)
        self.assertEqual(actual['ssim_windows'], count)
        self.assertAlmostEqual(actual['ssim_rgb'], expected, places=12)

    def test_small_or_empty_images_have_explicit_null_reasons(self):
        query, image = fixture(6, 8)
        self.assertEqual(measure(image, query)['ssim_reason'], 'image_smaller_than_window')
        query['depth_mask'][:] = False
        query['depth_m'][:] = 0
        self.assertEqual(measure(image, query)['depth_reason'], 'no_reference_depth')
        query['image_mask'][:] = False
        empty = measure(image, query)
        self.assertIsNone(empty['rgb_mse'])
        self.assertIsNone(empty['alpha_coverage'])
        self.assertFalse(empty['rgb_perfect'])

    def test_summary_uses_sufficient_statistics_and_window_counts(self):
        rows = []
        for size, value in ((12, .25), (20, .75)):
            query, image = fixture(size, size)
            query['rgb'][:] = 0
            image['rgb'][:] = value
            image['depth_m'][:] = 1 + value
            rows.append(measure(image, query))
        summary = aggregate_metrics(rows)
        expected = sum(row['rgb_mse'] * row['image_pixels'] for row in rows) / (12**2 + 20**2)
        self.assertAlmostEqual(summary['rgb_psnr_db'], -10*math.log10(expected))
        self.assertAlmostEqual(summary['ssim_rgb'], sum(r['ssim_sum'] for r in rows) /
                               sum(r['ssim_windows'] for r in rows))
        self.assertIsNone(aggregate_metrics([]))

    def test_invalid_or_nonfinite_inputs_fail_without_mutating_arrays(self):
        query, image = fixture()
        saved = copy.deepcopy((query, image))
        measure(image, query)
        for before, after in zip(saved, (query, image)):
            for key in before:
                np.testing.assert_array_equal(before[key], after[key])
        for key in ('rgb', 'alpha', 'depth_m'):
            bad = copy.deepcopy(image)
            bad[key].flat[0] = np.nan
            with self.subTest(key=key), self.assertRaises(ValueError):
                measure(bad, query)
        with self.assertRaises(ValueError):
            measure(dict(image, rgb=image['rgb'][:-1]), query)


if __name__ == '__main__':
    unittest.main()
