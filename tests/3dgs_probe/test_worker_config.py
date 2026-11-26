from pathlib import Path
import tempfile
import unittest

import yaml

from worker_config import DEFAULT_CONFIG, load_worker_config


class CoverageRatioConfigTest(unittest.TestCase):
    def test_sampling_method_default_and_validation(self):
        self.assertEqual(load_worker_config(DEFAULT_CONFIG).options.coverage_sampling, 'resample')
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'config.yaml'
            for value in ('resample', 'replace', 'other', True, None, 1):
                path.write_text(yaml.safe_dump({'gaussian': {'worker': {'coverage_sampling': value}}}))
                if value in ('resample', 'replace'):
                    self.assertEqual(load_worker_config(path).options.coverage_sampling, value)
                else:
                    with self.assertRaisesRegex(ValueError, 'coverage_sampling'):
                        load_worker_config(path)

    def load(self, value):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'config.yaml'
            path.write_text(yaml.safe_dump({'gaussian': {'worker': {'coverage_uncovered_ratio': value}}}))
            return load_worker_config(path)

    def test_default_and_valid_ratios(self):
        config = load_worker_config(DEFAULT_CONFIG)
        self.assertEqual(config.seed_policy, 'uniform')
        self.assertEqual(config.options.coverage_uncovered_ratio, .5)
        for ratio in (0, .25, .5, 1):
            with self.subTest(ratio=ratio):
                self.assertEqual(self.load(ratio).options.coverage_uncovered_ratio, ratio)

    def test_invalid_ratios_are_rejected(self):
        for ratio in (True, False, -.01, 1.01, '0.25', None, float('nan'), float('inf')):
            with self.subTest(ratio=ratio), self.assertRaisesRegex(ValueError, 'coverage_uncovered_ratio'):
                self.load(ratio)
