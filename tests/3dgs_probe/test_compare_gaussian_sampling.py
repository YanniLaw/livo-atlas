import copy
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from compare_gaussian_sampling import compare, depth_comparison
from sampling_fixtures import make_sampling_pair


class SamplingComparisonTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.baseline, self.candidate = make_sampling_pair(self.root)

    def test_shared_depth_and_diagnostics_use_exact_float_samples(self):
        result = compare(self.baseline, self.candidate, self.root / 'comparison')
        self.assertEqual(result['status'], 'succeeded')
        self.assertEqual(result['joint']['query_count'], 1)
        self.assertEqual(result['joint']['common_depth']['pixels'], 1)
        self.assertEqual(result['joint']['common_depth']['baseline_l1_m'], 1.)
        self.assertEqual(result['joint']['common_depth']['candidate_l1_m'], .5)
        self.assertTrue((self.root / 'comparison/views/9.png').is_file())
        self.assertEqual(result['training_calls'], 10)
        self.assertEqual(result['training_steps'], 200)

    def test_changed_controls_schedule_plan_and_scores_are_rejected(self):
        path = self.candidate / 'report.json'
        original = json.loads(path.read_text())
        changes = [lambda r: r.update(random_seed=2),
                   lambda r: r.update(training_options={'depth_loss_weight': .2}),
                   lambda r: r.update(final_poses_sha256='f'*64),
                   lambda r: r['submaps'][0]['training'][0].update(training_frame_ids=[1]*20),
                   lambda r: r['queries'][1].update(selected_ids=[2, 4]),
                   lambda r: r['queries'][0]['single'].update(rgb_mse=.9),
                   lambda r: r.update(training_calls=11)]
        for index, change in enumerate(changes):
            report = copy.deepcopy(original)
            change(report)
            path.write_text(json.dumps(report))
            with self.subTest(change=index), self.assertRaises(ValueError):
                compare(self.baseline, self.candidate, self.root / f'bad-{index}')
            self.assertFalse((self.root / f'bad-{index}/report.json').exists())

    def test_changed_reference_and_sample_bytes_are_rejected(self):
        path = self.candidate / 'views/9/samples.npz'
        with path.open('ab') as stream:
            stream.write(b'tampered')
        with self.assertRaisesRegex(ValueError, 'artifact'):
            compare(self.baseline, self.candidate, self.root / 'bad')

    def test_different_references_rejected_even_with_updated_artifact_hash(self):
        from prepare_holdout_split import file_sha
        path = self.candidate / 'views/9/samples.npz'
        with np.load(path, allow_pickle=False) as source:
            values = {key: source[key].copy() for key in source.files}
        values['reference_rgb'][0, 0] = .9
        np.savez(path, **values)
        report_path = self.candidate / 'report.json'
        report = json.loads(report_path.read_text())
        report['artifacts']['views/9/samples.npz'] = dict(bytes=path.stat().st_size, sha256=file_sha(path))
        report_path.write_text(json.dumps(report))
        with self.assertRaises(ValueError):
            compare(self.baseline, self.candidate, self.root / 'bad')

    def test_no_shared_coverage_has_explicit_null_depth(self):
        mask = np.ones((1, 2), dtype=bool)
        a = dict(alpha=np.array([[1., 0.]], 'f4'), depth_m=np.ones((1, 2), 'f4'))
        b = dict(alpha=np.array([[0., 1.]], 'f4'), depth_m=np.ones((1, 2), 'f4'))
        result = depth_comparison(a, b, np.ones((1, 2), 'f4'), mask)
        self.assertEqual(result['pixels'], 0)
        self.assertIsNone(result['baseline_l1_m'])
        self.assertEqual(result['reason'], 'no_common_covered_depth')

    def test_matching_but_unfrozen_camera_is_rejected(self):
        for root in (self.baseline, self.candidate):
            path = root / 'report.json'
            report = json.loads(path.read_text())
            report['queries'][0]['query']['K'][0][0] += 1
            path.write_text(json.dumps(report))
        with self.assertRaisesRegex(ValueError, 'camera'):
            compare(self.baseline, self.candidate, self.root / 'bad-camera')

    def test_matching_reference_tampering_cannot_bypass_frozen_split(self):
        from evaluation_metrics import measure
        from prepare_holdout_split import file_sha
        for root in (self.baseline, self.candidate):
            path = root / 'views/9/samples.npz'
            with np.load(path, allow_pickle=False) as source:
                values = {key: source[key].copy() for key in source.files}
            values['reference_rgb'][0, 0] = .9
            np.savez(path, **values)
            query = dict(rgb=values['reference_rgb'], depth_m=values['reference_depth_m'],
                         image_mask=values['image_mask'], depth_mask=values['depth_mask'])
            report_path = root / 'report.json'
            report = json.loads(report_path.read_text())
            for mode in ('single', 'joint'):
                report['queries'][1][mode] = measure(
                    {field: values[mode+'_'+field] for field in ('rgb', 'alpha', 'depth_m')}, query)
            report['artifacts']['views/9/samples.npz'] = dict(bytes=path.stat().st_size, sha256=file_sha(path))
            report_path.write_text(json.dumps(report))
        with self.assertRaisesRegex(ValueError, 'frozen'):
            compare(self.baseline, self.candidate, self.root / 'bad-reference')


if __name__ == '__main__':
    unittest.main()
