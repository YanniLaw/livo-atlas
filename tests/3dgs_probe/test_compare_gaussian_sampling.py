import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

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

    def test_matching_extrinsics_wrong_owner_and_tampered_poses_are_rejected(self):
        for root in (self.baseline, self.candidate):
            path = root / 'report.json'
            report = json.loads(path.read_text())
            report['queries'][0]['query']['camera_from_parent'][3] = 1234567
            path.write_text(json.dumps(report))
        with self.assertRaisesRegex(ValueError, 'camera|pose'):
            compare(self.baseline, self.candidate, self.root / 'bad-extrinsics')

    def test_owner_identity_is_bound_to_split(self):
        path = self.candidate / 'report.json'
        report = json.loads(path.read_text())
        report['queries'][0]['owner_submap_id'] = 999
        path.write_text(json.dumps(report))
        with self.assertRaisesRegex(ValueError, 'owner'):
            compare(self.baseline, self.candidate, self.root / 'bad-owner')

    def test_final_pose_payload_must_match_original_digest(self):
        path = self.candidate / 'poses.f64le'
        poses = np.frombuffer(path.read_bytes(), dtype='<f8').reshape(12, 4, 4).copy()
        poses[0, 0, 3] += 1
        path.write_bytes(poses.tobytes())
        with self.assertRaisesRegex(ValueError, 'pose'):
            compare(self.baseline, self.candidate, self.root / 'bad-pose')

    def test_missing_png_and_missing_receipt_cannot_be_certified(self):
        path = self.candidate / 'views/1/reference.png'
        path.unlink()
        with self.assertRaises((ValueError, OSError)):
            compare(self.baseline, self.candidate, self.root / 'missing-png')
        report_path = self.candidate / 'report.json'
        report = json.loads(report_path.read_text())
        del report['artifacts']['views/1/reference.png']
        report_path.write_text(json.dumps(report))
        with self.assertRaisesRegex(ValueError, 'artifact'):
            compare(self.baseline, self.candidate, self.root / 'missing-receipt')

    def test_partial_report_write_never_creates_published_report(self):
        import compare_gaussian_sampling as comparison
        write = comparison._exclusive_write
        def partial(path, payload):
            write(path, payload[:64])
            raise OSError('partial write')
        output = self.root / 'partial'
        with mock.patch('compare_gaussian_sampling._exclusive_write', side_effect=partial):
            with self.assertRaisesRegex(OSError, 'partial write'):
                compare(self.baseline, self.candidate, output)
        self.assertFalse((output / 'report.json').exists())

    def test_disk_reserve_drop_during_diagnostics_prevents_publication(self):
        import compare_gaussian_sampling as comparison
        diagnostic = comparison._diagnostic
        free = 4 * 1024**3
        def draw(*args):
            nonlocal free
            diagnostic(*args)
            free = 0
        output = self.root / 'disk-drop'
        with (mock.patch('compare_gaussian_sampling._diagnostic', side_effect=draw),
              mock.patch('compare_gaussian_sampling.shutil.disk_usage', side_effect=lambda _: mock.Mock(free=free))):
            with self.assertRaisesRegex(ValueError, 'reserve'):
                compare(self.baseline, self.candidate, output)
        self.assertFalse((output / 'report.json').exists())

    def test_run_summary_must_match_reconstructed_metrics(self):
        path = self.candidate / 'report.json'
        report = json.loads(path.read_text())
        report['summary']['joint']['rgb_psnr_db'] = 999
        path.write_text(json.dumps(report))
        with self.assertRaisesRegex(ValueError, 'summary'):
            compare(self.baseline, self.candidate, self.root / 'bad-summary')

    def configure_ratio(self, root, ratio, extra=None):
        import hashlib
        import yaml
        path = root / 'report.json'
        report = json.loads(path.read_text())
        report['training_options']['coverage_uncovered_ratio'] = ratio
        document = {'gaussian': {'worker': {'coverage_uncovered_ratio': ratio}}}
        if extra:
            document['gaussian'].update(extra)
        raw = yaml.safe_dump(document).encode()
        (root / 'config.yaml').write_bytes(raw)
        report['config_sha256'] = hashlib.sha256(raw).hexdigest()
        path.write_text(json.dumps(report))

    def test_explicit_ratio_comparison_allows_only_ratio_change(self):
        self.configure_ratio(self.baseline, 0.)
        self.configure_ratio(self.candidate, .25)
        with self.assertRaises(ValueError):
            compare(self.baseline, self.candidate, self.root / 'strict')
        result = compare(self.baseline, self.candidate, self.root / 'ratio', allow_coverage_ratio_change=True)
        self.assertEqual(result['comparison_scope'], 'coverage_uncovered_ratio')
        self.assertEqual(result['coverage_uncovered_ratios'], {'baseline': 0., 'candidate': .25})

    def test_explicit_ratio_mode_rejects_other_configuration_and_changed_archive(self):
        self.configure_ratio(self.baseline, 0.)
        self.configure_ratio(self.candidate, .25, {'evaluation': {'max_output_mib': 999}})
        with self.assertRaisesRegex(ValueError, 'config'):
            compare(self.baseline, self.candidate, self.root / 'other-key', allow_coverage_ratio_change=True)
        self.configure_ratio(self.candidate, .25)
        with (self.candidate / 'config.yaml').open('ab') as stream:
            stream.write(b'changed: true\n')
        with self.assertRaisesRegex(ValueError, 'config'):
            compare(self.baseline, self.candidate, self.root / 'changed-file', allow_coverage_ratio_change=True)

    def test_explicit_ratio_mode_rejects_mismatched_optimizer_and_missing_archive(self):
        self.configure_ratio(self.baseline, 0.)
        self.configure_ratio(self.candidate, .25)
        path = self.candidate / 'report.json'
        report = json.loads(path.read_text())
        report['training_options']['learning_rates']['means'] = .1
        path.write_text(json.dumps(report))
        with self.assertRaisesRegex(ValueError, 'config|option'):
            compare(self.baseline, self.candidate, self.root / 'other-rate', allow_coverage_ratio_change=True)
        (self.candidate / 'config.yaml').unlink()
        with self.assertRaises((ValueError, OSError)):
            compare(self.baseline, self.candidate, self.root / 'missing-config', allow_coverage_ratio_change=True)

    def test_ratio_mode_rejects_real_optimizer_change_and_accepts_zero_coverage_baseline(self):
        import hashlib
        import yaml
        self.configure_ratio(self.baseline, 0.)
        self.configure_ratio(self.candidate, .5)
        path = self.baseline / 'report.json'
        report = json.loads(path.read_text())
        report['seed_policy'] = 'coverage'
        for group in report['submaps']:
            for operation in group['training']:
                operation['seed_selection'] = dict(depth_candidates=1, prepass_seconds=None,
                    uncovered_candidates=None, reserved_uncovered=0)
        path.write_text(json.dumps(report))
        result = compare(self.baseline, self.candidate, self.root / 'zero-coverage', allow_coverage_ratio_change=True)
        self.assertEqual(result['coverage_uncovered_ratios']['baseline'], 0.)
        path = self.candidate / 'report.json'
        report = json.loads(path.read_text())
        report['training_options']['learning_rates']['means'] = .1
        config = yaml.safe_load((self.candidate / 'config.yaml').read_text())
        config['gaussian']['worker']['learning_rates'] = {'means': .1}
        raw = yaml.safe_dump(config).encode()
        (self.candidate / 'config.yaml').write_bytes(raw)
        report['config_sha256'] = hashlib.sha256(raw).hexdigest()
        path.write_text(json.dumps(report))
        with self.assertRaisesRegex(ValueError, 'Other config|training options'):
            compare(self.baseline, self.candidate, self.root / 'different-optimizer', allow_coverage_ratio_change=True)

    def test_ratio_mode_rejects_other_yaml_scalar_type_changes(self):
        self.configure_ratio(self.baseline, 0., {'publish_preview': False})
        self.configure_ratio(self.candidate, .25, {'publish_preview': 0})
        with self.assertRaisesRegex(ValueError, 'config'):
            compare(self.baseline, self.candidate, self.root / 'changed-type', allow_coverage_ratio_change=True)

    def test_effective_options_preserve_exact_scalar_types(self):
        import hashlib
        import yaml
        for root, ratio in ((self.baseline, 0.), (self.candidate, .25)):
            self.configure_ratio(root, ratio)
            path = root / 'report.json'
            report = json.loads(path.read_text())
            config = yaml.safe_load((root / 'config.yaml').read_text())
            config['gaussian']['worker']['depth_loss_weight'] = 0.
            raw = yaml.safe_dump(config).encode()
            (root / 'config.yaml').write_bytes(raw)
            report['config_sha256'] = hashlib.sha256(raw).hexdigest()
            report['training_options']['depth_loss_weight'] = 0. if root == self.baseline else False
            path.write_text(json.dumps(report))
        with self.assertRaisesRegex(ValueError, 'config|options'):
            compare(self.baseline, self.candidate, self.root / 'option-type', allow_coverage_ratio_change=True)

    def test_modern_report_cannot_omit_archive_declaration(self):
        path = self.candidate / 'report.json'
        report = json.loads(path.read_text())
        del report['config_file']
        path.write_text(json.dumps(report))
        (self.candidate / 'config.yaml').write_text('invalid and unrelated YAML')
        with self.assertRaisesRegex(ValueError, 'config'):
            compare(self.baseline, self.candidate, self.root / 'omitted-declaration')

    def test_genuine_legacy_report_without_archive_remains_supported(self):
        for root in (self.baseline, self.candidate):
            path = root / 'report.json'
            report = json.loads(path.read_text())
            del report['config_file']
            del report['training_options']['coverage_uncovered_ratio']
            report['training_options'].pop('coverage_sampling', None)
            report['training_options'].pop('seed_distribution', None)
            path.write_text(json.dumps(report))
            (root / 'config.yaml').unlink()
        result = compare(self.baseline, self.candidate, self.root / 'legacy')
        self.assertEqual(result['status'], 'succeeded')

    def configure_sampling(self, root, sampling):
        import hashlib
        import yaml
        path = root / 'report.json'
        report = json.loads(path.read_text())
        report['seed_policy'] = 'coverage'
        report['training_options']['coverage_sampling'] = sampling
        config = yaml.safe_load((root / 'config.yaml').read_text())
        config['gaussian']['worker']['coverage_sampling'] = sampling
        raw = yaml.safe_dump(config).encode()
        (root / 'config.yaml').write_bytes(raw)
        report['config_sha256'] = hashlib.sha256(raw).hexdigest()
        for group in report['submaps']:
            for operation in group['training']:
                evidence = dict(depth_candidates=1, prepass_seconds=None,
                                uncovered_candidates=None, reserved_uncovered=0)
                if sampling == 'replace':
                    evidence.update(uniform_uncovered=None, replaced_seeds=0)
                operation['seed_selection'] = evidence
        path.write_text(json.dumps(report))

    def test_explicit_sampling_comparison_allows_only_method_change(self):
        self.configure_ratio(self.baseline, .25)
        self.configure_ratio(self.candidate, .25)
        self.configure_sampling(self.baseline, 'resample')
        self.configure_sampling(self.candidate, 'replace')
        with self.assertRaises(ValueError):
            compare(self.baseline, self.candidate, self.root / 'strict-method')
        result = compare(self.baseline, self.candidate, self.root / 'method', allow_coverage_sampling_change=True)
        self.assertEqual(result['comparison_scope'], 'coverage_sampling')
        self.assertEqual(result['coverage_sampling'], {'baseline': 'resample', 'candidate': 'replace'})
        with self.assertRaises(ValueError):
            compare(self.baseline, self.candidate, self.root / 'two-flags',
                    allow_coverage_sampling_change=True, allow_coverage_ratio_change=True)

    def test_sampling_mode_rejects_changed_ratio_other_options_and_yaml_type(self):
        self.configure_ratio(self.baseline, .25, {'publish_preview': False})
        self.configure_ratio(self.candidate, .5, {'publish_preview': False})
        self.configure_sampling(self.baseline, 'resample')
        self.configure_sampling(self.candidate, 'replace')
        with self.assertRaises(ValueError):
            compare(self.baseline, self.candidate, self.root / 'wrong-ratio', allow_coverage_sampling_change=True)
        self.configure_ratio(self.candidate, .25, {'publish_preview': 0})
        self.configure_sampling(self.candidate, 'replace')
        with self.assertRaises(ValueError):
            compare(self.baseline, self.candidate, self.root / 'wrong-type', allow_coverage_sampling_change=True)
        self.configure_ratio(self.candidate, .25, {'publish_preview': False})
        self.configure_sampling(self.candidate, 'replace')
        path = self.candidate / 'report.json'
        report = json.loads(path.read_text())
        report['training_options']['depth_loss_weight'] = .9
        path.write_text(json.dumps(report))
        with self.assertRaises(ValueError):
            compare(self.baseline, self.candidate, self.root / 'wrong-option', allow_coverage_sampling_change=True)

    def test_ratio_mode_cannot_also_change_sampling_method(self):
        self.configure_ratio(self.baseline, 0.)
        self.configure_ratio(self.candidate, .25)
        self.configure_sampling(self.baseline, 'resample')
        self.configure_sampling(self.candidate, 'replace')
        with self.assertRaisesRegex(ValueError, 'config|options'):
            compare(self.baseline, self.candidate, self.root / 'ratio-and-method', allow_coverage_ratio_change=True)

    def test_old_archived_options_supply_only_missing_resample_default(self):
        for root in (self.baseline, self.candidate):
            path = root / 'report.json'
            report = json.loads(path.read_text())
            report['training_options'].pop('coverage_sampling')
            path.write_text(json.dumps(report))
        result = compare(self.baseline, self.candidate, self.root / 'old-archive')
        self.assertEqual(result['status'], 'succeeded')

    def test_default_comparison_accepts_mixed_historical_resample_option(self):
        path = self.baseline / 'report.json'
        report = json.loads(path.read_text())
        del report['training_options']['coverage_sampling']
        path.write_text(json.dumps(report))
        result = compare(self.baseline, self.candidate, self.root / 'mixed-archive')
        self.assertEqual(result['status'], 'succeeded')

    def test_method_controls_and_seed_types_remain_exact(self):
        self.configure_ratio(self.baseline, .25)
        self.configure_ratio(self.candidate, .25)
        self.configure_sampling(self.baseline, 'resample')
        self.configure_sampling(self.candidate, 'replace')
        path = self.candidate / 'report.json'
        original = json.loads(path.read_text())
        for key, value in (('random_seed', 0.), ('training_calls', 10.)):
            report = copy.deepcopy(original)
            report[key] = value
            path.write_text(json.dumps(report))
            with self.subTest(key=key), self.assertRaises(ValueError):
                compare(self.baseline, self.candidate, self.root / key, allow_coverage_sampling_change=True)
        path.write_text(json.dumps(original))
        for root in (self.baseline, self.candidate):
            p = root / 'report.json'
            report = json.loads(p.read_text())
            report['random_seed'] = 0.
            p.write_text(json.dumps(report))
        with self.assertRaises(ValueError):
            compare(self.baseline, self.candidate, self.root / 'both-float-seeds', allow_coverage_sampling_change=True)

    def test_coverage_retention_uses_one_shared_reference(self):
        import compare_gaussian_sampling as comparison
        reference = np.ones((1, 4), dtype='f4')
        resample = np.array([[1., 0., 0., 1.]], dtype='f4')
        replacement = np.array([[0., 1., 1., 1.]], dtype='f4')
        mask = np.ones((1, 4), bool)
        self.assertEqual(comparison.coverage_transition(reference, resample, mask),
                         dict(newly_covered=0, newly_uncovered=2, common_holes=0))
        self.assertEqual(comparison.coverage_transition(reference, replacement, mask),
                         dict(newly_covered=0, newly_uncovered=1, common_holes=0))
        self.assertEqual(comparison.coverage_transition(resample, replacement, mask),
                         dict(newly_covered=2, newly_uncovered=1, common_holes=0))
        with self.assertRaises(ValueError):
            comparison.coverage_transition(reference, replacement.T, mask)


    def configure_learning_rates(self, root, **rates):
        import hashlib
        import yaml
        path = root / 'report.json'
        report = json.loads(path.read_text())
        report['training_options']['learning_rates'].update(rates)
        config = yaml.safe_load((root / 'config.yaml').read_text())
        config['gaussian'].setdefault('worker', {}).setdefault('learning_rates', {}).update(rates)
        raw = yaml.safe_dump(config).encode()
        (root / 'config.yaml').write_bytes(raw)
        report['config_sha256'] = hashlib.sha256(raw).hexdigest()
        path.write_text(json.dumps(report))

    def freeze_pair(self):
        for root in (self.baseline, self.candidate):
            self.configure_ratio(root, .25)
            self.configure_sampling(root, 'replace')

    def archive_config(self, root, config):
        import hashlib
        import yaml
        raw = yaml.safe_dump(config).encode()
        (root / 'config.yaml').write_bytes(raw)
        path = root / 'report.json'
        report = json.loads(path.read_text())
        report['config_sha256'] = hashlib.sha256(raw).hexdigest()
        path.write_text(json.dumps(report))

    def test_freeze_accepts_aliased_defaults_with_only_worker_override(self):
        import yaml
        self.freeze_pair()
        self.configure_learning_rates(self.baseline, means=.0005)
        self.configure_learning_rates(self.candidate, means=0.)
        for root in (self.baseline, self.candidate):
            config = yaml.safe_load((root / 'config.yaml').read_text())
            config['default_rates'] = {'means': .0005}
            if root == self.baseline:
                config['gaussian']['worker']['learning_rates'] = config['default_rates']
            self.archive_config(root, config)
        result = compare(self.baseline, self.candidate, self.root / 'aliased-defaults',
                         frozen_parameter='means')
        self.assertEqual(result['status'], 'succeeded')

    def test_freeze_rejects_rate_change_aliased_into_another_section(self):
        import yaml
        self.freeze_pair()
        self.configure_learning_rates(self.baseline, means=.0005)
        self.configure_learning_rates(self.candidate, means=0.)
        for root in (self.baseline, self.candidate):
            config = yaml.safe_load((root / 'config.yaml').read_text())
            config['other_rates'] = config['gaussian']['worker']['learning_rates']
            self.archive_config(root, config)
        with self.assertRaisesRegex(ValueError, 'config'):
            compare(self.baseline, self.candidate, self.root / 'aliased-other',
                    frozen_parameter='means')

    def test_freeze_rejects_mapping_key_type_changes(self):
        import yaml
        self.freeze_pair()
        self.configure_learning_rates(self.candidate, means=0.)
        for index, (old, new) in enumerate(((1, '1'), (False, 'false'), (None, 'null'))):
            for root, key in ((self.baseline, old), (self.candidate, new)):
                config = yaml.safe_load((root / 'config.yaml').read_text())
                config['sensor_profiles'] = {key: 'camera'}
                self.archive_config(root, config)
            with self.subTest(key=old), self.assertRaisesRegex(ValueError, 'config'):
                compare(self.baseline, self.candidate, self.root / f'key-type-{index}',
                        frozen_parameter='means')

    def test_freeze_rejects_unrelated_sampling_default_spelling_change(self):
        import yaml
        self.freeze_pair()
        for root in (self.baseline, self.candidate):
            self.configure_sampling(root, 'resample')
        self.configure_learning_rates(self.candidate, means=0.)
        config = yaml.safe_load((self.baseline / 'config.yaml').read_text())
        del config['gaussian']['worker']['coverage_sampling']
        self.archive_config(self.baseline, config)
        with self.assertRaisesRegex(ValueError, 'config'):
            compare(self.baseline, self.candidate, self.root / 'sampling-default',
                    frozen_parameter='means')

    def test_frozen_parameter_comparison_accepts_only_selected_zero_rate(self):
        self.freeze_pair()
        for parameter in ('means', 'log_scales'):
            self.configure_learning_rates(self.candidate, means=.0005, log_scales=.002)
            self.configure_learning_rates(self.baseline, means=.0005, log_scales=.002)
            self.configure_learning_rates(self.candidate, **{parameter: 0.})
            with self.subTest(parameter=parameter):
                result = compare(self.baseline, self.candidate, self.root/parameter, frozen_parameter=parameter)
                self.assertEqual(result['comparison_scope'], 'learning_rates.'+parameter)
                self.assertEqual(result['learning_rates']['candidate'], 0.)
                self.assertGreater(result['learning_rates']['baseline'], 0.)
                self.assertEqual(result['learning_rates']['parameter'], parameter)
                # The comparison must not pop fields from either archived run.
                self.assertIn(parameter, json.loads((self.candidate/'report.json').read_text())['training_options']['learning_rates'])

    def test_freeze_comparison_normalizes_only_targeted_default(self):
        self.freeze_pair()
        self.configure_learning_rates(self.candidate, means=0.)
        result = compare(self.baseline, self.candidate, self.root/'implicit-default', frozen_parameter='means')
        self.assertEqual(result['learning_rates']['baseline'], .0005)

    def test_freeze_rejects_nonzero_reverse_two_rates_and_other_controls(self):
        self.freeze_pair()
        for index, (baseline, candidate) in enumerate(((.0005,.0001), (0.,0.), (0.,.0005))):
            self.configure_learning_rates(self.baseline, means=baseline)
            self.configure_learning_rates(self.candidate, means=candidate)
            with self.subTest(direction=index), self.assertRaises(ValueError):
                compare(self.baseline,self.candidate,self.root/f'direction-{index}',frozen_parameter='means')
        self.configure_learning_rates(self.baseline, means=.0005)
        self.configure_learning_rates(self.candidate, means=0., log_scales=0.)
        with self.assertRaises(ValueError):
            compare(self.baseline,self.candidate,self.root/'two-rates',frozen_parameter='means')
        self.configure_learning_rates(self.candidate, log_scales=.002)
        self.configure_learning_rates(self.baseline, log_scales=.002)
        self.configure_ratio(self.candidate, .5)
        self.configure_sampling(self.candidate, 'replace')
        self.configure_learning_rates(self.candidate, means=0., log_scales=.002)
        with self.assertRaises(ValueError):
            compare(self.baseline,self.candidate,self.root/'ratio',frozen_parameter='means')
        self.configure_ratio(self.candidate, .25)
        self.configure_sampling(self.candidate, 'resample')
        self.configure_learning_rates(self.candidate, means=0., log_scales=.002)
        with self.assertRaises(ValueError):
            compare(self.baseline,self.candidate,self.root/'sampling',frozen_parameter='means')
        self.configure_sampling(self.candidate, 'replace')
        for root, value in ((self.baseline,False),(self.candidate,0)):
            import yaml, hashlib
            path=root/'config.yaml';config=yaml.safe_load(path.read_text())
            config['gaussian']['publish_preview']=value
            raw=yaml.safe_dump(config).encode();path.write_bytes(raw)
            report=json.loads((root/'report.json').read_text());report['config_sha256']=hashlib.sha256(raw).hexdigest()
            (root/'report.json').write_text(json.dumps(report))
        with self.assertRaises(ValueError):
            compare(self.baseline,self.candidate,self.root/'yaml-types',frozen_parameter='means')

    def test_freeze_mode_names_and_modes_are_exclusive(self):
        import compare_gaussian_sampling as comparison
        self.freeze_pair()
        self.configure_learning_rates(self.candidate, means=0.)
        for name in ('colors','',False):
            with self.subTest(name=name), self.assertRaises(ValueError):
                compare(self.baseline,self.candidate,self.root/'name',frozen_parameter=name)
        for flag in ('allow_coverage_ratio_change','allow_coverage_sampling_change'):
            with self.subTest(flag=flag), self.assertRaises(ValueError):
                compare(self.baseline,self.candidate,self.root/flag,frozen_parameter='means',**{flag:True})
        with self.assertRaises(ValueError):
            compare(self.baseline,self.candidate,self.root/'strict-freeze')
        with mock.patch('sys.stderr'):
            with self.assertRaises(SystemExit) as result:
                comparison.main(['--baseline',str(self.baseline),'--candidate',str(self.candidate),
                    '--output',str(self.root/'cli'),'--frozen-parameter','means','--allow-coverage-ratio-change'])
        self.assertEqual(result.exception.code,2)

    def reduced_rate_pair(self, parameter='means'):
        self.freeze_pair()
        for root in (self.baseline, self.candidate):
            self.configure_learning_rates(root, means=.0005, log_scales=.002)
        self.configure_learning_rates(self.candidate, **{parameter: .00025 if parameter == 'means' else .001})

    def test_reduced_rate_accepts_only_one_positive_decrease_and_cli(self):
        import compare_gaussian_sampling as comparison
        for parameter, old, new in (('means', .0005, .00025), ('log_scales', .002, .001)):
            self.reduced_rate_pair(parameter)
            result = compare(self.baseline, self.candidate, self.root / parameter,
                             reduced_learning_rate=parameter)
            self.assertEqual(result['comparison_scope'], 'learning_rates.' + parameter)
            self.assertEqual(result['learning_rates'],
                             dict(parameter=parameter, baseline=old, candidate=new))
            self.assertEqual(json.loads((self.candidate / 'report.json').read_text())
                             ['training_options']['learning_rates'][parameter], new)
        self.assertEqual(comparison.main([
            '--baseline', str(self.baseline), '--candidate', str(self.candidate),
            '--output', str(self.root / 'cli-positive'), '--reduced-learning-rate', 'log_scales']), 0)

    def test_reduced_rate_rejects_zero_equal_increased_and_reverse_rates(self):
        for index, (old, new) in enumerate(((.0005, 0.), (.0005, .0005), (.0005, .001), (0., .00025))):
            self.reduced_rate_pair()
            self.configure_learning_rates(self.baseline, means=old)
            self.configure_learning_rates(self.candidate, means=new)
            with self.subTest(old=old, new=new), self.assertRaises(ValueError):
                compare(self.baseline, self.candidate, self.root / f'invalid-rate-{index}',
                        reduced_learning_rate='means')

    def test_reduced_rate_names_modes_and_old_mode_limits(self):
        import compare_gaussian_sampling as comparison
        self.reduced_rate_pair()
        for index, name in enumerate(('', False, 'color_logits')):
            with self.subTest(name=name), self.assertRaises(ValueError):
                compare(self.baseline, self.candidate, self.root / f'invalid-name-{index}',
                        reduced_learning_rate=name)
        for index, extra in enumerate((dict(frozen_parameter='means'),
                                       dict(allow_coverage_ratio_change=True),
                                       dict(allow_coverage_sampling_change=True))):
            with self.subTest(extra=extra), self.assertRaises(ValueError):
                compare(self.baseline, self.candidate, self.root / f'modes-{index}',
                        reduced_learning_rate='means', **extra)
        for index, extra in enumerate(({}, dict(frozen_parameter='means'))):
            with self.subTest(old_mode=extra), self.assertRaises(ValueError):
                compare(self.baseline, self.candidate, self.root / f'old-modes-{index}', **extra)
        with mock.patch('sys.stderr'), self.assertRaises(SystemExit) as result:
            comparison.main(['--baseline', str(self.baseline), '--candidate', str(self.candidate),
                             '--output', str(self.root / 'cli-conflict'), '--reduced-learning-rate', 'means',
                             '--frozen-parameter', 'means'])
        self.assertEqual(result.exception.code, 2)

    def test_reduced_rate_accepts_targeted_default_and_detaches_yaml_alias(self):
        import yaml
        self.freeze_pair()
        self.configure_learning_rates(self.candidate, means=.00025)
        result = compare(self.baseline, self.candidate, self.root / 'implicit-positive',
                         reduced_learning_rate='means')
        self.assertEqual(result['learning_rates']['baseline'], .0005)
        self.reduced_rate_pair()
        for root in (self.baseline, self.candidate):
            config = yaml.safe_load((root / 'config.yaml').read_text())
            config['default_rates'] = {'means': .0005, 'log_scales': .002}
            if root == self.baseline:
                config['gaussian']['worker']['learning_rates'] = config['default_rates']
            self.archive_config(root, config)
        self.assertEqual(compare(self.baseline, self.candidate, self.root / 'positive-alias',
                                 reduced_learning_rate='means')['status'], 'succeeded')

    def test_reduced_rate_rejects_other_controls_and_aliased_paths(self):
        import yaml
        for case in ('second-rate', 'ratio', 'sampling', 'alias', 'key-type', 'value-type', 'default-spelling'):
            self.reduced_rate_pair()
            if case == 'second-rate':
                self.configure_learning_rates(self.candidate, log_scales=.001)
            elif case == 'ratio':
                self.configure_ratio(self.candidate, .5)
                self.configure_sampling(self.candidate, 'replace')
                self.configure_learning_rates(self.candidate, means=.00025, log_scales=.002)
            elif case == 'sampling':
                self.configure_sampling(self.candidate, 'resample')
            else:
                if case == 'default-spelling':
                    for root in (self.baseline, self.candidate):
                        self.configure_sampling(root, 'resample')
                for root in (self.baseline, self.candidate):
                    config = yaml.safe_load((root / 'config.yaml').read_text())
                    if case == 'alias':
                        config['other_rates'] = config['gaussian']['worker']['learning_rates']
                    elif case == 'key-type':
                        config['profiles'] = {1 if root == self.baseline else '1': 'camera'}
                    elif case == 'value-type':
                        config['enabled'] = False if root == self.baseline else 0
                    elif root == self.baseline:
                        del config['gaussian']['worker']['coverage_sampling']
                    self.archive_config(root, config)
            with self.subTest(case=case), self.assertRaisesRegex(ValueError, 'config|options'):
                compare(self.baseline, self.candidate, self.root / case, reduced_learning_rate='means')

    def test_reduced_rate_accepts_identical_mixed_yaml_keys(self):
        import yaml
        self.reduced_rate_pair()
        for root in (self.baseline, self.candidate):
            config = yaml.safe_load((root / 'config.yaml').read_text())
            config['profiles'] = {1: 'camera', 'fallback': 'camera'}
            self.archive_config(root, config)
        self.assertEqual(compare(self.baseline, self.candidate, self.root / 'mixed-keys',
                                 reduced_learning_rate='means')['status'], 'succeeded')

    def test_reduced_rate_preserves_yaml_collection_value_types(self):
        import datetime
        import yaml
        self.reduced_rate_pair()
        for root in (self.baseline, self.candidate):
            config = yaml.safe_load((root / 'config.yaml').read_text())
            config['metadata'] = dict(day=datetime.date(2026, 11, 27), sensors={1, 'camera'}, ids=[1, False])
            self.archive_config(root, config)
        self.assertEqual(compare(self.baseline, self.candidate, self.root / 'yaml-values',
                                 reduced_learning_rate='means')['status'], 'succeeded')
        for index, value in enumerate((dict(day=datetime.date(2026, 11, 27), sensors={True, 'camera'}, ids=[1, False]),
                                       dict(day='2026-11-27', sensors={1, 'camera'}, ids=[1, False]),
                                       dict(day=datetime.date(2026, 11, 27), sensors={1, 'camera'}, ids=[True, 0]))):
            config = yaml.safe_load((self.candidate / 'config.yaml').read_text())
            config['metadata'] = value
            self.archive_config(self.candidate, config)
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'config'):
                compare(self.baseline, self.candidate, self.root / f'yaml-types-{index}',
                        reduced_learning_rate='means')



    def configure_distribution(self, root, value, *, implicit=False):
        import yaml
        path = root/'report.json'
        report = json.loads(path.read_text())
        document = yaml.safe_load((root/'config.yaml').read_text())
        if implicit:
            report['training_options'].pop('seed_distribution', None)
            document['gaussian']['worker'].pop('seed_distribution', None)
        else:
            report['training_options']['seed_distribution'] = value
            document['gaussian']['worker']['seed_distribution'] = value
        self.archive_config(root, document)
        updated = json.loads(path.read_text())
        updated['training_options'] = report['training_options']
        path.write_text(json.dumps(updated))

    def distribution_pair(self, policy='coverage', implicit=False):
        self.configure_ratio(self.baseline, .25)
        self.configure_ratio(self.candidate, .25)
        self.configure_sampling(self.baseline, 'replace')
        self.configure_sampling(self.candidate, 'replace')
        self.configure_distribution(self.baseline, 'flat', implicit=implicit)
        self.configure_distribution(self.candidate, 'stratified')
        if policy == 'uniform':
            for root in (self.baseline, self.candidate):
                path = root/'report.json'
                report = json.loads(path.read_text())
                report['seed_policy'] = 'uniform'
                for group in report['submaps']:
                    for operation in group['training']: operation.pop('seed_selection', None)
                path.write_text(json.dumps(report))

    def test_distribution_comparison_accepts_only_same_policy_flat_to_stratified(self):
        for policy, implicit in (('uniform', False), ('coverage', True)):
            self.distribution_pair(policy, implicit)
            report = compare(self.baseline, self.candidate, self.root/f'distribution-{policy}',
                             allow_seed_distribution_change=True)
            self.assertEqual(report['comparison_scope'], 'seed_distribution')
            self.assertEqual(report['seed_distributions'], dict(baseline='flat', candidate='stratified'))
            self.assertEqual(report['training_calls'], 10)

    def test_distribution_mode_rejects_other_knobs_types_aliases_and_missing_archives(self):
        import yaml
        for case in ('ratio', 'rate', 'policy', 'type', 'alias', 'archive', 'lying-legacy'):
            self.distribution_pair()
            if case == 'ratio':
                self.configure_ratio(self.candidate, .5)
                self.configure_sampling(self.candidate, 'replace')
                self.configure_distribution(self.candidate, 'stratified')
            if case == 'rate': self.configure_learning_rates(self.candidate, means=.00025)
            if case == 'policy':
                path = self.candidate/'report.json'
                report = json.loads(path.read_text())
                report['seed_policy'] = 'uniform'
                path.write_text(json.dumps(report))
            if case in ('type', 'alias'):
                for root in (self.baseline, self.candidate):
                    document = yaml.safe_load((root/'config.yaml').read_text())
                    if case == 'type': document['extra'] = True if root == self.baseline else 1
                    else: document['worker_alias'] = document['gaussian']['worker']
                    self.archive_config(root, document)
            if case == 'archive': (self.candidate/'config.yaml').unlink()
            if case == 'lying-legacy':
                path = self.candidate/'report.json'
                report = json.loads(path.read_text())
                report['training_options'].pop('seed_distribution')
                path.write_text(json.dumps(report))
            with self.subTest(case=case), self.assertRaises(FileNotFoundError if case == 'archive' else ValueError):
                compare(self.baseline, self.candidate, self.root/f'reject-distribution-{case}', allow_seed_distribution_change=True)

    def test_distribution_change_is_not_allowed_by_existing_modes_or_conflicting_flags(self):
        self.distribution_pair()
        for index, mode in enumerate(({}, dict(allow_coverage_ratio_change=True),
                dict(allow_coverage_sampling_change=True), dict(frozen_parameter='means'))):
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                compare(self.baseline, self.candidate, self.root/f'old-distribution-mode-{index}', **mode)
        for index, mode in enumerate((dict(allow_coverage_ratio_change=True), dict(allow_coverage_sampling_change=True),
                                     dict(frozen_parameter='means'), dict(reduced_learning_rate='means'))):
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                compare(self.baseline, self.candidate, self.root/f'conflict-distribution-{index}',
                        allow_seed_distribution_change=True, **mode)

    def test_distribution_cli_flag_and_direction_are_checked(self):
        import compare_gaussian_sampling as comparison
        self.distribution_pair()
        self.assertEqual(comparison.main(['--baseline', str(self.baseline), '--candidate', str(self.candidate),
            '--output', str(self.root/'distribution-cli'), '--allow-seed-distribution-change']), 0)
        for index, (old, new) in enumerate((('flat', 'flat'), ('stratified', 'flat'), ('stratified', 'stratified'))):
            self.configure_distribution(self.baseline, old)
            self.configure_distribution(self.candidate, new)
            with self.subTest(old=old, new=new), self.assertRaises(ValueError):
                compare(self.baseline, self.candidate, self.root/f'distribution-direction-{index}', allow_seed_distribution_change=True)

if __name__ == '__main__':
    unittest.main()
