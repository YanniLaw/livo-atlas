import argparse
import copy
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import numpy as np
import yaml

from evaluate_gaussian_sampling import frozen_groups, admit_training_view, run
from heldout_fixtures import make_heldout_snapshot
from heldout_query import load_evaluation_snapshot


class SamplingPlanTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.directory, self.keyframes, self.split, self.views = make_heldout_snapshot(self.root, blocks=6)
        self.snapshot = load_evaluation_snapshot(self.directory, self.keyframes)

    def test_frozen_roles_include_unselected_heldout_without_training_them(self):
        groups = frozen_groups(self.snapshot)
        self.assertEqual(groups[0]['train_ids'], [0])
        self.assertEqual(groups[0]['held_out_ids'], [1])
        self.assertEqual(groups[1]['train_ids'], [2, 3])
        self.assertEqual(groups[1]['held_out_ids'], [])
        self.assertEqual(groups[4]['train_ids'], [8])
        self.assertEqual(groups[4]['held_out_ids'], [9])
        self.assertEqual(sum(len(g['train_ids']) for g in groups), 10)

    def test_train_admission_rejects_heldout_and_changed_observations(self):
        view = self.views[0]
        original = copy.deepcopy(view)
        accepted = admit_training_view(self.split, view)
        self.assertEqual(accepted['split'], 'train')
        np.testing.assert_array_equal(accepted['camera_from_body'].ravel(), self.split.frames[0]['camera_from_body'])
        np.testing.assert_allclose(accepted['camera_from_parent'] @ accepted['parent_from_body'],
                                   accepted['camera_from_body'], atol=1e-12)
        for key in ('rgb', 'camera_from_parent', 'parent_from_body'):
            if key in original:
                np.testing.assert_array_equal(view[key], original[key])
        with self.assertRaisesRegex(ValueError, 'role'):
            admit_training_view(self.split, self.views[1])
        changed = copy.deepcopy(view)
        changed['rgb'][0, 0, 0] += .01
        with self.assertRaisesRegex(ValueError, 'frozen split'):
            admit_training_view(self.split, changed)

    def arguments(self):
        config = self.root / 'config.yaml'
        config.write_text(yaml.safe_dump({'gaussian': {'joint_render_policy': 'temporal',
            'evaluation': {'max_output_mib': 16, 'min_free_disk_mib': 1024},
            'worker': {'learning_rates': {'means': .0002}, 'depth_loss_weight': .07}}}))
        return argparse.Namespace(map_directory=self.directory, keyframes=self.keyframes,
            prepare_executable=Path(os.environ['LIVO_PREPARE_BIN']), config=config,
            selector_executable=None, output=self.root / 'result', seed_policy='uniform', random_seed=0)

    def test_budget_failure_never_creates_success_report(self):
        args = self.arguments()
        doc = yaml.safe_load(args.config.read_text())
        doc['gaussian']['evaluation']['max_output_mib'] = 1
        args.config.write_text(yaml.safe_dump(doc))
        with mock.patch('evaluate_gaussian_sampling.shutil.disk_usage', return_value=mock.Mock(free=0)):
            with self.assertRaisesRegex(ValueError, 'reserve|budget'):
                run(args)
        self.assertFalse(args.output.exists())


class SamplingCudaTest(SamplingPlanTest):
    @classmethod
    def setUpClass(cls):
        import torch
        if not torch.cuda.is_available():
            if os.environ.get('LIVO_REQUIRE_GPU') == '1':
                raise RuntimeError('CUDA is required')
            raise unittest.SkipTest('CUDA is not available')

    def prepared(self, executable, keyframes, output, group, source_sha256):
        keys = group['train_ids'] + group['held_out_ids']
        return {key: dict(self.views[key], split='train' if key in group['train_ids'] else 'validation')
                for key in keys}

    def test_run_uses_exact_schedule_config_and_owned_float_samples(self):
        args = self.arguments()
        with mock.patch('evaluate_gaussian_sampling.prepare_group', side_effect=self.prepared):
            report = run(args)
        self.assertEqual(report['status'], 'succeeded')
        self.assertEqual(report['split_id'], self.split.split_id)
        self.assertEqual(report['training_calls'], 10)
        self.assertEqual(report['training_steps'], 200)
        self.assertEqual(report['training_options']['depth_loss_weight'], .07)
        self.assertEqual(report['training_options']['learning_rates']['means'], .0002)
        self.assertEqual([r['keyframe_id'] for r in report['queries']], [1, 9])
        self.assertEqual(report['queries'][1]['selected_ids'], [3, 4])
        for group in report['submaps']:
            for operation in group['training']:
                self.assertEqual(len(operation['training_frame_ids']), 20)
                self.assertFalse({1, 9} & set(operation['training_frame_ids']))
        sample = np.load(args.output / 'views/9/samples.npz', allow_pickle=False)
        self.assertEqual(sample['single_rgb'].dtype, np.dtype('<f4'))
        self.assertEqual(sample['joint_rgb'].dtype, np.dtype('<f4'))
        self.assertEqual(json.loads((args.output / 'report.json').read_text())['training_steps'], 200)

    def test_interrupted_training_leaves_failed_ledger_without_success_report(self):
        args = self.arguments()
        with (mock.patch('evaluate_gaussian_sampling.prepare_group', side_effect=self.prepared),
              mock.patch('evaluate_gaussian_sampling.train_group', side_effect=KeyboardInterrupt)):
            with self.assertRaises(KeyboardInterrupt):
                run(args)
        self.assertFalse((args.output / 'report.json').exists())
        failure = json.loads((args.output / 'failure.json').read_text())
        self.assertEqual(failure['status'], 'failed')
        self.assertEqual(len(failure['queries']), 2)

    def test_training_options_control_actual_optimizer(self):
        from dataclasses import replace
        from evaluate_gaussian_submaps import train_group
        from online_model import checked_cuda_device
        from test_online_model import make_view
        from worker_config import DEFAULT_WORKER_OPTIONS

        view = dict(make_view(0), parent_from_body=np.eye(4))
        group = dict(submap_id=0, anchor_id=0, train_ids=[0], held_out_ids=[])
        options = replace(DEFAULT_WORKER_OPTIONS,
                          learning_rates=replace(DEFAULT_WORKER_OPTIONS.learning_rates, means=0.))
        block, _ = train_group(group, [view], [np.eye(4)], checked_cuda_device(), options=options)
        np.testing.assert_array_equal(block.parameters['means'], [[-1.5, .5, 1.]])


if __name__ == '__main__':
    unittest.main()
