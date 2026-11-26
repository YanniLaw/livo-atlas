import argparse
import copy
from dataclasses import replace
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import numpy as np
import torch
import yaml

from diagnose_gaussian_seeds import (TraceModel, cohort_summary, parameter_changes,
                                     run, seed_features, validate_target)
from online_model import OnlineGaussianModel, checked_cuda_device
from test_online_model import make_view
from worker_config import DEFAULT_WORKER_OPTIONS
from heldout_fixtures import make_heldout_snapshot


class SeedFeaturesTest(unittest.TestCase):
    def view(self):
        rgb = np.broadcast_to(np.arange(5)[None, :, None] / 4., (3, 5, 3)).copy()
        camera = np.eye(4)
        camera[0, 3] = 1
        return dict(rgb=rgb, image_mask=np.ones((3, 5), bool), depth_mask=np.ones((3, 5), bool),
                    depth_m=np.full((3, 5), 2.), K=np.eye(3), camera_from_parent=camera, split='train')

    def test_nonidentity_camera_depth_support_and_unknown(self):
        view = self.view()
        good = copy.deepcopy(view)
        good['rgb'][:] = .25
        far = copy.deepcopy(view)
        far['depth_m'][:] = 2.2
        unknown = copy.deepcopy(view)
        unknown['depth_mask'][:] = False
        behind = copy.deepcopy(view)
        behind['camera_from_parent'][2, 3] = -5
        outside = copy.deepcopy(view)
        outside['camera_from_parent'][0, 3] = 1000
        actual = seed_features(view, np.array([7]), [good, far, unknown, behind, outside])
        np.testing.assert_allclose(actual['points'], [[3., 2., 2.]])
        self.assertAlmostEqual(actual['gradient'][0], .25)
        self.assertTrue(actual['gradient_valid'][0])
        for key, value in [('visible_views', 3), ('depth_checks', 2), ('depth_support', 1),
                           ('occluded', 0), ('in_front', 1)]:
            self.assertEqual(actual[key].tolist(), [value])
        self.assertAlmostEqual(actual['color_error_sum'][0], .25)
        summary = cohort_summary(actual, np.array([True]), np.array([True]))
        self.assertEqual(summary['uniform_kept']['depth_support_ratio'], .5)
        self.assertEqual(summary['uniform_kept']['photometric_l1'], .25)

    def test_missing_depth_and_masked_gradient_are_not_zero_evidence(self):
        view = self.view()
        view['image_mask'][1, 1] = False
        result = seed_features(view, np.array([7]), [])
        self.assertFalse(result['gradient_valid'][0])
        summary = cohort_summary(result, np.array([True]), np.array([True]))
        self.assertIsNone(summary['uniform_kept']['gradient_mean'])
        self.assertIsNone(summary['uniform_kept']['depth_support_ratio'])
        self.assertIsNone(summary['uniform_kept']['photometric_l1'])
        self.assertEqual(summary['added']['count'], 0)
        self.assertIsNone(summary['added']['gradient_mean'])
        for pixels in (np.array([-1]), np.array([15]), np.array([7, 7])):
            with self.subTest(pixels=pixels), self.assertRaises(ValueError):
                seed_features(view, pixels, [])
        invalid = copy.deepcopy(view)
        invalid['split'] = 'validation'
        with self.assertRaises(ValueError):
            seed_features(view, np.array([7]), [invalid])

    def test_known_parameter_deltas_and_target_bounds(self):
        initial = dict(means=np.zeros((1, 3)), log_scales=np.full((1, 3), np.log(.03)),
                       color_logits=np.zeros((1, 3)), opacity_logits=np.zeros(1))
        final = dict(means=np.array([[3., 4., 0.]]), log_scales=np.full((1, 3), np.log(.06)),
                     color_logits=np.full((1, 3), np.log(3)), opacity_logits=np.full(1, np.log(3)))
        result = parameter_changes(initial, final)
        self.assertEqual(result['position_l2_m']['mean'], 5)
        self.assertAlmostEqual(result['scale_ratio']['mean'], 2)
        self.assertAlmostEqual(result['color_l1']['mean'], .25)
        self.assertAlmostEqual(result['opacity_delta']['mean'], .25)
        group = dict(train_ids=list(range(12)), held_out_ids=[12])
        validate_target(group, 2)
        for key in (0, 12, True):
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate_target(group, key)
        with self.assertRaises(ValueError):
            parameter_changes(initial, dict(final, means=np.empty((0, 3))))


class SeedTraceCudaTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not torch.cuda.is_available():
            if os.environ.get('LIVO_REQUIRE_GPU') == '1':
                raise RuntimeError('CUDA is required')
            raise unittest.SkipTest('CUDA unavailable')
        cls.device = checked_cuda_device()
        torch.set_num_threads(1)

    def test_real_initialization_owned_trace_and_unchanged_training(self):
        options = replace(DEFAULT_WORKER_OPTIONS, coverage_sampling='replace', coverage_uncovered_ratio=.25)
        views = [make_view(key, width=40, height=30, dense=True) for key in range(3)]
        original = copy.deepcopy(views)
        reference = OnlineGaussianModel(self.device, options=options)
        traced = TraceModel(self.device, 1, options=options)
        histories = []
        for model in (reference, traced):
            torch.manual_seed(3)
            model.reset(1)
            history = []
            for view in views:
                _, receipt = model.train_candidate(view, seed_policy='coverage')
                history.append(receipt['training_frame_ids'])
                model.decide(True)
            histories.append(history)
        self.assertEqual(histories[0], histories[1])
        for name in reference.parameters:
            np.testing.assert_allclose(traced.parameters[name].detach().cpu(),
                                       reference.parameters[name].detach().cpu(), atol=2e-5, rtol=0)
        self.assertEqual(traced.selected_pixels.size, 1000)
        self.assertEqual(len(traced.stages), 3)
        self.assertTrue(traced.prepass_available)
        initial = traced.stages[0][1]['means'].copy()
        traced.parameters['means'].data.add_(1)
        np.testing.assert_array_equal(traced.stages[0][1]['means'], initial)
        for view, before in zip(views, original):
            for key in ('rgb', 'depth_m', 'camera_from_parent'):
                np.testing.assert_array_equal(view[key], before[key])

    def test_cli_records_raw_arrays_and_never_certifies_failed_write(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        directory, keyframes, split, views = make_heldout_snapshot(root, blocks=2)
        config = root / 'config.yaml'
        config.write_text(yaml.safe_dump(dict(gaussian=dict(worker=dict(seed_policy='coverage',
            coverage_sampling='replace', coverage_uncovered_ratio=.25),
            evaluation=dict(max_output_mib=64, min_free_disk_mib=1024)))))
        args = argparse.Namespace(map_directory=directory, keyframes=keyframes,
            prepare_executable=Path(os.environ['LIVO_PREPARE_BIN']), config=config,
            submap_id=0, tracked_id=0, random_seed=0, output=root/'ok')

        def prepared(executable, source, output, group, source_hash):
            return {key: dict(views[key], split='train' if key in group['train_ids'] else 'validation')
                    for key in group['train_ids'] + group['held_out_ids']}

        with mock.patch('diagnose_gaussian_seeds.prepare_group', side_effect=prepared):
            report = run(args)
        self.assertEqual(report['status'], 'succeeded')
        self.assertEqual(report['training'][0]['training_frame_ids'], [0] * 20)
        with np.load(args.output/'trace.npz', allow_pickle=False) as trace:
            self.assertEqual(trace['means'].shape[0], 2)
            self.assertEqual(trace['stage_ids'].tolist(), [-1, 0])
        self.assertFalse(report['prepass_available'])
        args.output = root/'failed'
        with mock.patch('diagnose_gaussian_seeds.prepare_group', side_effect=prepared), \
                mock.patch('diagnose_gaussian_seeds.np.savez_compressed', side_effect=OSError('controlled trace write')):
            with self.assertRaises(OSError):
                run(args)
        self.assertFalse((args.output/'report.json').exists())
        self.assertTrue((args.output/'failure.json').is_file())
        args.output, args.tracked_id = root/'invalid', 1
        with self.assertRaises(ValueError):
            run(args)
        self.assertFalse(args.output.exists())
        args.tracked_id, args.output = 0, root/'small-budget'
        document = yaml.safe_load(config.read_text())
        document['gaussian']['evaluation']['max_output_mib'] = 1
        config.write_text(yaml.safe_dump(document))
        with mock.patch('diagnose_gaussian_seeds.prepare_group', side_effect=prepared), \
                self.assertRaisesRegex(ValueError, 'budget'):
            run(args)
        self.assertFalse(args.output.exists())


if __name__ == '__main__':
    unittest.main()
