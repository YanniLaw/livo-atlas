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
from heldout_query import load_evaluation_snapshot
import diagnose_gaussian_seeds as diagnostic


def cli_fixture(test, blocks=2):
    temporary = tempfile.TemporaryDirectory()
    test.addCleanup(temporary.cleanup)
    root = Path(temporary.name)
    directory, keyframes, split, views = make_heldout_snapshot(root, blocks=blocks)
    config = root / 'config.yaml'
    config.write_text(yaml.safe_dump(dict(gaussian=dict(worker=dict(seed_policy='coverage',
        coverage_sampling='replace', coverage_uncovered_ratio=.25),
        evaluation=dict(max_output_mib=64, min_free_disk_mib=1024)))))
    args = argparse.Namespace(map_directory=directory, keyframes=keyframes,
        prepare_executable=Path(os.environ['LIVO_PREPARE_BIN']), config=config,
        submap_id=0, tracked_id=0, random_seed=0, output=root/'out')

    def prepared(executable, source, output, group, source_hash):
        return {key: dict(views[key], split='train' if key in group['train_ids'] else 'validation')
                for key in group['train_ids'] + group['held_out_ids']}

    return args, prepared


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

    def test_native_pixel_footprint_accepts_rotated_borders_and_half_pixel_ties(self):
        view = self.view()
        angle = .37
        view['camera_from_parent'] = np.array([
            [np.cos(angle), 0, np.sin(angle), 1.7], [0, 1, 0, -1.3],
            [-np.sin(angle), 0, np.cos(angle), .8], [0, 0, 0, 1]])
        view['K'] = np.array([[381., 0, 2], [0, 379, 1], [0, 0, 1]])
        result = seed_features(view, np.arange(15), [copy.deepcopy(view)])
        np.testing.assert_array_equal(result['depth_support'], np.ones(15))
        view = self.view()
        for offset, expected in ((1., 1), (-4.8, 1), (-5.2, 0)):
            history = copy.deepcopy(view)
            history['camera_from_parent'][0, 3] += offset
            if offset == 1.:
                history['depth_mask'][:] = False
                history['depth_mask'][1, 3] = True
            with self.subTest(offset=offset):
                result = seed_features(view, np.array([7]), [history])
                self.assertEqual(result['depth_support'][0], expected)

    def test_unrelated_heldout_only_group_does_not_block_selected_group(self):
        args, prepared = cli_fixture(self, blocks=5)
        snapshot = load_evaluation_snapshot(args.map_directory, args.keyframes)
        first, last = copy.deepcopy(snapshot.document['entries'][0]), copy.deepcopy(snapshot.document['entries'][-1])
        first.update(primary_count=9)
        last.update(submap_id='1', anchor_keyframe_id='9', primary_count=1)
        snapshot.document['entries'] = [first, last]
        for key, frame in snapshot.split.frames.items():
            frame['submap_id'] = '0' if key < 9 else '1'
        args.tracked_id = 8
        with mock.patch('diagnose_gaussian_seeds.load_evaluation_snapshot', return_value=snapshot), \
                mock.patch('diagnose_gaussian_seeds.prepare_group', side_effect=prepared), \
                mock.patch('diagnose_gaussian_seeds.initialize_cuda', side_effect=RuntimeError('selected group admitted')):
            with self.assertRaisesRegex(RuntimeError, 'selected group admitted'):
                run(args)

    def test_preparation_budget_includes_borrowed_heldout_frame(self):
        args, _ = cli_fixture(self, blocks=2)
        snapshot = load_evaluation_snapshot(args.map_directory, args.keyframes)
        snapshot.split.frames[1].update(width=1000, height=1000)
        args.submap_id, args.tracked_id = 1, 3
        free = 1024**3 + 66*1024**2
        with mock.patch('diagnose_gaussian_seeds.load_evaluation_snapshot', return_value=snapshot), \
                mock.patch('diagnose_gaussian_seeds.shutil.disk_usage', return_value=mock.Mock(free=free)), \
                mock.patch('diagnose_gaussian_seeds.prepare_group', side_effect=AssertionError('insufficient preparation reserve')):
            with self.assertRaisesRegex(ValueError, 'reserve'):
                run(args)
        self.assertFalse(args.output.exists())

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


    def test_partial_report_write_never_publishes_success_path(self):
        args, prepared = cli_fixture(self)
        write = diagnostic._exclusive_write

        def partial(path, payload):
            if 'report.json' in path.name:
                write(path, payload[:len(payload)//2])
                raise OSError('controlled report flush')
            return write(path, payload)

        with mock.patch('diagnose_gaussian_seeds.prepare_group', side_effect=prepared), \
                mock.patch('diagnose_gaussian_seeds._exclusive_write', side_effect=partial):
            with self.assertRaises(OSError):
                run(args)
        self.assertFalse((args.output/'report.json').exists())
        self.assertTrue((args.output/'failure.json').exists())

    def test_disk_reserve_is_rechecked_before_trace_and_report(self):
        args, prepared = cli_fixture(self)
        base = args.output
        for drops_at in (1, 2, 3):
            with self.subTest(drops_at=drops_at):
                args.output = base.with_name('disk-'+str(drops_at))
                low = 1024**3 + (1024 if drops_at < 3 else -1024)
                values = [mock.Mock(free=8*1024**3)]*drops_at + [mock.Mock(free=low)]*4
                with mock.patch('diagnose_gaussian_seeds.prepare_group', side_effect=prepared), \
                        mock.patch('diagnose_gaussian_seeds.shutil.disk_usage', side_effect=values):
                    with self.assertRaisesRegex(ValueError, 'reserve'):
                        run(args)
                self.assertFalse((args.output/'report.json').exists())


if __name__ == '__main__':
    unittest.main()
