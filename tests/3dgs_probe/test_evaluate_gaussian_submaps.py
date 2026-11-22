import argparse
import copy
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest import mock

import numpy as np

from evaluate_gaussian_submaps import (TrainedBlock, camera_from_final_parent,
    check_split_identity, prepare_group, read_prepared_group, render_query, run,
    split_primary_ids, summarize_views, train_group)
from test_joint_renderer import parameters
from test_online_model import make_view


class HoldoutPreparationTest(unittest.TestCase):
    def test_split_excludes_held_out_ids(self):
        self.assertEqual(split_primary_ids(168, 4), ([168, 170, 171], [169]))
        train, held = split_primary_ids(0, 18)
        self.assertEqual(held, [1, 9])
        self.assertEqual(train, [0, 2, 3, 4, 5, 6, 7, 8, 10,
                                 11, 12, 13, 14, 15, 16, 17])
        for first, count in ((4, 2), (0, 33), (510, 3)):
            with self.subTest(first=first, count=count), self.assertRaises(ValueError):
                split_primary_ids(first, count)

    def test_rebind_camera_after_loop(self):
        raw_pose = np.array([[0., 1, 0, 4], [-1, 0, 0, 5], [0, 0, 1, 6], [0, 0, 0, 1]])
        raw_camera = np.array([[-1., 0, 0, 5], [0, -1, 0, 7], [0, 0, 1, -3], [0, 0, 0, 1]])
        final_pose = np.array([[1., 0, 0, 7], [0, 0, -1, 8], [0, 1, 0, 9], [0, 0, 0, 1]])
        expected = np.array([[0., 0, -1, 10], [1, 0, 0, -5], [0, -1, 0, 11], [0, 0, 0, 1]])
        view = dict(parent_from_body=raw_pose, camera_from_parent=raw_camera,
                    rgb=np.full((2, 2, 3), .5), depth_m=np.ones((2, 2)))
        original = copy.deepcopy(view)
        actual = camera_from_final_parent(view, final_pose)
        np.testing.assert_allclose(actual, expected, atol=1e-12, rtol=0)
        self.assertEqual(actual.dtype, np.dtype('float64'))
        for key in view:
            np.testing.assert_array_equal(view[key], original[key])

    def test_input_collisions_and_bundle_mismatches_are_rejected(self):
        observations = [dict(keyframe_id=0, split='train', capture_time=1., image_sha256='a' * 64),
                        dict(keyframe_id=5, split='validation', capture_time=2., image_sha256='b' * 64)]
        check_split_identity(observations)
        for field in ('capture_time', 'image_sha256'):
            invalid = copy.deepcopy(observations)
            invalid[1][field] = invalid[0][field]
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, '0.*5'):
                check_split_identity(invalid)

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            keyframes, bundle = root / 'dataset', root / 'bundle'
            subprocess.run([os.environ['LIVO_FIXTURE_BIN'], str(keyframes)], check=True)
            group = dict(submap_id=0, anchor_id=0, frame_count=2, train_ids=[0], held_out_ids=[1])
            views = prepare_group(os.environ['LIVO_PREPARE_BIN'], keyframes, bundle, group, 'a' * 64)
            self.assertEqual({i: view['split'] for i, view in views.items()}, {0: 'train', 1: 'validation'})
            manifest = bundle / 'bundle.json'
            original = json.loads(manifest.read_text())
            for edit in (lambda doc: doc['views'].append(doc['views'][0]),
                         lambda doc: doc['views'].pop(),
                         lambda doc: doc['views'][1].update(keyframe_id='9'),
                         lambda doc: doc['views'][1].update(split='train'),
                         lambda doc: doc['views'][1].update(frame_count='3'),
                         lambda doc: doc['views'][1].update(pose_version='1'),
                         lambda doc: doc['views'][1].pop('parent_from_body')):
                changed = copy.deepcopy(original)
                edit(changed)
                manifest.write_text(json.dumps(changed))
                with self.subTest(manifest=changed), self.assertRaises(ValueError):
                    read_prepared_group(keyframes, bundle, group, 'a' * 64)


def prepared_views(count):
    train, held = split_primary_ids(0, count)
    group = dict(submap_id=0, anchor_id=0, frame_count=count, train_ids=train, held_out_ids=held)
    views = {key: dict(make_view(key, dense=True), parent_from_body=np.eye(4),
                       split='train' if key in train else 'validation') for key in range(count)}
    return group, views, [np.eye(4) for _ in range(count)]


class HoldoutEvaluationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import torch
        from online_model import checked_cuda_device

        if not torch.cuda.is_available():
            if os.environ.get('LIVO_REQUIRE_GPU') == '1':
                raise RuntimeError('CUDA is required for this test')
            raise unittest.SkipTest('CUDA is not available')
        cls.device = checked_cuda_device()
        torch.set_num_threads(1)

    def test_held_out_observation_never_trains(self):
        group, views, poses = prepared_views(14)
        train_ids, held_ids = group['train_ids'], group['held_out_ids']
        with self.assertRaisesRegex(ValueError, 'training split'):
            train_group(group, list(views.values()), poses, self.device)
        block, evidence = train_group(group, [views[key] for key in train_ids], poses, self.device)
        operations = evidence['training']
        self.assertEqual([step['keyframe_id'] for step in operations], train_ids)
        self.assertEqual(evidence['retained_frame_ids'], train_ids[-10:])
        self.assertEqual(set(map(int, evidence['gaussian_origins'])), set(train_ids[-10:]))
        for operation in operations:
            supervised = operation['training_frame_ids']
            self.assertEqual(len(supervised), 20)
            self.assertIn(operation['keyframe_id'], supervised)
            self.assertFalse(set(supervised) & set(held_ids))
        parameters_before = copy.deepcopy(block.parameters)
        query = views[held_ids[0]]
        camera = camera_from_final_parent(query, poses[held_ids[0]])
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            before = render_query(root / 'original', query, camera, block, None,
                                  self.device, 'near_group_start')
            changed = dict(query, rgb=np.ones_like(query['rgb']))
            after = render_query(root / 'changed', changed, camera, block, None,
                                 self.device, 'near_group_start')
            rendered = Path('views') / str(held_ids[0]) / 'single.png'
            self.assertEqual((root / 'original' / rendered).read_bytes(),
                             (root / 'changed' / rendered).read_bytes())
        self.assertNotEqual(before['single']['metrics']['rgb_mse'], after['single']['metrics']['rgb_mse'])
        for name, values in parameters_before.items():
            np.testing.assert_array_equal(block.parameters[name], values)

    def test_first_group_and_missing_depth_are_reported(self):
        group, views, poses = prepared_views(3)
        query = views[1]
        block = TrainedBlock(0, np.eye(4), parameters([[-1, -2, -20]]))
        camera = camera_from_final_parent(query, poses[1])
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            result = render_query(root, query, camera, block, None, self.device, 'near_group_start')
            self.assertFalse((root / 'views/1/joint.png').exists())
        self.assertIsNone(result['joint'])
        metrics = result['single']['metrics']
        self.assertGreater(metrics['depth_pixels_available'], 0)
        self.assertEqual(metrics['depth_pixels_used'], 0)
        self.assertIsNone(metrics['depth_l1_m'])
        self.assertEqual(summarize_views([result])['paired']['query_count'], 0)

    def test_render_failure_leaves_no_success_report(self):
        group, views, poses = prepared_views(3)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            dataset, maps = root / 'dataset', root / 'maps'
            dataset.mkdir()
            maps.mkdir()
            args = argparse.Namespace(keyframes=dataset, map_directory=maps, output=root / 'out',
                                      prepare_executable=Path(os.environ['LIVO_PREPARE_BIN']))
            with (mock.patch('evaluate_gaussian_submaps.freeze_split',
                             return_value=(dict(groups=[group]), poses)),
                  mock.patch('evaluate_gaussian_submaps.prepare_group', return_value=views),
                  mock.patch('evaluate_gaussian_submaps.infer_blocks',
                             side_effect=RuntimeError('simulated render failure'))):
                with self.assertRaisesRegex(RuntimeError, 'simulated render failure'):
                    run(args)
            self.assertTrue((args.output / 'split.json').is_file())
            self.assertFalse((args.output / 'report.json').exists())


if __name__ == '__main__':
    unittest.main()
