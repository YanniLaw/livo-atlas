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
    check_split_identity, file_sha256, prepare_group, read_prepared_group, render_query, run,
    split_primary_ids, summarize_views, train_group)
from test_joint_renderer import parameters
from test_online_model import make_view


class HoldoutPreparationTest(unittest.TestCase):
    def prepare_fixture(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        keyframes, bundle = root / 'dataset', root / 'bundle'
        subprocess.run([os.environ['LIVO_FIXTURE_BIN'], str(keyframes), '--nonidentity-body'],
                       check=True)
        group = dict(submap_id=0, anchor_id=0, frame_count=2, train_ids=[0], held_out_ids=[1])
        views = prepare_group(os.environ['LIVO_PREPARE_BIN'], keyframes, bundle, group, 'a' * 64)
        self.assertEqual({i: view['split'] for i, view in views.items()}, {0: 'train', 1: 'validation'})
        return keyframes, bundle, group

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

        keyframes, bundle, group = self.prepare_fixture()
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

    def test_prepared_pose_must_match_its_raw_camera(self):
        keyframes, bundle, group = self.prepare_fixture()
        manifest = bundle / 'bundle.json'
        document = json.loads(manifest.read_text())
        train, held = document['views']
        self.assertNotEqual(train['camera_from_parent'], held['camera_from_parent'])
        train['camera_from_parent'] = held['camera_from_parent']
        manifest.write_text(json.dumps(document))
        with self.assertRaisesRegex(ValueError, 'camera pose.*keyframe 0'):
            read_prepared_group(keyframes, bundle, group, 'a' * 64)

    def test_prepared_camera_accepts_native_roundoff_projection(self):
        from gaussian_map import _yaml

        keyframes, bundle, group = self.prepare_fixture()
        path = keyframes / 'frames/0/frame.yml'
        raw = _yaml(path)
        rotation = np.asarray(raw['camera_from_odom']['rotation'], dtype='f8')
        raw['camera_from_odom']['rotation'] = (rotation * 1.0000008).tolist()
        path.write_text(json.dumps(raw))
        views = read_prepared_group(keyframes, bundle, group, 'a' * 64)
        expected = json.loads((bundle / 'bundle.json').read_text())['views'][0]['camera_from_parent']
        np.testing.assert_allclose(views[0]['camera_from_parent'].ravel(), expected, atol=1e-12, rtol=0)

    def test_prepared_camera_rejects_invalid_rotation_before_projection(self):
        from gaussian_map import _yaml

        keyframes, bundle, group = self.prepare_fixture()
        path = keyframes / 'frames/0/frame.yml'
        original = _yaml(path)
        for rotation in (np.diag([1.001, 1, 1]), np.diag([-1., 1, 1]),
                         np.full((3, 3), float('nan'))):
            raw = copy.deepcopy(original)
            raw['camera_from_odom']['rotation'] = rotation.ravel().tolist()
            path.write_text(json.dumps(raw))
            with self.subTest(rotation=rotation), self.assertRaises(ValueError):
                read_prepared_group(keyframes, bundle, group, 'a' * 64)

    def test_prepared_assets_must_belong_to_their_keyframe(self):
        keyframes, bundle, group = self.prepare_fixture()
        manifest = bundle / 'bundle.json'
        original = json.loads(manifest.read_text())
        for asset in ('image', 'image_mask', 'depth_mask', 'depth_m'):
            document = copy.deepcopy(original)
            train, held = document['views']
            train[asset] = held[asset]
            manifest.write_text(json.dumps(document))
            with self.subTest(asset=asset), self.assertRaisesRegex(ValueError, 'view 0.*asset'):
                read_prepared_group(keyframes, bundle, group, 'a' * 64)
        manifest.write_text(json.dumps(original))
        train_image = bundle / original['views'][0]['image']
        train_image.unlink()
        train_image.symlink_to(bundle / original['views'][1]['image'])
        with self.assertRaisesRegex(ValueError, 'view 0.*asset'):
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
            self.assertFalse((root / 'views/1/depth-samples.npy').exists())
        self.assertIsNone(result['joint'])
        metrics = result['single']['metrics']
        self.assertGreater(metrics['depth_pixels_available'], 0)
        self.assertEqual(metrics['depth_pixels_used'], 0)
        self.assertIsNone(metrics['depth_l1_m'])
        self.assertEqual(summarize_views([result])['paired']['query_count'], 0)

    def test_coverage_evaluation_records_the_scored_sparse_depth(self):
        groups = [dict(submap_id=i, anchor_id=key, train_ids=[key, key + 2],
                       held_out_ids=[key + 1]) for i, key in enumerate((0, 3))]
        views = {key: dict(make_view(key, dense=True), parent_from_body=np.eye(4),
                           split='validation' if key in (1, 4) else 'train') for key in range(6)}
        views[4]['image_mask'][0, 0] = False
        views[4]['depth_mask'][0, 1] = False
        views[4]['depth_m'][0, 1] = 0.
        split = dict(schema=1, groups=groups, session_id='a' * 32, pose_version=2,
                     frame_count=6, optimized_frame_count=5, parent_frame='map', map_sequence=4)
        poses = [np.eye(4) for _ in views]
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            dataset, maps = root / 'dataset', root / 'maps'
            dataset.mkdir()
            maps.mkdir()
            args = argparse.Namespace(keyframes=dataset, map_directory=maps, output=root / 'out',
                                      prepare_executable=Path(os.environ['LIVO_PREPARE_BIN']),
                                      seed_policy='coverage', save_depth_samples=True)
            with (mock.patch('evaluate_gaussian_submaps.freeze_split', return_value=(split, poses)),
                  mock.patch('evaluate_gaussian_submaps.prepare_group', return_value=views),
                  mock.patch('evaluate_gaussian_submaps.verify_split_sources'),
                  mock.patch('online_model._MAX_PER_VIEW', 16)):
                report = run(args)
            self.assertTrue((args.output / 'report.json').is_file())
            self.assertEqual(report['seed_policy'], 'coverage')
            for group, submap in zip(groups, report['submaps']):
                operations = submap['training']
                self.assertEqual([operation['keyframe_id'] for operation in operations], group['train_ids'])
                self.assertIsNone(operations[0]['seed_selection']['prepass_seconds'])
                self.assertIsNotNone(operations[1]['seed_selection']['prepass_seconds'])
                for operation in operations:
                    self.assertFalse({1, 4} & set(operation['training_frame_ids']))
            for row in report['views']:
                key = row['keyframe_id']
                path = args.output / row['depth_samples']
                self.assertEqual(path.relative_to(args.output).as_posix(), f'views/{key}/depth-samples.npy')
                self.assertEqual(file_sha256(path), row['depth_samples_sha256'])
                samples = np.load(path, allow_pickle=False)
                query = views[key]
                valid = query['image_mask'] & query['depth_mask']
                np.testing.assert_array_equal(samples['pixel_index'], np.flatnonzero(valid))
                np.testing.assert_array_equal(samples['observed_z_m'], query['depth_m'][valid])
                for name in ('single', 'joint'):
                    covered = samples[f'{name}_covered']
                    z = samples[f'{name}_z_m']
                    self.assertTrue((z[~covered] == 0).all())
                    if row[name] is None:
                        self.assertFalse(covered.any())
                        continue
                    metrics = row[name]['metrics']
                    self.assertEqual(metrics['depth_pixels_available'], len(samples))
                    self.assertEqual(metrics['depth_pixels_used'], int(covered.sum()))
                    self.assertGreater(int(covered.sum()), 0)
                    errors = np.abs(z[covered].astype(np.float64) - samples['observed_z_m'][covered])
                    self.assertAlmostEqual(metrics['depth_l1_m'], float(errors.mean()))

    def test_render_failure_leaves_no_success_report(self):
        group, views, poses = prepared_views(3)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            dataset, maps = root / 'dataset', root / 'maps'
            dataset.mkdir()
            maps.mkdir()
            args = argparse.Namespace(keyframes=dataset, map_directory=maps, output=root / 'out',
                                      prepare_executable=Path(os.environ['LIVO_PREPARE_BIN']),
                                      seed_policy='coverage', save_depth_samples=True)
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
