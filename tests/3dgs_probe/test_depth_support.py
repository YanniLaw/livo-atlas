import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np
import yaml

import evaluate_heldout_snapshot as scorer
from heldout_fixtures import make_heldout_snapshot
from joint_fixtures import transform


def write_cloud(path, xyz):
    points = np.zeros((len(xyz), 8), dtype='<f4')
    points[:, :3] = xyz
    header = ('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity normal_x normal_y normal_z curvature\n'
              'SIZE 4 4 4 4 4 4 4 4\nTYPE F F F F F F F F\nCOUNT 1 1 1 1 1 1 1 1\n'
              f'WIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(xyz)}\nDATA binary\n')
    path.write_bytes(header.encode('ascii') + points.tobytes())


def write_source(folder, view, *, second_label=False):
    record = yaml.safe_load((folder / 'frame.yml').read_text())
    body = np.eye(4)
    body[:3, :3] = np.array(record['odom_from_body']['rotation']).reshape(3, 3)
    body[:3, 3] = record['odom_from_body']['translation']
    camera = view['camera_from_body']
    record['camera_from_odom'] = transform(camera @ np.linalg.inv(body))
    (folder / 'frame.yml').write_text(yaml.safe_dump(record))
    depth, intrinsic = view['depth_m'], view['K']
    height, width = depth.shape
    if second_label:
        depth[height//2, width//2+2] = 4.
        view['depth_mask'] = depth > 0
    y, x = np.nonzero(depth)
    z = depth[y, x]
    xy = (np.column_stack((x, y))-.25-intrinsic[:2, 2])/np.diag(intrinsic)[:2] * z[:, None]
    points = np.column_stack((xy, z))
    write_cloud(folder / 'cloud.pcd', (points-camera[:3, 3]) @ camera[:3, :3])
    dataset = folder.parents[1] / 'dataset.yml'
    dataset.write_text(yaml.safe_dump(dict(camera=dict(width=width, height=height, scale=1.,
                                                       fx=float(intrinsic[0, 0]), fy=float(intrinsic[1, 1])))))


def score_case(root, directory, keyframes, views, labels):
    config = root / 'gaussian.yaml'
    config.write_text(yaml.safe_dump(dict(gaussian=dict(evaluation=dict(
        max_output_mib=16, min_free_disk_mib=0), joint_render_policy='temporal'))))
    preparer = root / 'prepare'
    preparer.write_text('fixture')
    runs = []
    for label, customize in labels.items():
        def render(blocks, _intrinsic, w, h, _device):
            view = views[1 if max(block[0] for block in blocks) == 0 else 9]
            alpha, depth = np.ones((h, w), 'f4'), view['depth_m'].copy()
            customize(alpha, depth, len(blocks))
            return dict(rgb=view['rgb'], alpha=alpha, depth_m=depth)

        def prepare(_snapshot, owner, *_rest):
            key = 1 if owner == 0 else 9
            return {key: views[key]}

        args = argparse.Namespace(map_directory=directory, keyframes=keyframes, output=root/label,
                                  config=config, prepare_executable=preparer)
        with mock.patch.object(scorer, 'initialize_cuda', return_value=('cpu', {})), \
                mock.patch.object(scorer, 'prepare_group_queries', side_effect=prepare):
            scorer.run(args, renderer=render)
        runs.append([label, str(directory), str(keyframes), str(root/label)])
    return runs


class DepthSupportTest(unittest.TestCase):
    def test_cli_reads_saved_scores_on_cpu_and_preserves_existing_report(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            directory, keyframes, _, views = make_heldout_snapshot(root, write_source=write_source)

            def increase(_alpha, depth, _blocks):
                depth[depth > 0] += .5

            runs = score_case(root, directory, keyframes, views, dict(flat=lambda *_: None, additions=increase))
            # Old reports may still contain the archive's original mount paths.
            moved = root / 'flat/report.json'
            archived = json.loads(moved.read_text())
            archived['sources']['map_directory'] = '/recorded/old/location/map'
            archived['selection_policy'] = 'spatial'
            moved.write_text(json.dumps(archived))

            output = root / 'depth-report.json'
            tool = Path(__file__).resolve().parents[2] / 'tools/3dgs_probe/evaluate_depth_support.py'
            command = [sys.executable, str(tool), '--run', 'flat', str(directory), str(keyframes), str(root/'flat'),
                       '--run', 'additions', str(directory), str(keyframes), str(root/'additions'),
                       '--pair', 'flat', 'additions', '--output', str(output)]
            environment = dict(os.environ, CUDA_VISIBLE_DEVICES='')
            result = subprocess.run(command, env=environment, capture_output=True, text=True, timeout=45)
            self.assertEqual(result.returncode, 0, result.stderr)
            original = output.read_bytes()
            report = json.loads(original)
            group = report['modes']['single']['radii']['2']['reference']
            self.assertEqual(group['pixels'], 1)
            self.assertEqual(group['deltas']['flat:additions']['bias_m'], .5)
            self.assertEqual(group['deltas']['flat:additions']['rmse_m'], .5)
            self.assertEqual(report['modes']['single']['radii']['2']['competing']['metrics'], {})
            self.assertEqual(report['runs']['flat']['selection_policy'], 'spatial')
            self.assertEqual(report['runs']['additions']['selection_policy'], 'temporal')
            result = subprocess.run(command, env=environment, capture_output=True, text=True, timeout=45)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(output.read_bytes(), original)

            import evaluate_depth_support as tool

            split = root / 'flat/split.json'
            payload = split.read_bytes()
            write = tool._exclusive_write

            def mutate_split(path, content):
                write(path, content)
                split.write_bytes(payload+b' ')

            raced_output = root / 'raced-depth-report.json'
            request = argparse.Namespace(run=runs, pair=[['flat', 'additions']], exclude_query=[], output=raced_output)
            with mock.patch.object(tool, '_exclusive_write', side_effect=mutate_split):
                with self.assertRaisesRegex(ValueError, 'Score split changed'):
                    tool.run(request)
            self.assertFalse(raced_output.exists())
            split.write_bytes(payload)
            self.assertEqual(report['views']['1']['query'], archived['queries'][0]['query'])
            self.assertEqual(report['views']['1']['selected_ids'], [0])
            corrupted = json.loads(moved.read_text())
            corrupted['queries'][0]['sources'][0]['training_lineage_ids'] = [1]
            moved.write_text(json.dumps(corrupted))
            with self.assertRaisesRegex(ValueError, 'Query training lineage differs'):
                tool.run(request)
            self.assertFalse(raced_output.exists())
            moved.write_text(json.dumps(archived))
            with (root/'additions/views/1/samples.npz').open('ab') as stream:
                stream.write(b'tampered')
            command[-1] = str(root/'bad-depth-report.json')
            result = subprocess.run(command, env=environment, capture_output=True, text=True, timeout=45)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('Sample artifact differs', result.stderr)
            self.assertFalse(Path(command[-1]).exists())

    def test_all_arms_share_a_different_mask_for_single_and_joint(self):
        import evaluate_depth_support as tool

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            directory, keyframes, _, views = make_heldout_snapshot(
                root, blocks=5, write_source=lambda folder, view: write_source(folder, view, second_label=True))

            def increase(_alpha, depth, _blocks):
                depth[3, 4] += .25
                depth[3, 6] += .5

            def restrict(alpha, _depth, blocks):
                alpha[3, 4 if blocks == 1 else 6] = 0.

            runs = score_case(root, directory, keyframes, views,
                              dict(flat=lambda *_: None, additions=increase, third=restrict))
            request = argparse.Namespace(run=runs, pair=[['flat', 'additions']], exclude_query=[1],
                                         output=root/'paired.json')
            report = tool.run(request)
            self.assertEqual(report['views']['9']['selected_ids'], [3, 4])
            masks = []
            for mode, expected in (('single', .5), ('joint', .25)):
                result = report['views']['9']['modes'][mode]
                self.assertEqual(result['all']['pixels'], 1)
                self.assertEqual(result['all']['deltas']['flat:additions']['bias_m'], expected)
                masks.append(result['depth_mask_sha256'])
            self.assertNotEqual(*masks)
            request = argparse.Namespace(run=runs[:1], pair=[], exclude_query=[1], output=root/'one.json')
            report = tool.run(request)
            self.assertEqual(report['modes']['joint']['all']['pixels'], 2)
            self.assertEqual(report['modes']['joint']['all']['deltas'], {})

    def test_native_reference_replays_unrounded_camera_intrinsics(self):
        from depth_support import project_support

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            folder = root / 'frames/0'
            folder.mkdir(parents=True)
            (root/'dataset.yml').write_text(yaml.safe_dump(dict(camera=dict(
                width=100, height=8, scale=1., fx=100.00000000004, fy=100.))))
            pose = transform(np.eye(4))
            (folder/'frame.yml').write_text(yaml.safe_dump(dict(camera_from_odom=pose, odom_from_body=pose)))
            write_cloud(folder/'cloud.pcd', [[-.25, 0., 1.]])
            reference = np.zeros((8, 100), 'f4')
            reference[4, 24] = 1.
            snapshot = SimpleNamespace(keyframes=root, split=SimpleNamespace(frames={0: {'cloud_timestamp': 0.}}))
            camera = dict(K=[[100., 0., 49.5], [0., 100., 3.5], [0., 0., 1.]],
                          camera_from_body=np.eye(4).ravel().tolist(), camera_from_parent=np.eye(4).ravel().tolist())
            neighbors, votes = project_support(snapshot, 0, camera, reference, np.ones_like(reference, dtype=bool), reference > 0)
            self.assertEqual(neighbors, [])
            self.assertEqual(votes[2].tolist(), [[0, 0, 0, 0]])

    def test_small_pair_delta_survives_cancellation_of_large_squared_totals(self):
        from depth_support import compare_view

        count = 100_000
        baseline, candidate = 90., float(np.nextafter(np.float32(100), np.float32(101)))-10.
        errors = dict(flat=np.full(count, baseline), additions=np.full(count, candidate))
        vote = np.tile([2, 0, 2, 0], (count, 1))
        result = compare_view(errors, {'pair': ('flat', 'additions')}, np.ones(count, dtype=bool), {2: vote, 4: vote})
        self.assertAlmostEqual(result['all']['deltas']['pair']['squared_m2'], count*(candidate-baseline)*(candidate+baseline))

    def test_native_projection_uses_nearest_valid_pixel_and_exact_cloud_size(self):
        from depth_support import project_depth, read_body_cloud

        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'cloud.pcd'
            write_cloud(path, [[0, 0, 4], [0, 0, 2], [0, 0, -1], [4, 0, 2]])
            points = read_body_cloud(path)
            image = np.ones((5, 5), dtype=bool)
            image[2, 4] = False
            camera = np.array([[1., 0., 1.5], [0., 1., 1.5], [0., 0., 1.]])
            z = project_depth(points, np.eye(4), camera, image)
            expected = np.zeros((5, 5), 'f4')
            expected[2, 2] = 2.
            np.testing.assert_array_equal(z, expected)
            path.write_bytes(path.read_bytes()[:-1])
            with self.assertRaisesRegex(ValueError, 'PCD payload'):
                read_body_cloud(path)

    def test_neighborhood_votes_count_frames_and_preserve_all_five_groups(self):
        from depth_support import support_masks, support_votes

        reference = np.full((9, 41), 4., 'f4')
        left, right = (np.zeros_like(reference) for _ in range(2))
        for buffer in (left, right):
            buffer[4, [0, 10, 11, 20]] = [4., 4., 2., 2.]
        left[4, [30, 40]] = [2., 4.]
        right[4, 30] = 6.
        x = np.array([0, 10, 20, 30, 40])
        y = np.full(5, 4)
        expected = np.eye(5, dtype=bool)
        for vote in support_votes([left, right], reference, y, x).values():
            np.testing.assert_array_equal(list(support_masks(vote).values()), expected)
            self.assertEqual(int(vote[1, 1]), 2)
            self.assertEqual(int(vote[1, 2]), 2)
