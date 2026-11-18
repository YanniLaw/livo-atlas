import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np

from joint_fixtures import make_snapshot, make_sparse_snapshot
from joint_renderer import load_snapshot
from render_gaussian_map import _sources_report, image_metrics, main, run


def arguments(directory, keyframes, output, query=5, selected=(1, 0)):
    return argparse.Namespace(map_directory=directory, keyframes=keyframes,
                              output=output, keyframe_id=query, submap_ids=list(selected))


def cli(args):
    return ['--map-directory', str(args.map_directory), '--keyframes', str(args.keyframes),
            '--keyframe-id', str(args.keyframe_id), '--submap-ids',
            *map(str, args.submap_ids), '--output', str(args.output)]


def hashes(*directories):
    return {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
            for directory in directories for path in directory.rglob('*') if path.is_file()}


class RenderMetricsTest(unittest.TestCase):
    def test_masked_rgb_coverage_and_sparse_camera_z_are_independent(self):
        query = dict(rgb=np.zeros((2, 2, 3)), image_mask=np.array([[1, 1], [1, 0]], bool),
                     depth_mask=np.array([[1, 1], [1, 0]], bool),
                     depth_m=np.array([[2, 2], [4, 9]], dtype=np.float32))
        result = dict(rgb=np.repeat(np.array([[0, .2], [.4, .6]])[..., None], 3, axis=2),
                      alpha=np.array([[0, .1], [.9, 1]]), depth_m=np.array([[10, 3], [2, 0]]))
        metrics = image_metrics(result, query)
        self.assertAlmostEqual(metrics['rgb_l1'], .2)
        self.assertAlmostEqual(metrics['rgb_mse'], 1 / 15)
        self.assertAlmostEqual(metrics['rgb_psnr_db'], -10 * math.log10(1 / 15))
        self.assertEqual(metrics['image_pixels'], 3)
        self.assertEqual(metrics['covered_pixels'], 2)
        self.assertAlmostEqual(metrics['alpha_coverage'], 2 / 3)
        self.assertEqual(metrics['depth_pixels_available'], 3)
        self.assertEqual(metrics['depth_pixels_used'], 2)
        self.assertAlmostEqual(metrics['depth_l1_m'], 1.5)
        self.assertAlmostEqual(metrics['depth_rmse_m'], math.sqrt(2.5))

    def test_zero_samples_and_perfect_mse_are_json_finite(self):
        query = dict(rgb=np.zeros((2, 2, 3)), image_mask=np.ones((2, 2), bool),
                     depth_mask=np.zeros((2, 2), bool), depth_m=np.zeros((2, 2)))
        result = dict(rgb=query['rgb'].copy(), alpha=np.zeros((2, 2)), depth_m=np.zeros((2, 2)))
        metrics = image_metrics(result, query)
        self.assertTrue(metrics['rgb_perfect'])
        self.assertIsNone(metrics['rgb_psnr_db'])
        self.assertIsNone(metrics['depth_l1_m'])
        self.assertEqual(metrics['depth_pixels_used'], 0)
        query['image_mask'][:] = False
        empty = image_metrics(result, query)
        for name in ('rgb_l1', 'rgb_mse', 'rgb_psnr_db', 'alpha_coverage'):
            self.assertIsNone(empty[name])
        json.dumps(empty, allow_nan=False)


class RenderPreflightTest(unittest.TestCase):
    def test_source_digest_supports_deep_paths_and_portable_identical_inputs(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            short = root / 'short'
            short.mkdir()
            directory, keyframes = make_sparse_snapshot(short)
            deep = root.joinpath(*(['d' * 60] * 20))
            deep.mkdir(parents=True)
            shutil.copytree(directory, deep / 'map')
            shutil.copytree(keyframes, deep / 'keyframes')
            before = _sources_report(load_snapshot(directory, keyframes, 1, [0]))
            after = _sources_report(load_snapshot(deep / 'map', deep / 'keyframes', 1, [0]))
            self.assertEqual(before['verified_small_files_sha256'],
                             after['verified_small_files_sha256'])

    def blocked(self, args):
        code = '''import importlib.abc, sys
class BlockTorch(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname == 'torch' or fullname.startswith('torch.'):
            raise AssertionError('CUDA_IMPORT_BEFORE_VALIDATION')
sys.meta_path.insert(0, BlockTorch())
from render_gaussian_map import main
sys.exit(main(sys.argv[1:]))
'''
        result = subprocess.run([sys.executable, '-c', code, *cli(args)],
                                text=True, capture_output=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn('CUDA_IMPORT_BEFORE_VALIDATION', result.stderr)
        return result

    def test_cli_invalid_ids_fail_without_cuda_or_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            directory, keyframes = make_snapshot(root)
            invalid = ['+1', '01', '-1', '18446744073709551616', 'true', '1.0', '6']
            for value in invalid:
                with self.subTest(query=value):
                    args = arguments(directory, keyframes, root / 'output', query=value)
                    self.blocked(args)
                    self.assertFalse(args.output.exists())
            for selected in ((0, 0), (0, 1, 2), ('01',), (-1,), (2**64,), (9,)):
                with self.subTest(selected=selected):
                    args = arguments(directory, keyframes, root / 'output', selected=selected)
                    self.blocked(args)
                    self.assertFalse(args.output.exists())

    def test_overlapping_existing_or_parentless_output_is_untouched(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            directory, keyframes = make_snapshot(root)
            existing = root / 'existing'
            existing.mkdir()
            sentinel = existing / 'sentinel'
            sentinel.write_text('keep')
            before = hashes(directory, keyframes, existing)
            for output in (existing, root, directory / 'child', keyframes / 'child',
                           root / 'absent/child'):
                with self.subTest(output=output):
                    self.blocked(arguments(directory, keyframes, output))
            self.assertEqual(hashes(directory, keyframes, existing), before)
            self.assertEqual(sentinel.read_text(), 'keep')

    def test_failed_finalization_precedes_cuda_and_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            directory, keyframes = make_snapshot(root)
            path = directory / 'finalization.json'
            report = json.loads(path.read_text())
            report['worker_stopped'] = 0
            path.write_text(json.dumps(report))
            args = arguments(directory, keyframes, root / 'output')
            self.blocked(args)
            self.assertFalse(args.output.exists())


class RenderCudaCliTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import torch
        from online_model import checked_cuda_device
        if not torch.cuda.is_available():
            if os.environ.get('LIVO_REQUIRE_GPU') == '1':
                raise RuntimeError('CUDA required')
            raise unittest.SkipTest('CUDA required')
        checked_cuda_device()
        torch.set_num_threads(1)

    def test_actual_cli_third_owner_and_complete_bounded_artifacts(self):
        from PIL import Image
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            directory, keyframes = make_snapshot(root)
            args = arguments(directory, keyframes, root / 'output')
            before = hashes(directory, keyframes)
            result = subprocess.run([sys.executable, 'tools/3dgs_probe/render_gaussian_map.py',
                                     *cli(args)], text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertEqual(hashes(directory, keyframes), before)
            self.assertEqual({path.name for path in args.output.iterdir()},
                             {'joint.png', 'reference.png', 'single-reference.png',
                              'render.npz', 'report.json'})
            self.assertEqual(args.output.stat().st_mode & 0o777, 0o700)
            raw = (args.output / 'report.json').read_bytes()
            self.assertLessEqual(len(raw), 65536)
            report = json.loads(raw, parse_constant=lambda value: self.fail(value))
            self.assertEqual(report['kind'], 'livo_atlas.gaussian_render')
            self.assertEqual(report['status'], 'passed')
            self.assertTrue(report['training_view'])
            self.assertEqual(report['selected_ids'], ['0', '1'])
            self.assertEqual(report['omitted_ids'], ['2'])
            self.assertEqual(report['missing_ids'], [])
            self.assertEqual(report['query']['owner_submap_id'], '2')
            self.assertFalse(report['query']['owner_selected'])
            self.assertEqual(report['joint']['gaussian_count'], 4)
            self.assertEqual(report['single_reference']['gaussian_count'], 2)
            self.assertEqual(report['blocks'][2]['retained_ids'], ['4', '5'])
            self.assertEqual(report['sources']['map_sha256'],
                             hashlib.sha256((directory / 'map.json').read_bytes()).hexdigest())
            self.assertGreater(report['gpu_peak_allocated_bytes'], 0)
            self.assertGreater(report['rss_peak_bytes'], 0)
            with Image.open(args.output / 'joint.png') as image:
                self.assertEqual(image.size, (8, 6))
            with np.load(args.output / 'render.npz', allow_pickle=False) as arrays:
                self.assertEqual(set(arrays.files), {'rgb', 'alpha', 'depth_m', 'valid_mask'})
                for name in ('rgb', 'alpha', 'depth_m'):
                    self.assertEqual(arrays[name].dtype, np.dtype('<f4'))
                    self.assertTrue(np.isfinite(arrays[name]).all())
                np.testing.assert_array_equal(arrays['valid_mask'], arrays['alpha'] >= .05)

    def test_maximum_content_directory_keeps_report_bounded(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            directory, keyframes = make_snapshot(root, blocks=32)
            args = arguments(directory, keyframes, root / 'output', query=63)
            report = run(args)
            self.assertEqual(len(report['blocks']), 32)
            self.assertLessEqual((args.output / 'report.json').stat().st_size, 65536)

    def test_many_missing_groups_still_publish_a_bounded_success_report(self):
        from gaussian_map import inspect_gaussian_map, inspect_gaussian_finalization

        for content_blocks in (1, 32):
            with self.subTest(content_blocks=content_blocks), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                directory, keyframes = make_sparse_snapshot(root, content_blocks)
                summary = inspect_gaussian_map(directory, keyframes, 0, 512)
                inspect_gaussian_finalization(directory, summary)
                args = arguments(directory, keyframes, root / 'output', query=1, selected=(0,))
                report = run(args)
                self.assertEqual(len(report['blocks']), content_blocks)
                self.assertEqual(report['missing_ids'],
                                 [str(i) for i in range(content_blocks, 512 - content_blocks)])
                self.assertLessEqual((args.output / 'report.json').stat().st_size, 65536)
                self.assertEqual(json.loads((args.output / 'report.json').read_text())['status'], 'passed')

    def test_cuda_oom_and_missing_prebuilt_extension_never_publish_success(self):
        import importlib
        import torch
        import rasterizer
        real_import = importlib.import_module

        def no_extension(name, *args, **kwargs):
            if name == 'gsplat.csrc':
                raise ImportError('controlled missing extension')
            return real_import(name, *args, **kwargs)

        faults = [mock.patch.object(rasterizer, 'render',
                                   side_effect=torch.cuda.OutOfMemoryError('controlled OOM')),
                  mock.patch.object(importlib, 'import_module', side_effect=no_extension)]
        for fault in faults:
            with tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                directory, keyframes = make_snapshot(root)
                args = arguments(directory, keyframes, root / 'output')
                before = hashes(directory, keyframes)
                with fault:
                    self.assertEqual(main(cli(args)), 1)
                self.assertEqual(hashes(directory, keyframes), before)
                self.assertFalse((args.output / 'report.json').exists())
                self.assertLessEqual((args.output / 'failure.json').stat().st_size, 4096)

    def test_long_unicode_failure_keeps_original_exception_and_bounded_report(self):
        import rasterizer

        for text in ('renderer: ' + '\U0001f9e0' * 600,
                     'renderer: ' + '\\\n' * 600,
                     'renderer: ' + '\udcff' * 600):
            with self.subTest(prefix=repr(text[:12])), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                directory, keyframes = make_snapshot(root)
                args = arguments(directory, keyframes, root / 'output')
                before = hashes(directory, keyframes)
                original = RuntimeError(text)
                with mock.patch.object(rasterizer, 'render', side_effect=original):
                    with self.assertRaises(RuntimeError) as caught:
                        run(args)
                self.assertIs(caught.exception, original)
                raw = (args.output / 'failure.json').read_bytes()
                self.assertLessEqual(len(raw), 4096)
                failure = json.loads(raw)
                self.assertEqual(failure['error_type'], 'RuntimeError')
                self.assertTrue(text.startswith(failure['error']))
                self.assertTrue(0 < len(failure['error']) <= 512)
                self.assertFalse((args.output / 'report.json').exists())
                self.assertEqual(hashes(directory, keyframes), before)

    def test_file_and_last_rename_failures_leave_no_success_marker(self):
        real_open, real_rename = os.open, os.rename

        def fail_open(path, *args, **kwargs):
            if Path(path).name == 'report.tmp.json':
                raise OSError('controlled report write failure')
            return real_open(path, *args, **kwargs)

        def fail_rename(source, destination, *args, **kwargs):
            if Path(destination).name == 'report.json':
                raise OSError('controlled last rename failure')
            return real_rename(source, destination, *args, **kwargs)

        faults = [mock.patch.object(np, 'savez', side_effect=OSError('controlled NPZ write failure')),
                  mock.patch.object(os, 'open', side_effect=fail_open),
                  mock.patch.object(os, 'rename', side_effect=fail_rename)]
        for fault in faults:
            with tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                directory, keyframes = make_snapshot(root)
                args = arguments(directory, keyframes, root / 'output')
                before = hashes(directory, keyframes)
                with fault:
                    self.assertEqual(main(cli(args)), 1)
                self.assertEqual(hashes(directory, keyframes), before)
                self.assertFalse((args.output / 'report.json').exists())
                self.assertFalse((args.output / 'report.tmp.json').exists())
                self.assertEqual(json.loads((args.output / 'failure.json').read_text())['status'], 'failed')

    def test_source_change_during_joint_render_prevents_publication(self):
        import rasterizer
        real = rasterizer.render
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            directory, keyframes = make_snapshot(root)
            args = arguments(directory, keyframes, root / 'output')
            calls = 0

            def mutate_after_joint(*values, **kwargs):
                nonlocal calls
                result = real(*values, **kwargs)
                calls += 1
                if calls == 2:
                    (keyframes / 'frames/0/frame.yml').touch()
                return result

            with mock.patch.object(rasterizer, 'render', side_effect=mutate_after_joint):
                self.assertEqual(main(cli(args)), 1)
            self.assertFalse((args.output / 'report.json').exists())
            failure = json.loads((args.output / 'failure.json').read_text())
            self.assertIn('source changed', failure['error'].lower())


if __name__ == '__main__':
    unittest.main()
