from pathlib import Path
import tempfile
import subprocess
import sys
import unittest

from tools.run_baseline import exited_cleanly, make_trajectory_path
from tools import run_baseline, run_ros2_baseline


class BaselineRunnerTest(unittest.TestCase):
    def test_gaussian_preview_requires_a_complete_pair(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.assertFalse(run_baseline.gaussian_preview_available(root))
            (root / 'preview-1.png').write_bytes(b'image')
            self.assertFalse(run_baseline.gaussian_preview_available(root))
            (root / 'preview-1.json').write_text('{}')
            self.assertTrue(run_baseline.gaussian_preview_available(root))

    def test_mapping_process_requires_zero_exit_status(self):
        self.assertTrue(exited_cleanly(0, '', False))
        self.assertFalse(exited_cleanly(-11, '', False))

    def test_gdb_requires_normal_inferior_exit_evidence(self):
        self.assertFalse(exited_cleanly(0, '', True))
        self.assertTrue(exited_cleanly(0, '[Inferior 1 (process 123) exited normally]', True))

    def test_gdb_fatal_signals_do_not_pass(self):
        for name in ['SIGSEGV', 'SIGABRT', 'SIGBUS', 'SIGFPE']:
            with self.subTest(signal=name):
                self.assertFalse(exited_cleanly(0, 'Program received signal ' + name, True))

    def test_gdb_nonzero_inferior_exit_does_not_pass(self):
        self.assertFalse(exited_cleanly(0, '[Inferior 1 (process 123) exited with code 01]', True))

    def test_gdb_signal_stop_is_rejected_even_with_a_normal_exit_marker(self):
        log = 'Program received signal SIGABRT\n[Inferior 1 (process 123) exited normally]'
        self.assertFalse(exited_cleanly(0, log, True))

    def test_stale_trajectory_is_rejected_and_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            package = Path(directory)
            stale = package / 'Log/result/test.txt'
            stale.parent.mkdir(parents=True)
            stale.write_text('old trajectory\n')
            with self.assertRaisesRegex(RuntimeError, 'exists'):
                make_trajectory_path(package, 'test')
            self.assertEqual(stale.read_text(), 'old trajectory\n')

    def test_ros2_separate_logs_preserve_stage_counts_and_fatal_errors(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'node.log').write_text('LIO Mapping Time\nVIO Time\n')
            (root / 'node-stderr.log').write_text('[FATAL] shutdown failed\n')
            stdout, stderr = run_ros2_baseline.read_node_logs(root)
            self.assertEqual(stdout.count('LIO Mapping Time'), 1)
            self.assertEqual(stdout.count('VIO Time'), 1)
            self.assertFalse(run_ros2_baseline.exited_cleanly(0, stdout + stderr))


class GlobalBaselineParserTest(unittest.TestCase):
    def invoke(self, script, directory, *flags):
        package = Path(__file__).resolve().parents[1]
        command = [sys.executable, str(package / 'tools' / script),
                   '--bag', str(directory / 'missing-bag'),
                   '--output', str(directory / 'output')]
        if script == 'run_baseline.py':
            command += ['--workspace', str(directory / 'missing-workspace')]
        else:
            command += ['--executable', str(directory / 'missing-node')]
        return subprocess.run(command + list(flags), capture_output=True, text=True)

    def test_global_output_rejects_groups_before_io(self):
        for script in ('run_baseline.py', 'run_ros2_baseline.py'):
            with self.subTest(script=script), tempfile.TemporaryDirectory() as name:
                root = Path(name)
                result = self.invoke(script, root, '--global-output', '--submap-mode', 'keyframe_groups')
                self.assertEqual(result.returncode, 2)
                self.assertIn('--global-output requires online submaps', result.stderr)
                self.assertFalse((root / 'output').exists())

    def test_global_output_preserves_queue_validation(self):
        for script in ('run_baseline.py', 'run_ros2_baseline.py'):
            with self.subTest(script=script), tempfile.TemporaryDirectory() as name:
                root = Path(name)
                result = self.invoke(script, root, '--global-output', '--image-queue-size', '0')
                self.assertEqual(result.returncode, 2)
                self.assertIn('--image-queue-size must be positive', result.stderr)
                self.assertFalse((root / 'output').exists())

    def test_geometry_map_rejects_groups_before_io(self):
        for script in ('run_baseline.py', 'run_ros2_baseline.py'):
            with self.subTest(script=script), tempfile.TemporaryDirectory() as name:
                root = Path(name)
                result = self.invoke(script, root, '--geometry-map', '--submap-mode', 'keyframe_groups')
                self.assertEqual(result.returncode, 2)
                self.assertIn('--geometry-map requires online submaps', result.stderr)
                self.assertFalse((root / 'output').exists())

    def test_gaussian_persistence_requires_worker_paths(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            result = self.invoke('run_baseline.py', root, '--gaussian-persist-submaps')
            self.assertEqual(result.returncode, 2)
            self.assertIn('requires --gaussian-socket', result.stderr)
            self.assertFalse((root / 'output').exists())
            args = run_baseline.parse_arguments([
                '--workspace', str(root / 'ws'), '--bag', str(root / 'bag'),
                '--output', str(root / 'output'), '--gaussian-persist-submaps',
                '--gaussian-socket', str(root / 'worker.sock'),
                '--gaussian-output', str(root / 'previews')])
            self.assertTrue(args.gaussian_persist_submaps)

    def test_gaussian_arguments_are_paired_and_require_online_submaps(self):
        for flags in (['--gaussian-socket', '/tmp/worker.sock'],
                      ['--gaussian-output', '/tmp/preview'],
                      ['--gaussian-socket', '', '--gaussian-output', '/tmp/preview'],
                      ['--gaussian-socket', '/tmp/worker.sock', '--gaussian-output', '',],
                      ['--gaussian-socket', '/tmp/worker.sock', '--gaussian-output', '/tmp/preview',
                       '--submap-mode', 'keyframe_groups']):
            with self.subTest(flags=flags), tempfile.TemporaryDirectory() as name:
                root = Path(name)
                result = self.invoke('run_baseline.py', root, *flags)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertFalse((root / 'output').exists())

    def test_gaussian_rejects_existing_output_without_modifying_it(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            previews = root / 'previews'
            previews.mkdir()
            sentinel = previews / 'keep.txt'
            sentinel.write_text('keep')
            result = self.invoke('run_baseline.py', root,
                                 '--gaussian-socket', str(root / 'worker.sock'),
                                 '--gaussian-output', str(previews))
            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertEqual(sentinel.read_text(), 'keep')
            self.assertFalse((root / 'output').exists())


class GlobalBaselineOptionsTest(unittest.TestCase):
    def test_gaussian_arguments_enable_required_storage(self):
        options = run_baseline.parse_arguments(
            ['--workspace', '/unused/workspace', '--bag', '/unused/bag',
             '--output', '/unused/output', '--gaussian-socket', '/tmp/worker.sock',
             '--gaussian-output', '/tmp/gaussian-test-output'])
        self.assertEqual(options.submap_mode, 'online')
        self.assertTrue(options.save_keyframes)
        self.assertTrue(options.save_submaps)
        self.assertFalse(options.loop_closure)
        self.assertFalse(options.global_output)

    def test_geometry_map_enables_only_its_required_inputs(self):
        for module in (run_baseline, run_ros2_baseline):
            with self.subTest(module=module.__name__):
                required = ['--bag', '/unused/bag', '--output', '/unused/output']
                required += (['--workspace', '/unused/workspace'] if module is run_baseline
                             else ['--executable', '/unused/node'])
                options = module.parse_arguments(required + ['--geometry-map'])
                self.assertTrue(options.geometry_map)
                self.assertTrue(options.global_output)
                self.assertTrue(options.save_keyframes)
                self.assertTrue(options.save_submaps)
                self.assertEqual(options.submap_mode, 'online')
                self.assertFalse(options.loop_closure)
                self.assertFalse(options.geometry_submaps)
                self.assertFalse(options.keyframe_trajectory)

    def test_keyframe_trajectory_enables_only_its_required_inputs(self):
        for module in (run_baseline, run_ros2_baseline):
            with self.subTest(module=module.__name__):
                required = ['--bag', '/unused/bag', '--output', '/unused/output']
                required += (['--workspace', '/unused/workspace'] if module is run_baseline
                             else ['--executable', '/unused/node'])
                options = module.parse_arguments(required + ['--keyframe-trajectory'])
                self.assertTrue(options.keyframe_trajectory)
                self.assertTrue(options.global_output)
                self.assertTrue(options.save_keyframes)
                self.assertTrue(options.save_submaps)
                self.assertEqual(options.submap_mode, 'online')
                self.assertFalse(options.loop_closure)
                self.assertFalse(options.geometry_submaps)

    def test_global_only_enables_online_storage_without_loop_or_geometry(self):
        for module in (run_baseline, run_ros2_baseline):
            with self.subTest(module=module.__name__):
                required = ['--bag', '/unused/bag', '--output', '/unused/output']
                required += (['--workspace', '/unused/workspace'] if module is run_baseline
                             else ['--executable', '/unused/node'])
                defaults = module.parse_arguments(required)
                self.assertFalse(defaults.global_output)
                self.assertEqual(defaults.submap_mode, 'keyframe_groups')
                self.assertFalse(defaults.save_keyframes)
                self.assertFalse(defaults.save_submaps)
                options = module.parse_arguments(required + ['--global-output'])
                self.assertEqual(options.submap_mode, 'online')
                self.assertTrue(options.save_submaps)
                self.assertTrue(options.save_keyframes)
                self.assertFalse(options.loop_closure)
                self.assertFalse(options.geometry_submaps)
