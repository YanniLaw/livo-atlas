from pathlib import Path
import tempfile
import subprocess
import sys
import unittest

from tools.run_baseline import exited_cleanly, make_trajectory_path
from tools import run_baseline, run_ros2_baseline


class BaselineRunnerTest(unittest.TestCase):
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


class GlobalBaselineOptionsTest(unittest.TestCase):
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
