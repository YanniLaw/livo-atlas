from pathlib import Path
import tempfile
import unittest

from tools.run_baseline import exited_cleanly, make_trajectory_path


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
