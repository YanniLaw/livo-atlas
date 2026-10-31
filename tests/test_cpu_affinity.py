"""Exercise affinity on the real process before its first payload instruction."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


@unittest.skipUnless(hasattr(os, 'sched_getaffinity'), 'Linux CPU affinity required')
class CpuAffinityTest(unittest.TestCase):
    def run_child(self, cpu):
        script = '''
import os, sys
from tools.cpu_affinity import with_cpu_affinity
payload = 'import json, os; print(json.dumps(sorted(os.sched_getaffinity(0))))'
cpu = None if sys.argv[1] == 'none' else int(sys.argv[1])
command = with_cpu_affinity([sys.executable, '-c', payload], cpu)
os.execvp(command[0], command)
'''
        return subprocess.run([sys.executable, '-c', script,
                               'none' if cpu is None else str(cpu)],
                              cwd=Path(__file__).resolve().parents[1],
                              text=True, capture_output=True)

    def test_child_starts_on_the_requested_cpu(self):
        cpu = min(os.sched_getaffinity(0))
        child = self.run_child(cpu)
        self.assertEqual(child.returncode, 0, child.stderr)
        self.assertEqual(json.loads(child.stdout), [cpu])

    def test_default_inherits_the_callers_allowed_cpus(self):
        expected = sorted(os.sched_getaffinity(0))
        child = self.run_child(None)
        self.assertEqual(child.returncode, 0, child.stderr)
        self.assertEqual(json.loads(child.stdout), expected)

    def test_invalid_cpus_never_execute_the_payload(self):
        for cpu in (-1, max(os.sched_getaffinity(0)) + 100000):
            with self.subTest(cpu=cpu):
                child = self.run_child(cpu)
                self.assertNotEqual(child.returncode, 0)
                self.assertEqual(child.stdout, '')

    def test_ros1_module_entry_reaches_input_validation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            missing = root / 'missing.bag'
            output = root / 'output'
            child = subprocess.run(
                [sys.executable, '-m', 'tools.run_baseline', '--workspace', directory,
                 '--bag', str(missing), '--output', str(output)],
                cwd=Path(__file__).resolve().parents[1], text=True, capture_output=True)
            self.assertNotEqual(child.returncode, 0)
            self.assertIn('FileNotFoundError', child.stderr)
            self.assertIn(str(missing), child.stderr)
            self.assertFalse(output.exists())

    def test_ros2_module_entry_rejects_an_existing_output(self):
        with tempfile.TemporaryDirectory() as directory:
            child = subprocess.run(
                [sys.executable, '-m', 'tools.run_ros2_baseline', '--bag', directory,
                 '--executable', '/unused', '--output', directory],
                cwd=Path(__file__).resolve().parents[1], text=True, capture_output=True)
            self.assertNotEqual(child.returncode, 0)
            self.assertIn('FileExistsError', child.stderr)
            self.assertEqual(list(Path(directory).iterdir()), [])


if __name__ == '__main__':
    unittest.main()
