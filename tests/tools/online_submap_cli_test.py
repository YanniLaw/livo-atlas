"""Installed CLI contract against fixtures emitted by installed_consumer."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import unittest


class OnlineSubmapCliTest(unittest.TestCase):
    def invoke(self, dataset, *args):
        return subprocess.run([str(EXECUTABLE), str(FIXTURES / dataset), *args],
                              capture_output=True, text=True)

    def test_online_materializes_declared_tail(self):
        result = self.invoke('online', '--online', '1')
        self.assertEqual(result.returncode, 0, result.stderr)
        row = json.loads(result.stdout)
        self.assertEqual(row['inspection'], 'metadata')
        self.assertEqual(row['mode'], 'online')
        self.assertEqual(row['stored_frame_count'], 3)
        self.assertEqual(row['frame_count'], 4)
        self.assertEqual(row['optimized_frame_count'], 2)
        self.assertEqual(row['primary_frames'], 3)
        self.assertEqual(row['unindexed_frames'], 1)
        self.assertEqual(row['support_references'], 2)
        self.assertEqual(row['active_submaps'], 1)

    def test_legacy_shape_is_unchanged(self):
        result = self.invoke('legacy')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout),
                         dict(submaps=2, frames=4, points=4, pose_version=0))
        self.assertNotEqual(self.invoke('legacy', '--online').returncode, 0)

    def test_version_zero_and_explicit_namespaces(self):
        result = self.invoke('online', '--online')
        self.assertEqual(result.returncode, 0, result.stderr)
        row = json.loads(result.stdout)
        self.assertEqual(row['stored_frame_count'], 0)
        self.assertEqual(row['optimized_frame_count'], 0)
        self.assertEqual(row['frame_count'], 4)
        result = self.invoke('online', '1')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)['frames'], 4)

    def test_bad_arguments_and_unknown_version(self):
        for args in [('--online', '--online'), ('--online', '01'),
                     ('--online', '-1'), ('--online', '18446744073709551616'),
                     ('--online', '1', '2')]:
            self.assertEqual(self.invoke('online', *args).returncode, 2, args)
        self.assertEqual(self.invoke('online', '--online', '9').returncode, 1)

    def test_inspection_is_read_only(self):
        def digest():
            return {str(p.relative_to(FIXTURES)): hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in FIXTURES.rglob('*') if p.is_file()}
        before = digest()
        for args in [(), ('1',), ('--online',), ('--online', '1'), ('--online', '9')]:
            self.invoke('online', *args)
        self.assertEqual(digest(), before)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--executable', type=Path, required=True)
    parser.add_argument('--fixtures', type=Path, required=True)
    options, rest = parser.parse_known_args()
    EXECUTABLE, FIXTURES = options.executable.resolve(), options.fixtures.resolve()
    unittest.main(argv=[__file__, *rest])
