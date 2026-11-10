#!/usr/bin/env python3
"""Exercise the installed geometry exporter using stopped online fixtures."""

import argparse
import csv
import hashlib
from pathlib import Path
import subprocess
import tempfile
import unittest


PARSER = argparse.ArgumentParser()
PARSER.add_argument('--executable', required=True, type=Path)
PARSER.add_argument('--fixtures', required=True, type=Path)
ARGS, UNITTEST_ARGS = PARSER.parse_known_args()


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class GeometryCliTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='geometry-cli-')
        self.output = Path(self.temp.name)
        self.dataset = ARGS.fixtures / 'online'

    def tearDown(self):
        self.temp.cleanup()

    def run_cli(self, *args):
        return subprocess.run([str(ARGS.executable), *map(str, args)],
                              text=True, capture_output=True, check=False)

    def test_exports_numeric_geometry_and_sources_without_changing_dataset(self):
        source = self.dataset / 'frames/0/cloud.pcd'
        before = digest(source)
        old = self.output / 'v0'
        corrected = self.output / 'v1'
        self.assertEqual(self.run_cli(self.dataset, old).returncode, 0)
        self.assertEqual(self.run_cli(self.dataset, corrected, '--pose-version', '1').returncode, 0)
        self.assertEqual(digest(source), before)
        for directory, expected in [('0', {0, 1}), ('1', {2})]:
            path = old / 'submaps' / directory
            with (path / 'voxels.csv').open(newline='') as stream:
                voxels = list(csv.DictReader(stream))
            with (path / 'contributors.csv').open(newline='') as stream:
                contributors = list(csv.DictReader(stream))
            self.assertEqual(sum(int(row['count']) for row in voxels), len(expected))
            self.assertEqual({int(row['keyframe_id']) for row in contributors}, expected)
            for voxel in voxels:
                expected_x = {0.0, 1.0} if directory == '0' else {0.0}
                self.assertIn(float(voxel['x']), expected_x)
                self.assertEqual(float(voxel['y']), 0.0)
                self.assertEqual(float(voxel['z']), 4.0)
                self.assertEqual(float(voxel['intensity']), 7.0)
                self.assertEqual(int(voxel['plane_valid']), 0)
                self.assertTrue(all(float(voxel[name]) == 0.0 for name in
                                    ('cxx', 'cxy', 'cxz', 'cyy', 'cyz', 'czz', 'nx', 'ny', 'nz')))
            self.assertEqual((path / 'voxels.csv').read_bytes(),
                             (corrected / 'submaps' / directory / 'voxels.csv').read_bytes())

    def test_cli_rejects_invalid_and_existing_outputs(self):
        out = self.output / 'out'
        for arguments in [('--pose-version', '01'), ('--pose-version', '-1'),
                          ('--voxel-size', '0'), ('--unknown', '1'),
                          ('--voxel-size', '.2', '--voxel-size', '.3')]:
            self.assertEqual(self.run_cli(self.dataset, out, *arguments).returncode, 2)
            self.assertFalse(out.exists())
        self.assertEqual(self.run_cli(self.dataset, out, '--pose-version', '99').returncode, 1)
        self.assertFalse(out.exists())
        out.mkdir()
        self.assertEqual(self.run_cli(self.dataset, out).returncode, 1)


if __name__ == '__main__':
    unittest.main(argv=[__file__, *UNITTEST_ARGS])
