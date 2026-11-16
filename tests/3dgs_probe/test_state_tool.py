import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

from gaussian_state import read_state, write_state
from state_fixtures import make_state


TOOL = Path(__file__).resolve().parents[2] / "tools/3dgs_probe/state_tool.py"


class StateToolTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="livo-state-cli-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.bundle = self.root / "bundle"
        dataset = self.root / "dataset"
        subprocess.run([os.environ["LIVO_FIXTURE_BIN"], str(dataset), "--nonidentity-body"],
                       check=True, capture_output=True)
        subprocess.run([os.environ["LIVO_PREPARE_BIN"], "--dataset", str(dataset),
                        "--output", str(self.bundle), "--train-ids", "0", "--validation-ids", "1",
                        "--source-sha256", "a" * 64, "--max-pixels", "48", "--max-cloud-points", "1"],
                       check=True, capture_output=True)
        # A third synthetic observation reuses identical pixels at a later time.
        path = self.bundle / "bundle.json"
        document = json.loads(path.read_text())
        third = copy.deepcopy(document["views"][1])
        third.update(keyframe_id="2", split="train", state_timestamp=102.0,
                     cloud_timestamp=101.99, image_timestamp=102.01)
        document["views"].append(third)
        for entry in document["views"]:
            entry["frame_count"] = "3"
        path.write_text(json.dumps(document))

    def run_tool(self, *args, environment=None):
        return subprocess.run([sys.executable, str(TOOL), *map(str, args)], capture_output=True,
                              text=True, env=environment, timeout=60)

    def roundtrip(self, output, *args, environment=None):
        return self.run_tool("roundtrip", "--bundle", self.bundle, "--output", output,
                             "--anchor-id", "0", "--submap-id", "0", *args,
                             environment=environment)

    def test_inspect_works_when_torch_import_is_blocked(self):
        state = self.root / "state"
        write_state(state, make_state())
        script = """
import runpy,sys
class BlockTorch:
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] == 'torch':
            raise AssertionError('inspect imported GPU runtime')
sys.meta_path.insert(0, BlockTorch())
sys.argv = [sys.argv[1], 'inspect', '--state', sys.argv[2]]
runpy.run_path(sys.argv[0], run_name='__main__')
"""
        result = subprocess.run([sys.executable, "-c", script, str(TOOL), str(state)],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads(result.stdout)
        self.assertEqual(report["status"], "passed")
        self.assertEqual(report["gaussian_count"], 2)

    def test_roundtrip_reports_success_and_cold_restore(self):
        output = self.root / "run"
        result = self.roundtrip(output)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        report = json.loads((output / "report.json").read_text())
        self.assertEqual(report["status"], "passed")
        self.assertEqual(report["capture_kind"], "offline_probe")
        self.assertTrue(report["parameter_bitwise_equal"])
        self.assertTrue(report["observation_bitwise_equal"])
        self.assertTrue(report["cold_restore"]["released_original"])
        self.assertLess(report["cold_restore"]["allocated_after_release_bytes"],
                        report["cold_restore"]["allocated_before_release_bytes"])
        self.assertEqual(report["continued"]["steps"], 20)
        self.assertTrue(report["continued"]["nonzero_gradient"])
        self.assertTrue(report["continued"]["parameter_updated"])
        restored = read_state(output / "state")
        self.assertEqual(restored.metadata["primary"][0]["submap_from_body"],
                         np.eye(4).ravel().tolist())
        np.testing.assert_allclose(restored.metadata["parent_from_submap"],
                                   [0, -1, 0, 4, 1, 0, 0, -2, 0, 0, 1, 1, 0, 0, 0, 1],
                                   atol=1e-12, rtol=0)
        self.assertEqual(restored.metadata["source_manifest_sha256"], "a" * 64)

    def test_inspect_reports_coverage_provenance_and_resource_usage(self):
        state = self.root / "state"
        write_state(state, make_state())
        result = self.run_tool("inspect", "--state", state)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        report = json.loads(result.stdout)
        self.assertTrue({"schema", "representation", "primary_ids", "frame_count",
                         "optimized_frame_count", "pose_version", "parent_frame",
                         "source_manifest_sha256", "bundle_manifest_sha256", "budget_checks"}
                        <= report.keys(), report)
        self.assertEqual(report["schema"], 1)
        self.assertEqual(report["representation"], "gsplat-sh0-window-v1")
        self.assertEqual(report["primary_ids"], [0, 1])
        self.assertEqual(report["retained_ids"], [0, 1])
        self.assertEqual(report["frame_count"], "2")
        self.assertEqual(report["optimized_frame_count"], "0")
        self.assertEqual(report["pose_version"], "0")
        self.assertEqual(report["parent_frame"], "odom")
        self.assertEqual(report["source_manifest_sha256"], "b" * 64)
        self.assertEqual(report["bundle_manifest_sha256"], "c" * 64)
        budgets = report["budget_checks"]
        for key, used, limit in (("primary_frames", 2, 512), ("retained_views", 2, 10),
                                  ("retained_pixels", 96, 10_000_000),
                                  ("max_view_pixels", 48, 1_000_000), ("gaussians", 2, 10_000)):
            self.assertEqual(budgets[key], {"used": used, "limit": limit})
        for filename, limit in (("manifest.json", 512 * 1024), ("gaussians.npz", 1024 * 1024),
                                 ("observations.npz", 192 * 1024 * 1024)):
            self.assertEqual(budgets["files"][filename],
                             {"used": (state / filename).stat().st_size, "limit": limit})
        no_provenance = make_state()
        no_provenance.metadata.pop("source_manifest_sha256")
        no_provenance.metadata.pop("bundle_manifest_sha256")
        write_state(self.root / "without-provenance", no_provenance)
        result = self.run_tool("inspect", "--state", self.root / "without-provenance")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        report = json.loads(result.stdout)
        self.assertNotIn("source_manifest_sha256", report)
        self.assertNotIn("bundle_manifest_sha256", report)

    def test_old_bundle_bad_anchor_and_overlapping_output_are_rejected(self):
        path = self.bundle / "bundle.json"
        original = path.read_text()
        for anchor, output in (("1", self.root / "bad-anchor"),
                                ("0", self.bundle / "nested")):
            result = self.run_tool("roundtrip", "--bundle", self.bundle, "--output", output,
                                   "--anchor-id", anchor, "--submap-id", "0")
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse(output.exists())
        document = json.loads(original)
        for entry in document["views"]:
            entry.pop("parent_from_body")
        path.write_text(json.dumps(document))
        output = self.root / "old-bundle"
        result = self.roundtrip(output)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("body", result.stdout.lower())
        self.assertFalse(output.exists())

    def test_missing_gpu_and_corrupt_state_return_nonzero(self):
        result = self.roundtrip(self.root / "no-gpu", environment=dict(os.environ, CUDA_VISIBLE_DEVICES=""))
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("CUDA is required", result.stdout + result.stderr)
        state = self.root / "state"
        write_state(state, make_state())
        (state / "gaussians.npz").write_bytes(b"truncated")
        result = self.run_tool("inspect", "--state", state)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(json.loads(result.stdout)["status"], "failed")

    def test_commit_uncertain_keeps_asset_and_reports_nonzero(self):
        output = self.root / "uncertain"
        script = """
import runpy,sys,gaussian_state
def fail_sync(path):
    raise OSError('Simulated directory sync failure')
gaussian_state._sync_directory = fail_sync
sys.argv = [sys.argv[1], 'roundtrip', '--bundle', sys.argv[2], '--output', sys.argv[3],
            '--anchor-id', '0', '--submap-id', '0']
runpy.run_path(sys.argv[0], run_name='__main__')
"""
        result = subprocess.run([sys.executable, "-c", script, str(TOOL), str(self.bundle), str(output)],
                                capture_output=True, text=True, timeout=60)
        self.assertNotEqual(result.returncode, 0)
        report = json.loads((output / "report.json").read_text())
        self.assertEqual(report["status"], "commit_uncertain")
        self.assertGreater(read_state(output / "state").origin_ids.size, 0)

    def test_pixel_budget_is_checked_before_eager_bundle_loader(self):
        path = self.bundle / "bundle.json"
        document = json.loads(path.read_text())
        document["views"][0].update(width=1_000_000, height=2)
        path.write_text(json.dumps(document))
        script = """
import runpy,sys,bundle
def fail_load(path):
    raise AssertionError('Eager loader was reached')
bundle.load_bundle = fail_load
sys.argv = [sys.argv[1], 'roundtrip', '--bundle', sys.argv[2], '--output', sys.argv[3],
            '--anchor-id', '0', '--submap-id', '0']
runpy.run_path(sys.argv[0], run_name='__main__')
"""
        result = subprocess.run([sys.executable, "-c", script, str(TOOL), str(self.bundle),
                                 str(self.root / "oversized")], capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("pixel", result.stdout.lower())

    def test_id_and_frontier_bounds_precede_eager_bundle_loading(self):
        path = self.bundle / "bundle.json"
        original = json.loads(path.read_text())
        script = """
import runpy,sys,bundle
def fail_load(path):
    raise AssertionError('Eager loader was reached')
bundle.load_bundle = fail_load
sys.argv = [sys.argv[1], 'roundtrip', '--bundle', sys.argv[2], '--output', sys.argv[3],
            '--anchor-id', '0', '--submap-id', '0']
runpy.run_path(sys.argv[0], run_name='__main__')
"""
        cases = (([0, 1, 1_000_000_000], [1_000_000_001] * 3),
                 ([0, 1, 2], [513] * 3), ([0, 1, 2], [3, 3, 4]),
                 ([0, 1, 3], [4] * 3), ([0, 1, 1], [3] * 3))
        for index, (ids, frontiers) in enumerate(cases):
            with self.subTest(ids=ids, frontiers=frontiers):
                document = copy.deepcopy(original)
                for entry, key, frontier in zip(document["views"], ids, frontiers):
                    entry.update(keyframe_id=str(key), frame_count=str(frontier))
                path.write_text(json.dumps(document))
                output = self.root / f"bad-frontier-{index}"
                # Stop at the eager loader even with the old implementation:
                # the regression must never allocate the enormous ID span.
                result = subprocess.run([sys.executable, "-c", script, str(TOOL),
                                         str(self.bundle), str(output)], capture_output=True,
                                        text=True, timeout=10)
                self.assertNotEqual(result.returncode, 0)
                report = json.loads(result.stdout)
                self.assertEqual(report["error_type"], "ValueError", report)
                self.assertRegex(report["error"], "frontier|contiguous")
                self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
