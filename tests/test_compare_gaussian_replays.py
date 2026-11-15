import json
from pathlib import Path
import tempfile
import unittest

from PIL import Image
import yaml

from tools.compare_gaussian_replays import compare


class CompareGaussianReplaysTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        root = Path(self.temporary.name)
        self.off, self.on, self.previews = (root / name for name in ("off", "on", "previews"))
        for run in (self.off, self.on):
            (run / "keyframes/frames/0").mkdir(parents=True)
            (run / "keyframes/dataset.yml").write_text("calibration")
            (run / "keyframes/frames/0/image.png").write_bytes(b"pixels")
            for name in ("trajectory.tum", "mat_pre.txt", "mat_out.txt", "imu.txt"):
                (run / name).write_text("1 2\n3 4\n")
            summary = {"clean_shutdown": True, "input_count_mismatches": {},
                       "expected_input_counts": {"/imu": 2}, "observed_counts": {"/imu": 2},
                       "lio_cycles": 2, "vio_cycles": 2, "mapping_cpu_affinity": [0],
                       "playback_rate": 1, "image_queue_size": 32,
                       "binary_sha256": "same-binary",
                       "source": {"file_sha256": {"src/example.cc": "same-source"}}}
            (run / "summary.json").write_text(json.dumps(summary))
            (run / "semantic-output.json").write_text(json.dumps({"/rgb_img": ["same"]}))
            (run / "parameters.yaml").write_text(yaml.safe_dump({"common": {"img_en": 1}}))
            (run / "mapping.log").write_text(
                "livo_atlas loops: frames=1 pose_version=0 accepted=0\n")
        summary = json.loads((self.on / "summary.json").read_text())
        summary["first_gaussian_preview_during_playback_s"] = 1.0
        (self.on / "summary.json").write_text(json.dumps(summary))
        with (self.on / "mapping.log").open("a") as log:
            log.write("livo_atlas gaussian: observed=1 submitted=1 completed=1 accepted=1 "
                      "failed=0 peak_pending=1 peak_in_flight=1\n")
        self.previews.mkdir()
        Image.new("RGB", (2, 2), (30, 20, 10)).save(self.previews / "preview-1.png")
        (self.previews / "preview-1.json").write_text(json.dumps({
            "nonzero_gradient": 1, "parameter_updated": 1,
            "supervised_depth_pixels": 20, "rgb_l1": 0.1, "depth_l1_m": 0.2}))

    def test_identical_frontend_and_confirmed_online_preview_pass(self):
        result = compare(self.off, self.on, self.previews)
        self.assertTrue(result["passed"], result)
        self.assertEqual(result["raw_files"], 2)
        self.assertEqual(result["first_preview_during_playback_s"], 1.0)

    def test_frontend_and_raw_differences_are_reported(self):
        (self.on / "mat_out.txt").write_text("1 2\n3 5\n")
        (self.on / "keyframes/frames/0/image.png").write_bytes(b"changed")
        result = compare(self.off, self.on, self.previews)
        self.assertFalse(result["passed"])
        self.assertIn("mat_out.txt", result["differences"])
        self.assertIn("raw_keyframes", result["differences"])

    def test_missing_online_proof_and_resource_failures_are_reported(self):
        summary = json.loads((self.on / "summary.json").read_text())
        summary["first_gaussian_preview_during_playback_s"] = None
        (self.on / "summary.json").write_text(json.dumps(summary))
        (self.on / "mapping.log").write_text(
            "livo_atlas loops: frames=1 pose_version=0 accepted=0\n"
            "livo_atlas gaussian: observed=1 submitted=1 completed=1 accepted=1 "
            "failed=1 peak_pending=2 peak_in_flight=1\n")
        result = compare(self.off, self.on, self.previews)
        self.assertFalse(result["passed"])
        self.assertIn("online_preview", result["differences"])
        self.assertIn("gaussian_stats", result["differences"])

    def test_incomplete_or_unreadable_preview_is_rejected(self):
        (self.previews / "preview-1.png").write_bytes(b"not a PNG")
        result = compare(self.off, self.on, self.previews)
        self.assertFalse(result["passed"])
        self.assertIn("preview_files", result["differences"])

    def test_parameter_and_final_backend_differences_are_reported(self):
        (self.on / "parameters.yaml").write_text(yaml.safe_dump({"common": {"img_en": 0}}))
        with (self.on / "mapping.log").open("a") as log:
            log.write("livo_atlas loops: frames=1 pose_version=1 accepted=1\n")
        result = compare(self.off, self.on, self.previews)
        self.assertFalse(result["passed"])
        self.assertIn("parameters", result["differences"])
        self.assertIn("loop_backend", result["differences"])

    def test_published_semantics_differences_are_reported(self):
        (self.on / "semantic-output.json").write_text(json.dumps({"/rgb_img": ["changed"]}))
        result = compare(self.off, self.on, self.previews)
        self.assertFalse(result["passed"])
        self.assertIn("published_semantics", result["differences"])

    def test_source_snapshot_difference_is_reported(self):
        summary = json.loads((self.on / "summary.json").read_text())
        summary["source"]["file_sha256"]["src/example.cc"] = "changed"
        (self.on / "summary.json").write_text(json.dumps(summary))
        result = compare(self.off, self.on, self.previews)
        self.assertFalse(result["passed"])
        self.assertIn("source_snapshot", result["differences"])


if __name__ == "__main__":
    unittest.main()
