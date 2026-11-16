import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

import numpy as np
from PIL import Image

from bundle import load_bundle


class BundleTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="livo-view-test-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.dataset = self.root / "dataset"
        self.output = self.root / "bundle"
        subprocess.run([os.environ["LIVO_FIXTURE_BIN"], str(self.dataset)],
                       check=True, capture_output=True, text=True)

    def prepare(self, output=None, **options):
        values = {
            "dataset": self.dataset,
            "output": output if output is not None else self.output,
            "train-ids": "0",
            "validation-ids": "1",
            "source-sha256": "a" * 64,
            "max-pixels": "48",
            "max-cloud-points": "1",
        }
        values.update(options)
        args = [os.environ["LIVO_PREPARE_BIN"]]
        for key, value in values.items():
            args.extend(["--" + key, str(value)])
        return subprocess.run(args, capture_output=True, text=True)

    def make_bundle(self):
        result = self.prepare()
        self.assertEqual(result.returncode, 0, result.stderr)
        return load_bundle(self.output)

    def mutate_manifest(self, edit):
        path = self.output / "bundle.json"
        data = json.loads(path.read_text())
        edit(data)
        path.write_text(json.dumps(data))

    def test_round_trip_preserves_rgb_depth_pose_and_splits(self):
        views = self.make_bundle()
        self.assertEqual(len(views), 2)
        by_id = {view["keyframe_id"]: view for view in views}
        self.assertEqual(set(by_id), {0, 1})
        self.assertEqual(by_id[0]["split"], "train")
        self.assertEqual(by_id[1]["split"], "validation")
        view = by_id[0]
        np.testing.assert_allclose(view["rgb"][3, 4],
                                   np.array([30, 20, 10]) / 255.0)
        self.assertTrue(view["image_mask"][3, 4])
        self.assertTrue(view["depth_mask"][3, 4])
        self.assertEqual(view["depth_m"][3, 4], 4.0)
        np.testing.assert_array_equal(
            view["K"], [[4, 0, 3.5], [0, 4, 2.5], [0, 0, 1]]
        )
        np.testing.assert_allclose(
            view["camera_from_parent"],
            [[0, -1, 0, 1], [1, 0, 0, 2], [0, 0, 1, 3], [0, 0, 0, 1]],
            atol=1e-12, rtol=0,
        )
        self.assertEqual(view["pose_version"], 0)
        raw = np.fromfile(self.output / "views/0/depth.f32le", dtype="<f4")
        self.assertEqual(raw.size, 48)
        self.assertEqual(raw.reshape(6, 8)[3, 4], 4.0)

    def test_existing_and_dataset_nested_outputs_are_rejected(self):
        self.output.mkdir()
        marker = self.output / "keep"
        marker.write_text("untouched")
        self.assertNotEqual(self.prepare().returncode, 0)
        self.assertEqual(marker.read_text(), "untouched")
        self.assertNotEqual(self.prepare(self.dataset / "nested").returncode, 0)
        alias = self.root / "dataset-link"
        alias.symlink_to(self.dataset, target_is_directory=True)
        self.assertNotEqual(self.prepare(alias / "nested").returncode, 0)
        self.assertFalse((self.dataset / "nested").exists())

    def test_optional_body_pose_preserves_old_bundle(self):
        views = self.make_bundle()
        self.assertIn("parent_from_body", views[0])
        np.testing.assert_array_equal(views[0]["parent_from_body"], np.eye(4))
        self.mutate_manifest(lambda data: [entry.pop("parent_from_body")
                                          for entry in data["views"]])
        old = load_bundle(self.output)
        self.assertNotIn("parent_from_body", old[0])
        np.testing.assert_array_equal(old[0]["rgb"], views[0]["rgb"])

    def test_nonidentity_body_pose_matches_source_and_camera(self):
        self.dataset = self.root / "rotated-dataset"
        subprocess.run([os.environ["LIVO_FIXTURE_BIN"], str(self.dataset),
                        "--nonidentity-body"], check=True, capture_output=True)
        views = self.make_bundle()
        np.testing.assert_allclose(views[0]["parent_from_body"],
                                   [[0, -1, 0, 4], [1, 0, 0, -2],
                                    [0, 0, 1, 1], [0, 0, 0, 1]], rtol=0, atol=1e-12)
        np.testing.assert_allclose(views[0]["camera_from_parent"],
                                   [[1, 0, 0, -3], [0, 1, 0, 4],
                                    [0, 0, 1, 2], [0, 0, 0, 1]], rtol=0, atol=1e-12)
        self.assertEqual(views[0]["depth_m"][3, 4], 4.0)
        np.testing.assert_allclose(views[0]["rgb"][3, 4], [30/255, 20/255, 10/255])
        np.testing.assert_array_equal(views[0]["K"], [[4, 0, 3.5], [0, 4, 2.5], [0, 0, 1]])

    def test_rejects_invalid_optional_body_pose(self):
        self.make_bundle()
        for pose in ([1] * 16, [float("nan")] + [0] * 15):
            with self.subTest(pose=pose):
                self.mutate_manifest(lambda data: data["views"][0].__setitem__(
                    "parent_from_body", pose))
                with self.assertRaises(ValueError):
                    load_bundle(self.output)

    def test_rejects_overlapping_ids_and_limits(self):
        result = self.prepare(**{"validation-ids": "0"})
        self.assertNotEqual(result.returncode, 0)
        result = self.prepare(**{"max-pixels": "47"})
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse((self.output / "bundle.json").exists())

    def test_rejects_missing_or_truncated_data(self):
        self.make_bundle()
        path = self.output / "views/0/depth.f32le"
        original = path.read_bytes()
        path.write_bytes(original[:-1])
        with self.assertRaises((OSError, ValueError)):
            load_bundle(self.output)
        path.unlink()
        with self.assertRaises((OSError, ValueError)):
            load_bundle(self.output)

    def test_rejects_invalid_mask_and_depth_outside_image(self):
        self.make_bundle()
        depth_mask = self.output / "views/0/depth_mask.png"
        mask = np.asarray(Image.open(depth_mask)).copy()
        mask[3, 4] = 255
        Image.fromarray(mask).save(depth_mask)
        with self.assertRaises(ValueError):
            load_bundle(self.output)
        mask[3, 4] = 1
        Image.fromarray(mask).save(depth_mask)
        image_mask = self.output / "views/0/image_mask.png"
        mask = np.asarray(Image.open(image_mask)).copy()
        mask[3, 4] = 0
        Image.fromarray(mask).save(image_mask)
        with self.assertRaises(ValueError):
            load_bundle(self.output)

    def test_rejects_nonfinite_pose_and_escaping_paths(self):
        self.make_bundle()
        self.mutate_manifest(
            lambda data: data["views"][0]["camera_from_parent"].__setitem__(
                0, float("nan"))
        )
        with self.assertRaises(ValueError):
            load_bundle(self.output)
        self.mutate_manifest(
            lambda data: data["views"][0]["camera_from_parent"].__setitem__(0, 0)
        )
        self.mutate_manifest(
            lambda data: data["views"][0].__setitem__("image", "../dataset/dataset.yml")
        )
        with self.assertRaises((OSError, ValueError)):
            load_bundle(self.output)


if __name__ == "__main__":
    unittest.main()
