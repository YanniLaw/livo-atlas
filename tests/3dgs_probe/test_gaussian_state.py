import copy
import hashlib
import io
import json
import os
from pathlib import Path
import stat
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
import warnings
import zipfile

import numpy as np

import gaussian_state
from gaussian_state import CommitUncertainError, read_state, validate_state, write_state
from state_fixtures import make_state


class GaussianStateTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="livo-state-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.output = self.root / "state"
        self.state = make_state()

    def saved(self):
        write_state(self.output, self.state)
        return self.output

    def update_manifest(self, edit):
        path = self.output / "manifest.json"
        data = json.loads(path.read_text())
        edit(data)
        path.write_text(json.dumps(data))

    def replace_archive(self, edit, compression=zipfile.ZIP_STORED, filename="gaussians.npz"):
        path = self.output / filename
        with zipfile.ZipFile(path) as archive:
            entries = [(name, archive.read(name)) for name in archive.namelist()]
        entries = edit(entries)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            with zipfile.ZipFile(path, "w", compression=compression) as archive:
                for name, payload in entries:
                    archive.writestr(name, payload)
        self.update_manifest(lambda data: data["files"].__setitem__(filename, {
            "bytes": path.stat().st_size, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}))

    def test_stored_masks_are_validated_before_bool_conversion(self):
        for name, invalid in (("image_mask", 255), ("depth_mask", 2)):
            with self.subTest(mask=name):
                self.output = self.root / name
                self.saved()
                mask = self.state.views[0][name].astype("u1")
                mask[3, 4] = invalid
                payload = io.BytesIO()
                np.save(payload, mask, allow_pickle=False)
                self.replace_archive(
                    lambda entries: [(key, payload.getvalue() if key == f"v0_{name}.npy" else data)
                                     for key, data in entries], filename="observations.npz")
                with self.assertRaisesRegex(ValueError, "mask"):
                    read_state(self.output)

    def test_state_roundtrip_preserves_parameters_views_and_primary_poses(self):
        restored = read_state(self.saved())
        self.assertEqual(restored.metadata, self.state.metadata)
        for name in self.state.parameters:
            np.testing.assert_array_equal(restored.parameters[name], self.state.parameters[name])
        np.testing.assert_array_equal(restored.origin_ids, [0, 1])
        for before, after in zip(self.state.views, restored.views):
            for key in ("rgb", "image_mask", "depth_mask", "depth_m", "K",
                        "camera_from_body", "camera_from_parent"):
                np.testing.assert_array_equal(after[key], before[key])
                self.assertTrue(after[key].flags.owndata, key)
        self.state.parameters["means"][:] = 999
        self.assertEqual(restored.parameters["means"][0, 2], 4)
        self.assertEqual({p.name for p in self.output.iterdir()},
                         {"manifest.json", "gaussians.npz", "observations.npz"})

    def test_cpu_read_does_not_import_torch(self):
        self.saved()
        script = """
import sys
class BlockGpu:
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('torch', 'rospy', 'cv2'):
            raise AssertionError('Unexpected runtime import: ' + fullname)
sys.meta_path.insert(0, BlockGpu())
from gaussian_state import read_state
state = read_state(sys.argv[1])
assert state.origin_ids.tolist() == [0, 1]
"""
        subprocess.run([sys.executable, "-c", script, str(self.output)],
                       check=True, capture_output=True, text=True)

    def test_system_python_reads_saved_state_without_gpu_modules(self):
        self.saved()
        system_python = Path("/usr/bin/python3")
        if not system_python.is_file():
            self.skipTest("System Python is unavailable")
        script = """
import sys
class BlockGpu:
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('torch', 'rospy', 'rclpy', 'cv2'):
            raise AssertionError('Unexpected runtime import: ' + fullname)
sys.meta_path.insert(0, BlockGpu())
from gaussian_state import read_state
assert read_state(sys.argv[1]).origin_ids.tolist() == [0, 1]
"""
        result = subprocess.run([str(system_python), "-c", script, str(self.output)],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_invalid_metadata_is_rejected_before_creating_output(self):
        for key, value in (("schema", 2), ("optimizer", "adam_pickle"),
                           ("generation", "01"), ("pose_version", True),
                           ("frame_count", "513"), ("origin_session_id", "bad"),
                           ("unknown", 1)):
            with self.subTest(key=key):
                state = copy.deepcopy(self.state)
                state.metadata[key] = value
                with self.assertRaises(ValueError):
                    write_state(self.output, state)
                self.assertFalse(self.output.exists())

    def test_bad_arrays_masks_depth_and_coordinates_are_rejected(self):
        mutations = [
            lambda s: s.parameters.__setitem__("means", np.zeros((2, 3), dtype=np.float64)),
            lambda s: s.parameters["means"].__setitem__((0, 0), np.nan),
            lambda s: s.parameters["quats"].__setitem__((0, 0), 2),
            lambda s: s.parameters["log_scales"].__setitem__((0, 0), 0),
            lambda s: s.parameters.__setitem__("extra", np.zeros(2)),
            lambda s: s.origin_ids.__setitem__(0, 4),
            lambda s: s.views[0]["rgb"].__setitem__((0, 0, 0), 1.1),
            lambda s: s.views[0]["depth_m"].__setitem__((0, 0), 1),
            lambda s: s.views[0]["image_mask"].__setitem__((3, 4), False),
            lambda s: s.views[0]["camera_from_body"].__setitem__((0, 3), 2),
            lambda s: s.metadata["primary"][0]["submap_from_body"].__setitem__(3, 1),
        ]
        for index, change in enumerate(mutations):
            with self.subTest(index=index):
                state = copy.deepcopy(self.state)
                change(state)
                with self.assertRaises(ValueError):
                    validate_state(state)

    def test_model_and_view_budgets_are_enforced(self):
        for count, per_view in ((11, 1), (10, 1001)):
            with self.subTest(count=count, per_view=per_view):
                with self.assertRaises(ValueError):
                    write_state(self.output, make_state(count=count, per_view=per_view))
                self.assertFalse(self.output.exists())

    def test_bad_digest_missing_and_truncated_files_are_rejected(self):
        self.saved()
        path = self.output / "gaussians.npz"
        data = path.read_bytes()
        path.write_bytes(data[:-1])
        with self.assertRaises(ValueError):
            read_state(self.output)
        path.write_bytes(data)
        self.update_manifest(lambda m: m["files"]["gaussians.npz"].__setitem__("sha256", "0" * 64))
        with self.assertRaises(ValueError):
            read_state(self.output)
        path.unlink()
        with self.assertRaises((ValueError, OSError)):
            read_state(self.output)

    def test_npz_layout_and_headers_are_checked_before_loading(self):
        original = io.BytesIO()
        np.lib.format.write_array_header_1_0(original, {
            "descr": "<f4", "fortran_order": False, "shape": (2**40, 3)})
        cases = [
            (lambda entries: entries + entries[:1], zipfile.ZIP_STORED),
            (lambda entries: entries + [("unknown.npy", b"invalid")], zipfile.ZIP_STORED),
            (lambda entries: entries, zipfile.ZIP_DEFLATED),
            (lambda entries: [(name, original.getvalue() if name == "means.npy" else data)
                              for name, data in entries], zipfile.ZIP_STORED),
        ]
        for index, (edit, compression) in enumerate(cases):
            with self.subTest(index=index):
                self.output = self.root / ("layout-" + str(index))
                self.saved()
                self.replace_archive(edit, compression)
                with mock.patch("gaussian_state.np.load", side_effect=AssertionError("allocated")):
                    with self.assertRaises(ValueError):
                        read_state(self.output)

    def test_object_dtype_is_rejected_without_unpickling(self):
        self.saved()
        data = io.BytesIO()
        np.save(data, np.array([object()], dtype=object), allow_pickle=True)
        self.replace_archive(lambda entries: [(name, data.getvalue() if name == "means.npy" else body)
                                               for name, body in entries])
        with mock.patch("gaussian_state.np.load", side_effect=AssertionError("unpickled")):
            with self.assertRaises(ValueError):
                read_state(self.output)

    def test_unknown_and_duplicate_manifest_fields_are_rejected(self):
        self.saved()
        path = self.output / "manifest.json"
        original = path.read_text()
        path.write_text('{"schema": 1,' + original[1:])
        with self.assertRaises(ValueError):
            read_state(self.output)
        path.write_text(original)
        self.update_manifest(lambda m: m["views"][0].__setitem__("unknown", 1))
        with self.assertRaises(ValueError):
            read_state(self.output)

    def test_excessive_zip_directory_is_rejected_before_zipfile(self):
        self.saved()
        path = self.output / "gaussians.npz"
        payload = bytearray(path.read_bytes())
        # An inconsistent central-directory count must not reach ZipInfo allocation.
        struct.pack_into("<HH", payload, len(payload) - 14, 65535, 65535)
        path.write_bytes(payload)
        self.update_manifest(lambda m: m["files"]["gaussians.npz"].__setitem__(
            "sha256", hashlib.sha256(payload).hexdigest()))
        with mock.patch("gaussian_state.zipfile.ZipFile", side_effect=AssertionError("ZIP allocation")):
            with self.assertRaises(ValueError):
                read_state(self.output)

    def test_close_failure_before_commit_does_not_publish(self):
        real_open = Path.open

        class FailingClose:
            def __init__(self, stream):
                self.stream = stream

            def __enter__(self):
                return self.stream.__enter__()

            def __exit__(self, *args):
                self.stream.__exit__(*args)
                raise OSError("Simulated file close failure")

        for filename in ("gaussians.npz", "observations.npz", "manifest.json.pending"):
            with self.subTest(filename=filename):
                self.output = self.root / filename

                def open_file(path, mode="r", *args, **kwargs):
                    stream = real_open(path, mode, *args, **kwargs)
                    return FailingClose(stream) if path.name == filename and mode == "xb" else stream

                with mock.patch("gaussian_state.Path.open", open_file):
                    with self.assertRaises(OSError):
                        write_state(self.output, self.state)
                self.assertFalse((self.output / "manifest.json").exists())

    def test_existing_symlink_and_partial_outputs_are_not_overwritten(self):
        self.output.mkdir()
        marker = self.output / "keep"
        marker.write_text("untouched")
        with self.assertRaises(FileExistsError):
            write_state(self.output, self.state)
        self.assertEqual(marker.read_text(), "untouched")
        with self.assertRaises((ValueError, OSError)):
            read_state(self.output)
        target = self.root / "alias"
        target.symlink_to(self.output, target_is_directory=True)
        with self.assertRaises((ValueError, OSError)):
            write_state(target, self.state)
        self.output = self.root / "valid"
        self.saved()
        path = self.output / "gaussians.npz"
        real = self.root / "outside.npz"
        path.rename(real)
        path.symlink_to(real)
        with self.assertRaises((ValueError, OSError)):
            read_state(self.output)

    def test_failure_before_commit_never_publishes_manifest(self):
        for operation in ("np.savez", "os.fsync", "os.rename"):
            with self.subTest(operation=operation):
                self.output = self.root / operation
                with mock.patch("gaussian_state." + operation, side_effect=OSError(operation)):
                    with self.assertRaises(OSError):
                        write_state(self.output, self.state)
                self.assertTrue(self.output.exists())
                self.assertFalse((self.output / "manifest.json").exists())

    def test_directory_sync_failure_reports_uncertain_readable_commit(self):
        real_sync = os.fsync
        for fail_at in (1, 2):
            with self.subTest(directory_sync=fail_at):
                self.output = self.root / ("dir-sync-" + str(fail_at))
                calls = 0

                def sync(fd):
                    nonlocal calls
                    if stat.S_ISDIR(os.fstat(fd).st_mode):
                        calls += 1
                        if calls == fail_at:
                            raise OSError("Directory sync failed")
                    real_sync(fd)

                with mock.patch("gaussian_state.os.fsync", side_effect=sync):
                    with self.assertRaises(CommitUncertainError):
                        write_state(self.output, self.state)
                restored = read_state(self.output)
                np.testing.assert_array_equal(restored.origin_ids, [0, 1])

    def test_maximum_legal_payload_roundtrip(self):
        state = make_state(count=10, width=1000, height=1000, per_view=1000)
        write_state(self.output, state)
        restored = read_state(self.output)
        self.assertEqual(len(restored.views), 10)
        self.assertEqual(restored.origin_ids.size, 10_000)
        np.testing.assert_array_equal(restored.views[-1]["rgb"], state.views[-1]["rgb"])


if __name__ == "__main__":
    unittest.main()
