import copy
import os
from pathlib import Path
import tempfile
import shutil
from unittest import mock
import unittest

from heldout_fixtures import make_heldout_snapshot, remove_models
from heldout_query import load_evaluation_snapshot
from heldout_selection import EvaluationSelector


class HeldoutSelectionTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        directory, keyframes, _, _ = make_heldout_snapshot(self.root)
        remove_models(directory, {1})
        self.snapshot = load_evaluation_snapshot(directory, keyframes)

    def test_temporal_does_not_jump_over_a_missing_direct_predecessor(self):
        selector = EvaluationSelector(self.snapshot, "temporal")
        frame = dict(self.snapshot.split.frames[1], keyframe_id="5", submap_id="2")
        result = selector.select(frame)
        self.assertEqual(result["selected_ids"], [2])
        self.assertEqual(result["neighbor_reason"], "neighbor_unavailable")

    @unittest.skipUnless(
        os.environ.get("LIVO_SELECTOR_BIN"), "Native selector not configured"
    )
    def test_spatial_reuses_native_ranking_for_independent_camera(self):
        selector = EvaluationSelector(
            self.snapshot, "spatial", Path(os.environ["LIVO_SELECTOR_BIN"])
        )
        frame = self.snapshot.split.frames[1]
        result = selector.select(frame)
        self.assertEqual(result["selected_ids"], [0])
        self.assertEqual(result["neighbor_reason"], "no_spatial_candidate")
        frame = dict(frame, keyframe_id="5", submap_id="2")
        result = selector.select(frame)
        self.assertEqual(result["selected_ids"], [0, 2])
        self.assertEqual(result["selection"]["candidate_count"], 1)
        self.assertEqual(result["selection_policy"], "spatial")

    @unittest.skipUnless(
        os.environ.get("LIVO_SELECTOR_BIN"), "Native selector not configured"
    )
    def test_helper_failure_and_changed_binary_cannot_fall_back(self):
        executable = self.root / "selector"
        executable.write_text("#!/bin/sh\nexit 1\n")
        executable.chmod(0o700)
        selector = EvaluationSelector(self.snapshot, "spatial", executable)
        with self.assertRaises(Exception):
            selector.select(self.snapshot.split.frames[1])
        executable.write_text("#!/bin/sh\nexit 0\n")
        with self.assertRaisesRegex(ValueError, "changed"):
            selector.verify()

    @unittest.skipUnless(
        os.environ.get("LIVO_SHARED_SELECTOR_BIN"),
        "Shared native selector not configured",
    )
    def test_shared_ranking_library_is_part_of_verified_runtime(self):
        binary = Path(os.environ["LIVO_SHARED_SELECTOR_BIN"])
        library = Path(os.environ["LIVO_SHARED_SELECTOR_LIBRARY"])
        copied = self.root / library.name
        shutil.copy2(library, copied)
        env = dict(
            os.environ,
            LD_LIBRARY_PATH=str(self.root)
            + ":"
            + os.environ.get("LD_LIBRARY_PATH", ""),
        )
        with mock.patch.dict(os.environ, env, clear=True):
            selector = EvaluationSelector(self.snapshot, "spatial", binary)
        with copied.open("ab") as stream:
            stream.write(b"changed-ranking-library")
        with self.assertRaisesRegex(ValueError, "changed"):
            selector.verify()

    def test_faulty_helper_cannot_write_past_receipt_file_budget(self):
        marker = self.root / "written-bytes"
        helper = self.root / "oversized-selector"
        helper.write_text(
            '#!/usr/bin/env python3\nimport sys\nfrom pathlib import Path\nout=Path(sys.argv[2])\ntry:\n out.write_bytes(b"x"*262144)\nfinally:\n Path('
            + repr(str(marker))
            + ").write_text(str(out.stat().st_size))\n"
        )
        helper.chmod(0o700)
        selector = EvaluationSelector(self.snapshot, "spatial", helper)
        with self.assertRaises(Exception):
            selector.select(self.snapshot.split.frames[1])
        self.assertLessEqual(int(marker.read_text()), 65536)
