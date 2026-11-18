import copy
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import sys
import unittest

from PIL import Image
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent / "3dgs_probe"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/3dgs_probe"))
from state_fixtures import make_state
from gaussian_state import write_state
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
        (self.on / "parameters.yaml").write_text(yaml.safe_dump({
            "common": {"img_en": 1},
            "gaussian": {"enabled": True, "output_directory": str(self.previews)}}))
        self.previews.mkdir()
        Image.new("RGB", (2, 2), (30, 20, 10)).save(self.previews / "preview-1.png")
        (self.previews / "status.json").write_text(json.dumps({"session_id": "a" * 32}))
        (self.previews / "preview-1.json").write_text(json.dumps({
            "schema": 1, "type": "preview", "session_id": "a" * 32,
            "job_id": "1", "generation": "0", "submap_id": "0",
            "pose_version": "0", "width": 2, "height": 2,
            "nonzero_gradient": 1, "parameter_updated": 1,
            "supervised_depth_pixels": 20, "rgb_l1": 0.1, "depth_l1_m": 0.2}))

    def test_identical_frontend_and_confirmed_online_preview_pass(self):
        result = compare(self.off, self.on, self.previews)
        self.assertTrue(result["passed"], result)
        self.assertEqual(result["raw_files"], 2)
        self.assertEqual(result["first_preview_during_playback_s"], 1.0)

    def test_joint_comparison_keeps_training_on_both_sides_and_requires_online_proof(self):
        for run, enabled in ((self.off, False), (self.on, True)):
            path = run / 'parameters.yaml'
            config = yaml.safe_load(path.read_text())
            config['gaussian'] = dict(enabled=True, persist_submaps=True,
                finalize_on_shutdown=True,
                joint_render_enabled=enabled,
                output_directory=str(self.previews if enabled else self.off / 'previews'))
            path.write_text(yaml.safe_dump(config))
        result = compare(self.off, self.on, self.previews, joint_render=True)
        self.assertFalse(result['passed'])
        self.assertIn('joint_render', result['differences'])
        self.assertNotIn('gaussian_configuration', result['differences'])

    def test_publication_comparison_keeps_both_renderers_on_and_requires_receiver_data(self):
        for run, publish in ((self.off, False), (self.on, True)):
            config = dict(enabled=True, persist_submaps=True, finalize_on_shutdown=True,
                joint_render_enabled=True, joint_render_policy='spatial', publish_preview=publish,
                output_directory=str(self.previews if publish else self.off / 'previews'))
            (run / 'parameters.yaml').write_text(yaml.safe_dump({'gaussian': config}))
        result = compare(self.off, self.on, self.previews, publication=True)
        self.assertFalse(result['passed'])
        self.assertNotIn('gaussian_configuration', result['differences'])
        self.assertIn('gaussian_publication', result['differences'])
        path = self.off / 'parameters.yaml'
        config = yaml.safe_load(path.read_text())
        config['gaussian']['joint_render_enabled'] = False
        path.write_text(yaml.safe_dump(config))
        self.assertIn('gaussian_configuration',
                      compare(self.off, self.on, self.previews, publication=True)['differences'])

    def test_joint_comparison_requires_explicit_finalize_on_both_sides(self):
        for off_finalize, on_finalize in ((False, False), (False, True), (True, False), (None, True)):
            for run, enabled, finalize in ((self.off, False, off_finalize), (self.on, True, on_finalize)):
                config = dict(enabled=True, persist_submaps=True, joint_render_enabled=enabled,
                              output_directory=str(self.previews if enabled else self.off / 'previews'))
                if finalize is not None:
                    config['finalize_on_shutdown'] = finalize
                (run / 'parameters.yaml').write_text(yaml.safe_dump({'gaussian': config}))
            with self.subTest(off=off_finalize, on=on_finalize):
                result = compare(self.off, self.on, self.previews, joint_render=True)
                self.assertIn('Finalize', result['differences'].get('gaussian_configuration', ''))

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

    def test_same_run_cannot_be_both_off_and_on(self):
        result = compare(self.on, self.on, self.previews)
        self.assertFalse(result["passed"])
        self.assertIn("run_directories", result["differences"])

    def test_gaussian_enabled_state_must_match_the_run(self):
        for run, enabled in ((self.off, True), (self.on, False)):
            with self.subTest(run=run):
                path = run / "parameters.yaml"
                original = path.read_text()
                params = yaml.safe_load(original)
                params.setdefault("gaussian", {})["enabled"] = enabled
                path.write_text(yaml.safe_dump(params))
                result = compare(self.off, self.on, self.previews)
                self.assertFalse(result["passed"])
                self.assertIn("gaussian_configuration", result["differences"])
                path.write_text(original)

    def test_preview_directory_must_match_on_configuration(self):
        other = self.previews.parent / "other-previews"
        shutil.copytree(self.previews, other)
        result = compare(self.off, self.on, other)
        self.assertFalse(result["passed"])
        self.assertIn("preview_directory", result["differences"])

    def test_preview_identity_and_image_size_must_match(self):
        path = self.previews / "preview-1.json"
        original = json.loads(path.read_text())
        for key, value in (("session_id", "b" * 32), ("job_id", "2"), ("width", 3)):
            with self.subTest(key=key):
                data = dict(original)
                data[key] = value
                path.write_text(json.dumps(data))
                result = compare(self.off, self.on, self.previews)
                self.assertFalse(result["passed"])
                self.assertIn("preview_files", result["differences"])


class PersistentGaussianReplayTest(unittest.TestCase):
    def setUp(self):
        CompareGaussianReplaysTest.setUp(self)
        state = make_state()
        state.metadata["capture_kind"] = "online_confirmed"
        state.metadata.pop("source_manifest_sha256")
        state.metadata.pop("bundle_manifest_sha256")
        self.states = self.previews / "states"
        self.states.mkdir()
        write_state(self.states / "1", state)
        metadata = state.metadata
        source = {key: copy.deepcopy(metadata[key]) for key in
                  ("generation", "submap_id", "pose_version", "membership_revision",
                   "frame_count", "optimized_frame_count", "anchor_keyframe_id",
                   "submap_state", "parent_frame", "parent_from_submap", "primary")}
        source["primary_count"] = 2
        content = {"session_id": "a" * 32, "submap_id": "0",
                   "source_generation": "1", "content_job_id": "1"}
        asset = dict(content, asset_id="1", manifest_sha256=hashlib.sha256(
            (self.states / "1/manifest.json").read_bytes()).hexdigest())
        for key, name in (("manifest_bytes", "manifest.json"), ("gaussian_bytes", "gaussians.npz"),
                          ("observation_bytes", "observations.npz")):
            asset[key] = str((self.states / "1" / name).stat().st_size)
        self.entry = {"submap_id": "0", "anchor_keyframe_id": "0", "primary_count": 2,
                      "membership_revision": "2", "submap_state": "active",
                      "availability": "current", "reason": "", "resident": 1,
                      "binding": {key: value for key, value in source.items() if key != "primary"},
                      "binding_job_id": "1", "content": content, "volatile": 0,
                      "content_source": copy.deepcopy(source), "trained_frame_ids": ["0", "1"],
                      "gaussian_count": 2, "asset": asset, "asset_source": copy.deepcopy(source),
                      "asset_trained_frame_ids": ["0", "1"], "asset_gaussian_count": 2,
                      "asset_current": 1}
        self.map = {"schema": 1, "kind": "livo_atlas.gaussian_map", "session_id": "a" * 32,
                    "map_sequence": "1", "pose_version": "0", "frame_count": "2",
                    "optimized_frame_count": "0", "parent_frame": "odom",
                    "written_at_unix_seconds": 1.0, "entries": [self.entry]}
        self.save_map()
        params = yaml.safe_load((self.on / "parameters.yaml").read_text())
        params["gaussian"].update(persist_submaps=True, shutdown_timeout_ms=30000)
        (self.on / "parameters.yaml").write_text(yaml.safe_dump(params))
        self.poses = []
        for local in metadata["primary"]:
            import numpy as np
            parent = np.asarray(metadata["parent_from_submap"]).reshape(4, 4)
            pose = parent @ np.asarray(local["submap_from_body"]).reshape(4, 4)
            self.poses.append(pose)
        for run in (self.off, self.on):
            for i, pose in enumerate(self.poses):
                path = run / "keyframes/frames" / str(i) / "frame.yml"
                path.parent.mkdir(exist_ok=True)
                path.write_text(yaml.safe_dump({"id": str(i), "odom_from_body": self.transform(pose)}))
            (run / "mapping.log").write_text(
                "livo_atlas loops: frames=2 pose_version=0 accepted=0\n")
        with (self.on / "mapping.log").open("a") as log:
            log.write("livo_atlas gaussian: observed=2 submitted=2 completed=2 accepted=2 "
                      "failed=0 peak_pending=1 peak_in_flight=1\n")
        self.submap_stats()
        path = self.previews / "preview-1.json"
        data = json.loads(path.read_text())
        data.update(generation="1", operation="view", steps=20, frame_ids=["0", "1"],
                    gaussian_count=2, width=8, height=6)
        path.write_text(json.dumps(data))
        Image.new("RGB", (8, 6), (30, 20, 10)).save(self.previews / "preview-1.png")

    @staticmethod
    def transform(pose):
        return {"rotation": pose[:3, :3].ravel().tolist(),
                "translation": pose[:3, 3].tolist()}

    def save_map(self):
        (self.previews / "map.json").write_text(json.dumps(self.map))

    def submap_stats(self, incomplete=0, needs_rebuild=0, rebinds=0, effective_training=2):
        with (self.on / "mapping.log").open("a") as log:
            log.write("livo_atlas gaussian submaps: checkpoints=1 loads=0 rebinds=%d rebuilds=0 "
                      "prunes=0 map_sequence=1 indexed=1 missing=0 needs_rebuild=%d dirty=0 "
                      "asset_bytes=%d incomplete=%d effective_training=%d\n" % (rebinds, needs_rebuild,
                      sum(int(self.entry["asset"][key]) for key in
                          ("manifest_bytes", "gaussian_bytes", "observation_bytes")), incomplete, effective_training))

    def test_verified_persistent_map_passes(self):
        result = compare(self.off, self.on, self.previews)
        self.assertTrue(result["passed"], result)
        self.assertEqual(result["gaussian_map"]["verified_assets"], 1)

    def advance_frontier(self):
        for run in (self.off, self.on):
            path = run / "keyframes/frames/2/frame.yml"
            path.parent.mkdir()
            path.write_text(yaml.safe_dump({"id": "2", "odom_from_body": self.transform(self.poses[-1])}))
            with (run / "mapping.log").open("a") as log:
                log.write("livo_atlas loops: frames=3 pose_version=0 accepted=0\n")
        self.map["frame_count"] = "3"
        self.entry["binding"]["frame_count"] = "3"

    def test_map_rejects_omitted_final_group(self):
        self.advance_frontier()
        for target in (self.entry, self.entry["binding"]):
            target.update(submap_state="sealed", membership_revision="3")
        self.save_map()
        self.assertFalse(compare(self.off, self.on, self.previews)["passed"])

    def test_map_rejects_truncated_final_group(self):
        self.advance_frontier()
        self.save_map()
        self.assertFalse(compare(self.off, self.on, self.previews)["passed"])

    def test_older_asset_coverage_can_have_complete_logical_tail(self):
        self.advance_frontier()
        for target in (self.entry, self.entry["binding"]):
            target.update(primary_count=3, membership_revision="3")
        self.save_map()
        result = compare(self.off, self.on, self.previews)
        self.assertTrue(result["passed"], result)

    def test_map_grouping_must_match_authoritative_records(self):
        folder = self.on / "keyframes/online_submaps/records/0"
        folder.mkdir(parents=True)
        record = {"schema_version": 1, "id": "0", "membership_revision": "3",
                  "state": "sealed", "anchor_keyframe_id": "0", "keyframe_ids": ["0", "1"],
                  "support_keyframe_ids": []}
        (folder / "3.yml").write_text(yaml.safe_dump(record))
        self.assertFalse(compare(self.off, self.on, self.previews)["passed"])

    def test_reuse_does_not_count_as_training(self):
        path = self.previews / "preview-1.json"
        data = json.loads(path.read_text())
        data.update(operation="reuse", steps=0)
        path.write_text(json.dumps(data))
        result = compare(self.off, self.on, self.previews)
        self.assertEqual(result["previews"]["trained"], 0)
        self.assertFalse(result["passed"])

    def test_accepted_ineffective_training_cannot_pass(self):
        path = self.previews / "preview-1.json"
        data = json.loads(path.read_text())
        data.update(nonzero_gradient=0, parameter_updated=0, supervised_depth_pixels=0)
        path.write_text(json.dumps(data))
        self.submap_stats(effective_training=0)
        result = compare(self.off, self.on, self.previews)
        self.assertEqual(result["previews"]["trained"], 0)
        self.assertFalse(result["passed"])

    def test_readonly_reuse_uses_verified_earlier_training(self):
        path = self.previews / "preview-1.json"
        data = json.loads(path.read_text())
        data.update(job_id="3", generation="2", operation="reuse", steps=0,
                    nonzero_gradient=0, parameter_updated=0, supervised_depth_pixels=0)
        (self.previews / "preview-3.json").write_text(json.dumps(data))
        path.unlink()
        (self.previews / "preview-1.png").rename(self.previews / "preview-3.png")
        self.entry["binding"].update(generation="2")
        self.entry["binding_job_id"] = "3"
        self.save_map()
        with (self.on / "mapping.log").open("a") as log:
            log.write("livo_atlas gaussian: observed=2 submitted=3 completed=3 accepted=3 "
                      "failed=0 peak_pending=1 peak_in_flight=1\n")
        self.submap_stats(rebinds=1)
        result = compare(self.off, self.on, self.previews)
        self.assertTrue(result["passed"], result)
        self.assertEqual(result["previews"]["trained"], 0)
        self.assertEqual(result["confirmed_training_operations"], 2)
        self.submap_stats(rebinds=1, effective_training=0)
        self.assertFalse(compare(self.off, self.on, self.previews)["passed"])
        self.submap_stats(rebinds=3)
        self.assertFalse(compare(self.off, self.on, self.previews)["passed"])

    def test_map_rejects_wrong_session_or_binding(self):
        for change in (lambda: self.map.update(session_id="b" * 32),
                       lambda: self.entry["binding"].update(pose_version="1"),
                       lambda: self.entry["binding"]["parent_from_submap"].__setitem__(3, 9)):
            original = copy.deepcopy(self.map)
            change()
            self.save_map()
            self.assertFalse(compare(self.off, self.on, self.previews)["passed"])
            self.map = original
            self.entry = self.map["entries"][0]

    def test_map_requires_verified_assets(self):
        self.entry["asset"]["manifest_sha256"] = "0" * 64
        self.save_map()
        self.assertFalse(compare(self.off, self.on, self.previews)["passed"])
        (self.states / "1/manifest.json").unlink()
        self.assertFalse(compare(self.off, self.on, self.previews)["passed"])

    def test_map_enforces_asset_budget(self):
        for i in range(2, 35):
            (self.states / str(i)).mkdir()
        result = compare(self.off, self.on, self.previews)
        self.assertFalse(result["passed"])
        self.assertIn("budget", result["gaussian_map"]["error"].lower())
        for i in range(2, 35):
            (self.states / str(i)).rmdir()
        self.map["entries"] = [copy.deepcopy(self.entry) for _ in range(33)]
        self.save_map()
        result = compare(self.off, self.on, self.previews)
        self.assertFalse(result["passed"])
        self.assertIn("budget", result["gaussian_map"]["error"].lower())

    def test_volatile_content_is_not_durable(self):
        self.entry["content"]["content_job_id"] = "2"
        self.entry["binding_job_id"] = "2"
        self.entry["volatile"] = 1
        self.save_map()
        result = compare(self.off, self.on, self.previews)
        self.assertFalse(result["passed"])
        self.assertEqual(result["gaussian_map"]["volatile"], 1)

    def deform(self):
        poses = [value.copy() for value in self.poses]
        poses[1][0, 3] += .1
        for run in (self.off, self.on):
            path = run / "keyframes/online_submaps/poses/1.yml"
            path.parent.mkdir(parents=True)
            path.write_text(yaml.safe_dump({"schema_version": 2, "version": "1",
                "base_version": "0", "parent_frame": "map", "frame_count": "2",
                "optimized_frame_count": "2", "tail_policy": "last_optimized_anchor_projected",
                "tail_anchor_keyframe_id": "1", "poses": [
                    {"id": str(i), "map_from_body": self.transform(pose)} for i, pose in enumerate(poses)]}))
            with (run / "mapping.log").open("a") as log:
                log.write("livo_atlas loops: frames=2 pose_version=1 accepted=1\n")
        self.map.update(pose_version="1", optimized_frame_count="2", parent_frame="map")
        self.entry["binding"].update(pose_version="1", optimized_frame_count="2", parent_frame="map")
        self.entry["asset_current"] = 0
        self.save_map()

    def test_nonrigid_content_cannot_be_current(self):
        self.deform()
        self.assertFalse(compare(self.off, self.on, self.previews)["passed"])

    def test_explicit_historical_asset_remains_a_rebuild_input(self):
        self.deform()
        self.entry["availability"] = "needs_rebuild"
        self.entry["reason"] = "primary_pose_changed"
        self.save_map()
        self.submap_stats(needs_rebuild=1)
        result = compare(self.off, self.on, self.previews)
        self.assertTrue(result["passed"], result)
        self.assertEqual(result["gaussian_map"]["needs_rebuild"], 1)

    def test_incomplete_shutdown_fails_acceptance(self):
        self.submap_stats(incomplete=1)
        self.assertFalse(compare(self.off, self.on, self.previews)["passed"])


class FinalizedGaussianReplayTest(unittest.TestCase):
    transform = staticmethod(PersistentGaussianReplayTest.transform)
    save_map = PersistentGaussianReplayTest.save_map
    submap_stats = PersistentGaussianReplayTest.submap_stats

    def setUp(self):
        PersistentGaussianReplayTest.setUp(self)
        params = yaml.safe_load((self.on / 'parameters.yaml').read_text())
        params['gaussian']['finalize_on_shutdown'] = True
        (self.on / 'parameters.yaml').write_text(yaml.safe_dump(params))
        for target in (self.entry, self.entry['binding']):
            target.update(submap_state='sealed', membership_revision='3')
        self.save_map()
        self.report = {'schema': 1, 'kind': 'livo_atlas.gaussian_finalization',
                       'status': 'succeeded', 'reason': '', 'session_id': 'a'*32,
                       'pose_version': '0', 'frame_count': '2', 'optimized_frame_count': '0',
                       'parent_frame': 'odom', 'map_sequence': '1',
                       'target_valid': 1, 'eligible_set_frozen': 1, 'durable': 1, 'worker_stopped': 1,
                       'eligible_submap_ids': ['0'], 'asset_ids': ['1'],
                       'eligible': 1, 'current': 1, 'needs_rebuild': 0, 'missing': 0, 'dirty': 0,
                       'repair_attempts': '0', 'elapsed_seconds': .2, 'repair_seconds': .1,
                       'shutdown_seconds': .1,
                       'before': {'accepted': '2', 'effective_training': '2',
                                  'rebuilds': '0', 'checkpoints': '0'},
                       'after': {'accepted': '2', 'effective_training': '2',
                                 'rebuilds': '0', 'checkpoints': '1'}}
        self.save_report()
        summary = json.loads((self.on / 'summary.json').read_text())
        summary.update(forced_termination=False, node_returncode_after_sigint=0)
        (self.on / 'summary.json').write_text(json.dumps(summary))
        self.final_log = ('livo_atlas gaussian finalization: status=succeeded session_id=' + 'a'*32 +
            ' pose_version=0 frame_count=2 optimized_frame_count=0 parent_frame=odom map_sequence=1'
            ' target_valid=1 eligible_set_frozen=1 eligible_submap_ids=0 asset_ids=1 eligible=1'
            ' current=1 needs_rebuild=0 missing=0 dirty=0 durable=1 worker_stopped=1 report_persisted=1'
            ' repair_attempts=0 before_accepted=2 before_effective_training=2 before_rebuilds=0'
            ' before_checkpoints=0 after_accepted=2 after_effective_training=2 after_rebuilds=0'
            ' after_checkpoints=1 elapsed_seconds=0.2 repair_seconds=0.1 shutdown_seconds=0.1'
            ' total_seconds=0.21\n')
        with (self.on / 'mapping.log').open('a') as log:
            log.write(self.final_log)

    def save_report(self):
        (self.previews / 'finalization.json').write_text(json.dumps(self.report))

    def test_verified_finalization_passes(self):
        result = compare(self.off, self.on, self.previews)
        self.assertTrue(result['passed'], result)
        self.assertEqual(result['gaussian_finalization']['current'], 1)

    def test_missing_report_is_rejected(self):
        (self.previews / 'finalization.json').unlink()
        self.assertFalse(compare(self.off, self.on, self.previews)['passed'])

    def test_joint_comparison_validates_the_off_finalization_independently(self):
        off_previews = self.off / 'previews'
        shutil.copytree(self.previews, off_previews)
        for name in ('summary.json', 'mapping.log'):
            shutil.copyfile(self.on / name, self.off / name)
        for run, enabled, previews in ((self.off, False, off_previews),
                                       (self.on, True, self.previews)):
            path = run / 'parameters.yaml'
            config = yaml.safe_load((self.on / 'parameters.yaml').read_text())
            config['gaussian'].update(joint_render_enabled=enabled, output_directory=str(previews))
            path.write_text(yaml.safe_dump(config))
        baseline = compare(self.off, self.on, self.previews, joint_render=True)
        self.assertNotIn('off_gaussian_map', baseline['differences'])
        self.assertNotIn('gaussian_finalization', baseline['differences'])
        (off_previews / 'finalization.json').unlink()
        result = compare(self.off, self.on, self.previews, joint_render=True)
        self.assertIn('off_gaussian_map', result['differences'])

    def test_forged_report_cannot_replace_map_and_stop_evidence(self):
        changes = [('status', 'timed_out'), ('session_id', 'b'*32), ('frame_count', '3'),
                   ('pose_version', '1'), ('optimized_frame_count', '1'), ('parent_frame', 'map'),
                   ('map_sequence', '2'), ('eligible_submap_ids', []), ('eligible_submap_ids', ['0','0']),
                   ('asset_ids', ['2']), ('eligible', 2), ('current', 0), ('needs_rebuild', 1),
                   ('dirty', 1), ('missing', 1), ('worker_stopped', 0), ('durable', 0),
                   ('eligible_set_frozen', 0), ('target_valid', 0), ('elapsed_seconds', float('nan')),
                   ('before', dict(self.report['before'], accepted='3'))]
        original = copy.deepcopy(self.report)
        for key, value in changes:
            with self.subTest(key=key, value=value):
                self.report = dict(original, **{key: value})
                self.save_report()
                self.assertFalse(compare(self.off, self.on, self.previews)['passed'])
        self.report = original

    def test_publication_and_exit_failures_override_visible_succeeded(self):
        path = self.on / 'mapping.log'
        original = path.read_text()
        for before, after in [('report_persisted=1', 'report_persisted=0'),
                              ('worker_stopped=1', 'worker_stopped=0'),
                              ('session_id='+'a'*32, 'session_id='+'b'*32),
                              ('asset_ids=1', 'asset_ids=2'),
                              ('before_accepted=2', 'before_accepted=1')]:
            with self.subTest(after=after):
                path.write_text(original.replace(before, after))
                self.assertFalse(compare(self.off, self.on, self.previews)['passed'])
        path.write_text(original)
        summary_path = self.on / 'summary.json'
        original_summary = json.loads(summary_path.read_text())
        for key, value in [('forced_termination', True), ('node_returncode_after_sigint', -15)]:
            summary_path.write_text(json.dumps(dict(original_summary, **{key: value})))
            self.assertFalse(compare(self.off, self.on, self.previews)['passed'])

    def test_missing_group_is_explicit_without_inventing_content(self):
        from gaussian_map import inspect_gaussian_finalization, inspect_gaussian_map
        PersistentGaussianReplayTest.advance_frontier(self)
        binding = dict(self.entry['binding'], submap_id='1', anchor_keyframe_id='2',
                       primary_count=1, membership_revision='2', submap_state='sealed',
                       parent_from_submap=self.poses[-1].ravel().tolist())
        self.map['entries'].append(dict(binding=binding, submap_id='1', anchor_keyframe_id='2',
            primary_count=1, membership_revision='2', submap_state='sealed', availability='missing',
            reason='no_image', resident=0))
        self.save_map()
        self.report.update(frame_count='3', missing=1)
        self.save_report()
        actual = inspect_gaussian_map(self.previews, self.on / 'keyframes', 0, 3)
        final = inspect_gaussian_finalization(self.previews, actual)
        self.assertEqual(final['missing'], 1)
        self.assertEqual(final['eligible_submap_ids'], ['0'])

    def test_report_is_bounded_and_cannot_self_certify_publication(self):
        path = self.previews / 'finalization.json'
        path.write_bytes(b' ' * (16 * 1024 + 1))
        self.assertFalse(compare(self.off, self.on, self.previews)['passed'])
        self.report['report_persisted'] = True
        self.save_report()
        log = self.on / 'mapping.log'
        log.write_text(log.read_text().replace('report_persisted=1', 'report_persisted=0'))
        self.assertFalse(compare(self.off, self.on, self.previews)['passed'])

    def test_historical_content_cannot_be_finalized_by_sidecar(self):
        PersistentGaussianReplayTest.deform(self)
        self.entry['availability'] = 'needs_rebuild'
        self.entry['reason'] = 'primary_pose_changed'
        self.save_map()
        self.submap_stats(needs_rebuild=1)
        self.report.update(pose_version='1', optimized_frame_count='2', parent_frame='map')
        self.save_report()
        self.assertFalse(compare(self.off, self.on, self.previews)['passed'])


if __name__ == "__main__":
    unittest.main()
