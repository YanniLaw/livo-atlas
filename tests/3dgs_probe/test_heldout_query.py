import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from heldout_fixtures import make_heldout_snapshot, replace_asset, remove_models
from gaussian_state import read_state
from holdout_split import canonical_json, build_manifest
from heldout_query import load_evaluation_snapshot, prepare_query, render_query, verify_evaluation_snapshot
from joint_renderer import load_snapshot
from joint_fixtures import rigid_update


class HeldoutQueryTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory, self.keyframes, self.split, self.views = make_heldout_snapshot(Path(self.temporary.name))

    def test_independent_heldout_camera_does_not_relax_retained_preview(self):
        with self.assertRaises(ValueError):
            load_snapshot(self.directory, self.keyframes, 1, [0])
        snapshot = load_evaluation_snapshot(self.directory, self.keyframes)
        query, owner, camera = prepare_query(snapshot, self.views[1])
        self.assertEqual(owner, 0)
        np.testing.assert_allclose(camera, query['camera_from_body'] @ np.linalg.inv(snapshot.poses[1]))
        for key in (0, 99):
            view = copy.deepcopy(self.views[0])
            view['keyframe_id'] = key
            with self.assertRaises(ValueError):
                prepare_query(snapshot, view)

    def test_changed_query_and_source_are_rejected(self):
        snapshot = load_evaluation_snapshot(self.directory, self.keyframes)
        bad = copy.deepcopy(self.views[1])
        bad['rgb'].flat[0] += .01
        with self.assertRaises(ValueError):
            prepare_query(snapshot, bad)
        (self.keyframes / 'frames/1/image.png').write_bytes(b'changed')
        with self.assertRaises(ValueError):
            verify_evaluation_snapshot(snapshot)

    def test_final_partition_must_match_split_even_when_every_model_is_missing(self):
        remove_models(self.directory, {0, 1, 2})
        frames = [dict(frame) for frame in self.split.frames.values()]
        frames[1]['submap_id'] = '1'
        bad = build_manifest(self.split.source, frames)
        (self.directory/'split.json').write_bytes(canonical_json(bad))
        with self.assertRaises(ValueError):
            load_evaluation_snapshot(self.directory, self.keyframes)

    def test_reference_precision_and_camera_are_canonical_owned_values(self):
        snapshot = load_evaluation_snapshot(self.directory, self.keyframes)
        view = copy.deepcopy(self.views[1])
        view['rgb'] = view['rgb'].astype('f8') + 1e-11
        view['depth_m'] = view['depth_m'].astype('f8')
        view['depth_m'][view['depth_mask']] += 1e-11
        view['camera_from_body'][0, 2] += 1e-11
        query, _, _ = prepare_query(snapshot, view)
        self.assertEqual(query['rgb'].dtype, np.dtype('f4'))
        self.assertEqual(query['depth_m'].dtype, np.dtype('f4'))
        np.testing.assert_array_equal(query['rgb'], self.views[1]['rgb'])
        self.assertFalse(np.shares_memory(query['rgb'], view['rgb']))
        np.testing.assert_array_equal(query['camera_from_body'].ravel(), self.split.frames[1]['camera_from_body'])

    def test_rigid_correction_rebinds_camera_without_changing_reference(self):
        raw = load_evaluation_snapshot(self.directory, self.keyframes)
        query_before, _, camera_before = prepare_query(raw, self.views[1])
        rigid_update(self.directory, self.keyframes)
        corrected = load_evaluation_snapshot(self.directory, self.keyframes)
        query_after, _, camera_after = prepare_query(corrected, self.views[1])
        np.testing.assert_array_equal(query_before['rgb'], query_after['rgb'])
        np.testing.assert_array_equal(query_before['depth_m'], query_after['depth_m'])
        self.assertFalse(np.allclose(camera_before, camera_after))
        for group in (0, 2):
            before = np.asarray(raw.document['entries'][group]['binding']['parent_from_submap']).reshape(4, 4)
            after = np.asarray(corrected.document['entries'][group]['binding']['parent_from_submap']).reshape(4, 4)
            np.testing.assert_allclose(camera_before @ before, camera_after @ after, atol=1e-12)

    def test_legacy_and_contaminated_histories_fail_even_with_updated_file_hashes(self):
        path = self.directory / 'map.json'
        original = json.loads(path.read_text())
        state = read_state(self.directory / 'states/1')
        for change in ('legacy', 'wrong_split', 'heldout_history'):
            document = copy.deepcopy(original)
            bad = copy.deepcopy(state)
            if change == 'legacy':
                bad.metadata.update(schema=1)
                bad.metadata.pop('holdout')
            elif change == 'wrong_split':
                bad.metadata['holdout']['split_id'] = 'f' * 64
            else:
                proof = bad.metadata['holdout']
                proof['lineage'].append(dict(keyframe_id='1', observation_sha256=self.split.frames[1]['observation_sha256']))
                proof['lineage_sha256'] = hashlib.sha256(canonical_json(proof['lineage'])).hexdigest()
            replace_asset(self.directory, document['entries'][0], bad)
            path.write_text(json.dumps(document))
            with self.subTest(change=change), self.assertRaises(ValueError):
                load_evaluation_snapshot(self.directory, self.keyframes)

    def test_legacy_asset_cannot_be_evaluated_as_proven(self):
        (self.directory / 'split.json').unlink()
        with self.assertRaises((ValueError, FileNotFoundError)):
            load_evaluation_snapshot(self.directory, self.keyframes)

    def test_nonadjacent_joint_query_has_explicit_content_and_camera(self):
        snapshot = load_evaluation_snapshot(self.directory, self.keyframes)
        received = []

        def render(blocks, K, width, height, device):
            received.append([block[0] for block in blocks])
            return dict(rgb=self.views[1]['rgb'].copy(), alpha=np.ones((height, width), 'f4'),
                        depth_m=self.views[1]['depth_m'].copy(), gaussian_count=len(blocks))

        output = render_query(snapshot, self.views[1], [0, 2], 'test', renderer=render)
        self.assertEqual(received, [[0], [0, 2]])
        self.assertEqual(output['report']['phase'], 'final_snapshot')
        self.assertEqual(output['report']['selected_ids'], [0, 2])
        self.assertTrue(output['report']['single']['rgb_perfect'])
        self.assertTrue(output['report']['joint']['rgb_perfect'])
        self.assertEqual(output['report']['as_of']['map_sequence'], '1')
        self.assertEqual([s['asset_id'] for s in output['report']['sources']], ['1', '3'])
        binding = copy.deepcopy(snapshot.document['entries'][0]['binding'])
        output['report']['sources'][0]['binding']['parent_from_submap'][3] += 1
        self.assertEqual(snapshot.document['entries'][0]['binding'], binding)
        verify_evaluation_snapshot(snapshot)

    def test_invalid_selection_and_inference_mutation_do_not_pass(self):
        snapshot = load_evaluation_snapshot(self.directory, self.keyframes)
        for ids in ([1], [0, 0], [0, 1, 2], [0, 99]):
            with self.subTest(ids=ids), self.assertRaises(ValueError):
                render_query(snapshot, self.views[1], ids, 'test', renderer=lambda *args: None)

        def changing_renderer(*args):
            (self.directory / 'status.json').write_text('{}')
            return dict(rgb=self.views[1]['rgb'], alpha=np.ones((6, 8), 'f4'), depth_m=self.views[1]['depth_m'])

        with self.assertRaises(ValueError):
            render_query(snapshot, self.views[1], [0], 'test', renderer=changing_renderer)


if __name__ == '__main__':
    unittest.main()
