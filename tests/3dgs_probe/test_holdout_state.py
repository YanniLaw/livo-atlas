import copy
import tempfile
from pathlib import Path
import unittest

from gaussian_state import read_state, validate_state, write_state
from holdout_split import HoldoutSplit, canonical_json
from state_fixtures import make_state
from test_holdout_split import sample_manifest, sample_view


def proven_state():
    split = HoldoutSplit(canonical_json(sample_manifest()))
    state = make_state(1)
    state.metadata = make_state(12).metadata
    state.views = [dict(sample_view(0), camera_from_parent=sample_view(0)['camera_from_body'])]
    state.metadata.update(schema=2, holdout=split.proof([0, 2, 3]))
    return split, state


class HoldoutStateTest(unittest.TestCase):
    def test_versioned_state_roundtrip_and_historical_lineage(self):
        split, state = proven_state()
        validate_state(state)
        split.check_state(state)
        with tempfile.TemporaryDirectory() as root:
            output = Path(root) / 'state'
            write_state(output, state)
            restored = read_state(output)
            split.check_state(restored)
            self.assertEqual(restored.metadata['holdout'], state.metadata['holdout'])
            self.assertEqual(len(restored.metadata['holdout']['lineage']), 3)

    def test_legacy_and_unproven_state_are_rejected(self):
        split, state = proven_state()
        for change in ('legacy', 'wrong_split', 'missing_origin', 'heldout_history'):
            bad = copy.deepcopy(state)
            if change == 'legacy':
                bad.metadata.pop('holdout')
                bad.metadata['schema'] = 1
            elif change == 'wrong_split':
                bad.metadata['holdout']['split_id'] = 'f' * 64
            elif change == 'missing_origin':
                bad.origin_ids[:] = 4
            else:
                bad.metadata['holdout']['lineage'][1]['keyframe_id'] = '1'
            with self.subTest(change=change), self.assertRaises(ValueError):
                split.check_state(bad)

    def test_schema_one_rejects_new_proof_fields(self):
        _, state = proven_state()
        state.metadata['schema'] = 1
        with self.assertRaises(ValueError):
            validate_state(state)


if __name__ == '__main__':
    unittest.main()
