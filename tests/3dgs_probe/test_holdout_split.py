import copy
import json
import hashlib
import unittest

import numpy as np

from holdout_split import (HoldoutSplit, build_manifest, canonical_json,
                           observation_record)


def sample_view(key):
    return dict(keyframe_id=key, calibration_id=0, state_timestamp=100.0 + key,
                cloud_timestamp=100.0 + key, image_timestamp=100.01 + key,
                inverse_exposure_time=1.0, K=np.eye(3), camera_from_body=np.eye(4),
                rgb=np.full((6, 8, 3), key / 255, dtype=np.float32),
                image_mask=np.ones((6, 8), dtype=bool),
                depth_mask=np.ones((6, 8), dtype=bool),
                depth_m=np.full((6, 8), 2, dtype=np.float32))


def sample_manifest(count=12):
    source = dict(dataset_sha256='a' * 64, camera_sha256='b' * 64,
                  input_sha256='c' * 64, preparer_sha256='d' * 64,
                  frontend_profile='noetic-test')
    source['assets'] = [dict(keyframe_id=str(key), image_sha256=hashlib.sha256(str(key).encode()).hexdigest(),
                             cloud_sha256='b' * 64, record_sha256='c' * 64)
                        for key in range(count)]
    frames = [observation_record(sample_view(key), '0',
                                'heldout' if key % 8 == 1 else 'train')
              for key in range(count)]
    return build_manifest(source, frames)


class HoldoutSplitTest(unittest.TestCase):
    def test_canonical_identity_and_predeclared_queries(self):
        manifest = sample_manifest()
        split = HoldoutSplit(canonical_json(manifest))
        self.assertEqual(split.split_id, manifest['split_id'])
        self.assertEqual([int(f['keyframe_id']) for f in manifest['frames'] if f['query']], [1, 9])
        self.assertEqual(split.check_view(sample_view(0)), 0)
        for key in (1, 9, 12):
            with self.assertRaises(ValueError):
                split.check_view(sample_view(key))

    def test_modified_input_is_rejected(self):
        split = HoldoutSplit(canonical_json(sample_manifest()))
        for field in ('rgb', 'depth_m', 'image_mask', 'depth_mask', 'K',
                      'camera_from_body', 'image_timestamp', 'calibration_id'):
            view = sample_view(0)
            if isinstance(view[field], np.ndarray):
                view[field].flat[0] = 0 if view[field].dtype == bool else 0.5
            else:
                view[field] += 1
            with self.subTest(field=field), self.assertRaises(ValueError):
                split.check_view(view)

    def test_tampered_manifest_and_duplicates_are_rejected(self):
        manifest = sample_manifest()
        manifest['frames'][1]['role'] = 'train'
        with self.assertRaises(ValueError):
            HoldoutSplit(canonical_json(manifest))
        for field in ('image_timestamp', 'rgb_sha256'):
            original = sample_manifest()
            frames = copy.deepcopy(original['frames'])
            frames[1][field] = frames[0][field]
            with self.subTest(field=field), self.assertRaises(ValueError):
                build_manifest(original['source'], frames)

    def test_lineage_is_complete_sorted_and_bound_to_split(self):
        split = HoldoutSplit(canonical_json(sample_manifest()))
        proof = split.proof([0, 2, 3])
        split.check_proof(proof)
        for change in ('split_id', 'lineage_sha256'):
            bad = copy.deepcopy(proof)
            bad[change] = 'f' * 64
            with self.assertRaises(ValueError):
                split.check_proof(bad)
        with self.assertRaises(ValueError):
            split.proof([0, 1])
        bad = copy.deepcopy(proof)
        bad['lineage'][1]['observation_sha256'] = 'f' * 64
        with self.assertRaises(ValueError):
            split.check_proof(bad)

    def test_limits_and_incomplete_prefix(self):
        with self.assertRaises(ValueError):
            HoldoutSplit(b'x' * (1024 * 1024 + 1))
        original = sample_manifest()
        with self.assertRaises(ValueError):
            build_manifest(original['source'], original['frames'][1:])
        with self.assertRaises(ValueError):
            HoldoutSplit(json.dumps(original).replace('"schema": 1', '"schema": true').encode())

    def test_subpixel_rgb_changes_and_group_changes_are_rejected(self):
        split = HoldoutSplit(canonical_json(sample_manifest()))
        view = sample_view(0)
        view['rgb'].flat[0] += 0.00001
        with self.assertRaises(ValueError):
            split.check_view(view)
        view = dict(sample_view(0), submap_id='1')
        with self.assertRaises(ValueError):
            split.check_view(view)


if __name__ == '__main__':
    unittest.main()
