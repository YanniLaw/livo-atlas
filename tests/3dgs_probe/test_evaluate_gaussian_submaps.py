import copy
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

import numpy as np

from evaluate_gaussian_submaps import (camera_from_final_parent,
    check_split_identity, prepare_group, read_prepared_group, split_primary_ids)


class HoldoutPreparationTest(unittest.TestCase):
    def test_split_excludes_held_out_ids(self):
        self.assertEqual(split_primary_ids(168, 4), ([168, 170, 171], [169]))
        train, held = split_primary_ids(0, 18)
        self.assertEqual(held, [1, 9])
        self.assertEqual(train, [0, 2, 3, 4, 5, 6, 7, 8, 10,
                                 11, 12, 13, 14, 15, 16, 17])
        for first, count in ((4, 2), (0, 33), (510, 3)):
            with self.subTest(first=first, count=count), self.assertRaises(ValueError):
                split_primary_ids(first, count)

    def test_rebind_camera_after_loop(self):
        raw_pose = np.array([[0., 1, 0, 4], [-1, 0, 0, 5], [0, 0, 1, 6], [0, 0, 0, 1]])
        raw_camera = np.array([[-1., 0, 0, 5], [0, -1, 0, 7], [0, 0, 1, -3], [0, 0, 0, 1]])
        final_pose = np.array([[1., 0, 0, 7], [0, 0, -1, 8], [0, 1, 0, 9], [0, 0, 0, 1]])
        expected = np.array([[0., 0, -1, 10], [1, 0, 0, -5], [0, -1, 0, 11], [0, 0, 0, 1]])
        view = dict(parent_from_body=raw_pose, camera_from_parent=raw_camera,
                    rgb=np.full((2, 2, 3), .5), depth_m=np.ones((2, 2)))
        original = copy.deepcopy(view)
        actual = camera_from_final_parent(view, final_pose)
        np.testing.assert_allclose(actual, expected, atol=1e-12, rtol=0)
        self.assertEqual(actual.dtype, np.dtype('float64'))
        for key in view:
            np.testing.assert_array_equal(view[key], original[key])

    def test_input_collisions_and_bundle_mismatches_are_rejected(self):
        observations = [dict(keyframe_id=0, split='train', capture_time=1., image_sha256='a' * 64),
                        dict(keyframe_id=5, split='validation', capture_time=2., image_sha256='b' * 64)]
        check_split_identity(observations)
        for field in ('capture_time', 'image_sha256'):
            invalid = copy.deepcopy(observations)
            invalid[1][field] = invalid[0][field]
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, '0.*5'):
                check_split_identity(invalid)

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            keyframes, bundle = root / 'dataset', root / 'bundle'
            subprocess.run([os.environ['LIVO_FIXTURE_BIN'], str(keyframes)], check=True)
            group = dict(submap_id=0, anchor_id=0, frame_count=2, train_ids=[0], held_out_ids=[1])
            views = prepare_group(os.environ['LIVO_PREPARE_BIN'], keyframes, bundle, group, 'a' * 64)
            self.assertEqual({i: view['split'] for i, view in views.items()}, {0: 'train', 1: 'validation'})
            manifest = bundle / 'bundle.json'
            original = json.loads(manifest.read_text())
            for edit in (lambda doc: doc['views'].append(doc['views'][0]),
                         lambda doc: doc['views'].pop(),
                         lambda doc: doc['views'][1].update(keyframe_id='9'),
                         lambda doc: doc['views'][1].update(split='train'),
                         lambda doc: doc['views'][1].update(frame_count='3'),
                         lambda doc: doc['views'][1].update(pose_version='1'),
                         lambda doc: doc['views'][1].pop('parent_from_body')):
                changed = copy.deepcopy(original)
                edit(changed)
                manifest.write_text(json.dumps(changed))
                with self.subTest(manifest=changed), self.assertRaises(ValueError):
                    read_prepared_group(keyframes, bundle, group, 'a' * 64)


if __name__ == '__main__':
    unittest.main()
