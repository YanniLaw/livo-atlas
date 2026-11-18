"""Confirmed bounds describe optimized parameters, never camera-path bounds."""
import copy
import unittest

import numpy as np

from spatial_metadata import extract_metadata, validate_metadata
from state_fixtures import make_state


CONTENT = dict(session_id='a' * 32, submap_id='0', source_generation='1', content_job_id='2')


class SpatialMetadataTest(unittest.TestCase):
    def setUp(self):
        self.state = make_state(per_view=2)
        self.state.parameters['means'][:] = [[1, 2, 3], [4, -2, 6], [8, 9, 5], [-3, 2, 7]]
        self.state.parameters['log_scales'][:] = np.log(
            [[.02, .03, .04], [.05, .06, .07], [.01, .02, .03], [.08, .09, .10]])

    def test_groups_actual_optimized_means_and_largest_axis_scale(self):
        result = extract_metadata(self.state, CONTENT)
        self.assertEqual(result['content'], CONTENT)
        self.assertEqual(result['extents'][0]['min'], [1., -2., 3.])
        self.assertEqual(result['extents'][0]['max'], [4., 2., 6.])
        self.assertEqual(result['extents'][1]['min'], [-3., 2., 5.])
        self.assertEqual(result['extents'][1]['max'], [8., 9., 7.])
        self.assertAlmostEqual(result['extents'][0]['max_scale'], .07, places=7)
        self.assertAlmostEqual(result['extents'][1]['max_scale'], .10, places=7)
        self.assertEqual([item['origin_keyframe_id'] for item in result['extents']], ['0', '1'])
        self.assertEqual(result['camera']['keyframe_id'], '1')
        self.assertEqual(result['camera']['capture_time'], 101.01)
        self.assertEqual(result['camera']['K'], [4., 0., 3.5, 0., 4., 2.5, 0., 0., 1.])
        self.assertEqual(result['camera']['camera_from_body'][:4], [0., -1., 0., 1.])
        self.assertEqual((result['camera']['width'], result['camera']['height']), (8, 6))
        self.state.parameters['means'][:] = 99
        self.assertEqual(result['extents'][0]['min'], [1., -2., 3.])

    def test_capture_parent_motion_does_not_change_local_bounds_or_query(self):
        before = extract_metadata(self.state, CONTENT)
        self.state.metadata['parent_from_submap'][3] = 1000
        self.state.views[-1]['camera_from_parent'][0, 3] = 2000
        self.assertEqual(extract_metadata(self.state, CONTENT), before)

    def test_malformed_identity_bounds_coverage_and_camera_are_rejected(self):
        original = extract_metadata(self.state, CONTENT)
        edits = [lambda d: d['content'].update(content_job_id='3'),
                 lambda d: d['content'].update(source_generation='0'),
                 lambda d: d['extents'].pop(),
                 lambda d: d['extents'][1].update(origin_keyframe_id='0'),
                 lambda d: d['extents'][0].update(max_scale=0),
                 lambda d: d['extents'][0].update(max_scale=float('nan')),
                 lambda d: d['extents'][0].update(min=[5., 0., 0.]),
                 lambda d: d['extents'][0].update(max=[float('inf'), 0., 0.]),
                 lambda d: d['camera'].update(keyframe_id='0'),
                 lambda d: d['camera'].update(width=1_000_001),
                 lambda d: d['camera']['K'].__setitem__(1, 1.),
                 lambda d: d['camera']['K'].__setitem__(0, -1.),
                 lambda d: d['camera']['camera_from_body'].__setitem__(0, 2e-6),
                 lambda d: d.update(unknown=1)]
        for edit in edits:
            value = copy.deepcopy(original)
            edit(value)
            with self.subTest(edit=edit), self.assertRaises(ValueError):
                validate_metadata(value, CONTENT, [0, 1])

    def test_missing_or_unknown_gaussian_origins_cannot_create_extents(self):
        for origins in ([0, 0, 0, 0], [0, 0, 1, 99]):
            self.state.origin_ids[:] = origins
            with self.subTest(origins=origins), self.assertRaises(ValueError):
                extract_metadata(self.state, CONTENT)

    def test_extent_and_camera_budgets_apply_before_large_allocations(self):
        value = extract_metadata(self.state, CONTENT)
        value['camera'].update(width=1000, height=1000)
        validate_metadata(value, CONTENT, [0, 1])
        value['camera']['height'] = 1001
        with self.assertRaises(ValueError):
            validate_metadata(value, CONTENT, [0, 1])
        value['extents'] *= 6
        with self.assertRaises(ValueError):
            validate_metadata(value, CONTENT, [0, 1])


if __name__ == '__main__':
    unittest.main()
