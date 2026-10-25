import unittest

import numpy as np

from tools.compare_trajectories import compare_trajectories


class TrajectoryComparisonTest(unittest.TestCase):
    def setUp(self):
        self.reference = np.array([
            [1000, 0, 0, 0, 0, 0, 0, 1],
            [1001, 0, 0, 0, 0, 0, 0, 1],
        ], dtype=float)

    def test_measures_known_translation_and_rotation_without_alignment(self):
        candidate = np.array([
            [1000, 1, 0, 0, 0, 0, 0.7071067811865476, 0.7071067811865476],
            [1001, 1, 0, 0, 0, 0, 0.7071067811865476, 0.7071067811865476],
        ])
        result = compare_trajectories(self.reference, candidate)
        self.assertEqual(result['position_rms_m'], 1.0)
        self.assertEqual(result['position_max_m'], 1.0)
        self.assertAlmostEqual(result['orientation_max_deg'], 90.0)
        self.assertFalse(result['pose_alignment_applied'])

    def test_quaternion_sign_does_not_change_rotation(self):
        candidate = self.reference.copy()
        candidate[:, 4:8] *= -1
        result = compare_trajectories(self.reference, candidate)
        self.assertEqual(result['orientation_max_deg'], 0.0)
        self.assertEqual(result['position_max_m'], 0.0)

    def test_rejects_different_sensor_timestamp_sequence(self):
        candidate = self.reference.copy()
        candidate[1, 0] = 1002
        with self.assertRaisesRegex(ValueError, 'timestamp'):
            compare_trajectories(self.reference, candidate)

    def test_rejects_invalid_quaternion(self):
        candidate = self.reference.copy()
        candidate[0, 4:8] = 0
        with self.assertRaisesRegex(ValueError, 'quaternion'):
            compare_trajectories(self.reference, candidate)
