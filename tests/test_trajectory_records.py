import json
from types import SimpleNamespace as Value
import unittest

from tools.trajectory_records import trajectory_record, trajectory_path_record


class TrajectoryRecordsTest(unittest.TestCase):
    def test_records_own_all_values_and_keep_uint64_precision(self):
        stamp = Value(sec=9, nanosec=500000000)
        header = Value(stamp=stamp, frame_id='map')
        pose = Value(position=Value(x=1.0, y=2.0, z=3.0),
                     orientation=Value(x=0.0, y=0.0, z=.6, w=.8))
        message = Value(header=header, producer_id='a' * 32,
                        revision=1152921504606846979, pose_version=2,
                        frame_count=1, optimized_frame_count=1, status=0, error='',
                        keyframes=[Value(id=0, timestamp=9.5, pose=pose)])
        to_ns = lambda t: t.sec * 1000000000 + t.nanosec
        record = trajectory_record(message, to_ns)
        path = trajectory_path_record(Value(header=header, poses=[Value(header=header, pose=pose)]), to_ns)
        pose.position.x = 99
        record = json.loads(json.dumps(record))
        self.assertEqual(record, dict(stamp_ns=9500000000, frame='map',
                                      producer_id='a' * 32,
                                      revision=1152921504606846979, pose_version=2,
                                      frame_count=1, optimized_frame_count=1, status=0, error='',
                                      keyframes=[dict(id=0, timestamp=9.5,
                                                      translation=[1, 2, 3],
                                                      rotation_xyzw=[0, 0, .6, .8])]))
        self.assertEqual(path, dict(stamp_ns=9500000000, frame='map',
                                    poses=[dict(stamp_ns=9500000000, frame='map',
                                                translation=[1, 2, 3], rotation_xyzw=[0, 0, .6, .8])]))
