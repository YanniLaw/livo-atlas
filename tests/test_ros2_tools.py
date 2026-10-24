"""Run with Python 3.10+ and rosbags==0.10.9; no ROS master required."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
import sys

if sys.version_info < (3, 10):
    raise unittest.SkipTest("ROS2 migration tests require Python 3.10 and rosbags==0.10.9")

import numpy as np
from rosbags.typesys import Stores, get_typestore, get_types_from_msg


def module(name):
    path = Path(__file__).resolve().parents[1] / 'tools' / (name + '.py')
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


convert = module('convert_ros1_bag')
runner = module('run_ros2_baseline')


class Ros2ToolsTest(unittest.TestCase):
    def test_time_and_every_payload_field_affect_digest_but_header_seq_does_not(self):
        store = get_typestore(Stores.ROS1_NOETIC)
        stamp = store.types['builtin_interfaces/msg/Time'](17, 123456789)
        header = store.types['std_msgs/msg/Header'](77, stamp, 'camera')
        image = store.types['sensor_msgs/msg/Image'](header, 1, 2, 'bgr8', 0, 6,
                                                   np.array([1, 2, 3, 4, 5, 6], dtype=np.uint8))
        digest = convert.message_digest(image)
        header.seq = 78
        self.assertEqual(convert.message_digest(image), digest)
        stamp.nanosec += 1
        self.assertNotEqual(convert.message_digest(image), digest)
        stamp.nanosec -= 1
        image.data[0] = 99
        self.assertNotEqual(convert.message_digest(image), digest)

    def test_header_conversion_drops_seq_and_preserves_integer_stamp(self):
        source = get_typestore(Stores.ROS1_NOETIC)
        target = get_typestore(Stores.ROS2_HUMBLE)
        stamp = source.types['builtin_interfaces/msg/Time'](1700000000, 123456789)
        header = source.types['std_msgs/msg/Header'](23, stamp, 'livox_frame')
        converted = convert.convert_message(header, target)
        self.assertIsInstance(converted, target.types['std_msgs/msg/Header'])
        self.assertFalse(hasattr(converted, 'seq'))
        self.assertEqual(converted.stamp.sec, stamp.sec)
        self.assertEqual(converted.stamp.nanosec, stamp.nanosec)
        self.assertEqual(convert.message_digest(converted), convert.message_digest(header))

    def test_livox_point_conversion_preserves_precision_and_fields(self):
        source = get_typestore(Stores.ROS1_NOETIC)
        target = get_typestore(Stores.ROS2_HUMBLE)
        definition = 'uint32 offset_time\nfloat32 x\nfloat32 y\nfloat32 z\nuint8 reflectivity\nuint8 tag\nuint8 line\n'
        for store, package in ((source, 'livox_ros_driver'), (target, 'livox_ros_driver2')):
            store.register(get_types_from_msg(definition, package + '/msg/CustomPoint'))
        point = source.types['livox_ros_driver/msg/CustomPoint'](4294967290, 1.25, -2.5, 3.75, 231, 16, 5)
        actual = convert.convert_message(point, target)
        self.assertIsInstance(actual, target.types['livox_ros_driver2/msg/CustomPoint'])
        self.assertEqual((actual.offset_time, actual.x, actual.y, actual.z, actual.reflectivity, actual.tag, actual.line),
                         (4294967290, 1.25, -2.5, 3.75, 231, 16, 5))
        self.assertEqual(convert.message_digest(point), convert.message_digest(actual))

    def test_runner_rejects_errors_and_missing_or_extra_inputs(self):
        self.assertTrue(runner.exited_cleanly(0, 'normal shutdown'))
        self.assertFalse(runner.exited_cleanly(-11, ''))
        self.assertFalse(runner.exited_cleanly(0, 'terminate called after throwing'))
        self.assertEqual(runner.count_mismatches({'imu': 3}, {'imu': 2}), {'imu': {'expected': 3, 'observed': 2}})
        self.assertEqual(runner.count_mismatches({'imu': 3}, {'imu': 3, 'image': 1}), {'image': {'expected': 0, 'observed': 1}})


if __name__ == '__main__':
    unittest.main()
