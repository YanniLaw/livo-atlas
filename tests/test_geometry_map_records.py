import json
import struct
from types import SimpleNamespace as Value
import unittest

from tools.geometry_map_records import geometry_map_record


class GeometryMapRecordsTest(unittest.TestCase):
    def fixture(self):
        stamp = Value(sec=9, nanosec=500000000)
        header = Value(stamp=stamp, frame_id='map')
        fields = [Value(name=name, offset=offset, datatype=7, count=1)
                  for name, offset in [('x', 0), ('y', 4), ('z', 8), ('intensity', 12)]]
        cloud = Value(header=header, height=1, width=2, fields=fields,
                      is_bigendian=False, point_step=16, row_step=32,
                      is_dense=True,
                      data=bytearray(struct.pack('<8f', 1.25, -2.5, 3.75, 4,
                                                 -5.5, 6.25, 7.5, 8)))
        parameters = Value(voxel_size=.2, min_points_per_voxel=6,
                           max_plane_rms=.02, max_plane_ratio=.1,
                           max_voxels_per_submap=100000,
                           max_points_per_keyframe=1000000,
                           max_resident_submaps=3)
        source = Value(submap_id=0, anchor_keyframe_id=0, first_keyframe_id=0,
                       primary_keyframe_count=1, membership_revision=1, state=0,
                       representation_revision=1152921504606846981,
                       built_pose_version=2, source_voxel_count=2,
                       input_point_count=8, point_offset=0, point_count=2)
        return Value(header=header, producer_id='a' * 32,
                     revision=1152921504606846979, pose_version=2,
                     frame_count=1, optimized_frame_count=1, submap_count=1,
                     timestamp=9.5, status=0, error='',
                     representation='voxel_moments_v1', fusion_policy='primary_only',
                     sampling_policy='geometry_rank_v1', parameters=parameters,
                     max_keyframes=20, max_points=2, max_cached_points=300000,
                     max_message_bytes=8388608, effective_max_points=2,
                     source_voxel_count=2, sampled=False, sources=[source], cloud=cloud)

    def test_copies_points_and_all_metadata_without_losing_uint64(self):
        message = self.fixture()
        record = geometry_map_record(message, lambda t: t.sec * 1000000000 + t.nanosec)
        message.cloud.data[3] = 0
        message.sources[0].point_count = 99
        value = json.loads(json.dumps(record))
        self.assertEqual(value['stamp_ns'], 9500000000)
        self.assertEqual(value['revision'], 1152921504606846979)
        self.assertEqual(value['sources'][0]['representation_revision'], 1152921504606846981)
        self.assertEqual(value['sources'][0]['point_count'], 2)
        self.assertEqual(value['cloud']['xyzi'],
                         [[1.25, -2.5, 3.75, 4], [-5.5, 6.25, 7.5, 8]])
        self.assertEqual(value['parameters']['voxel_size'], .2)
        self.assertEqual(value['cloud']['fields'][3]['offset'], 12)
        self.assertEqual(value['cloud']['stamp_ns'], value['stamp_ns'])
        self.assertEqual(value['max_cached_points'], 300000)

    def test_rejects_malformed_cloud_layout(self):
        for mutate in (lambda cloud: setattr(cloud, 'point_step', 12),
                       lambda cloud: setattr(cloud, 'is_bigendian', True),
                       lambda cloud: cloud.data.pop(),
                       lambda cloud: setattr(cloud.fields[0], 'offset', 4)):
            with self.subTest(mutate=mutate):
                message = self.fixture()
                mutate(message.cloud)
                with self.assertRaises(ValueError):
                    geometry_map_record(message, lambda t: t.sec * 1000000000 + t.nanosec)


if __name__ == '__main__':
    unittest.main()
