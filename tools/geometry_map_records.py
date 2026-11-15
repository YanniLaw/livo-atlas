"""Copy versioned geometry messages into ordinary JSON values for validation."""

import math
import struct


_PARAMETER_FIELDS = (
    'voxel_size', 'min_points_per_voxel', 'max_plane_rms', 'max_plane_ratio',
    'max_voxels_per_submap', 'max_points_per_keyframe', 'max_resident_submaps',
)
_SOURCE_FIELDS = (
    'submap_id', 'anchor_keyframe_id', 'first_keyframe_id',
    'primary_keyframe_count', 'membership_revision', 'state',
    'representation_revision', 'built_pose_version', 'source_voxel_count',
    'input_point_count', 'point_offset', 'point_count',
)
_ROOT_FIELDS = (
    'producer_id', 'revision', 'pose_version', 'frame_count',
    'optimized_frame_count', 'submap_count', 'timestamp', 'status', 'error',
    'representation', 'fusion_policy', 'sampling_policy', 'max_keyframes',
    'max_points', 'max_cached_points', 'max_message_bytes',
    'effective_max_points', 'source_voxel_count', 'sampled',
)


def geometry_map_record(message, to_ns):
    """Decode the declared little-endian XYZI cloud and own every value."""
    cloud = message.cloud
    fields = [dict(name=field.name, offset=field.offset, datatype=field.datatype,
                   count=field.count) for field in cloud.fields]
    expected = [(name, offset, 7, 1) for name, offset in
                (('x', 0), ('y', 4), ('z', 8), ('intensity', 12))]
    layout = [(field['name'], field['offset'], field['datatype'], field['count'])
              for field in fields]
    data = bytes(cloud.data)
    stamp_ns = to_ns(message.header.stamp)
    cloud_stamp_ns = to_ns(cloud.header.stamp)
    if (cloud.height != 1 or cloud.is_bigendian or cloud.point_step != 16 or
            cloud.row_step != cloud.width * 16 or len(data) != cloud.row_step or
            layout != expected or cloud.header.frame_id != message.header.frame_id or
            cloud_stamp_ns != stamp_ns):
        raise ValueError('Invalid geometry map cloud layout or header')
    points = [list(point) for point in struct.iter_unpack('<ffff', data)]
    if not all(math.isfinite(value) for point in points for value in point):
        raise ValueError('Geometry map cloud must contain finite float32 values')
    result = {name: getattr(message, name) for name in _ROOT_FIELDS}
    result.update(stamp_ns=stamp_ns, frame=message.header.frame_id,
                  parameters={name: getattr(message.parameters, name)
                              for name in _PARAMETER_FIELDS},
                  sources=[{name: getattr(source, name) for name in _SOURCE_FIELDS}
                           for source in message.sources],
                  cloud=dict(stamp_ns=cloud_stamp_ns, frame=cloud.header.frame_id,
                             height=cloud.height, width=cloud.width, fields=fields,
                             is_bigendian=cloud.is_bigendian, point_step=cloud.point_step,
                             row_step=cloud.row_step, is_dense=cloud.is_dense,
                             xyzi=points))
    return result
