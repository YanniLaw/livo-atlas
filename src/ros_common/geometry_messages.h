#ifndef LIVO_ATLAS_SRC_ROS_COMMON_GEOMETRY_MESSAGES_H_
#define LIVO_ATLAS_SRC_ROS_COMMON_GEOMETRY_MESSAGES_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include "livo_atlas/mapping/geometry_map_snapshot.h"
#include "mapping/geometry_map_sampling.h"
#include "ros_common/time_conversion.h"

namespace livo_atlas::ros_detail {

inline void StoreFloat32(double value, std::uint8_t* destination) {
  if (!std::isfinite(value) ||
      std::abs(value) > std::numeric_limits<float>::max())
    throw std::invalid_argument("Geometry point exceeds float32 range");
  static_assert(sizeof(float) == sizeof(std::uint32_t) &&
                    std::numeric_limits<float>::is_iec559,
                "Geometry messages require IEEE-754 float32");
  const float converted = static_cast<float>(value);
  std::uint32_t bits;
  std::memcpy(&bits, &converted, sizeof(bits));
  for (int i = 0; i < 4; ++i)
    destination[i] = static_cast<std::uint8_t>(bits >> (8 * i));
}

// ROS1 and ROS2 generated messages use the same schema and little-endian
// XYZI layout. No mutable geometry or middleware object enters the core.
template <typename Message, typename Cloud, typename Stamp>
std::pair<Message, Cloud> MakeGeometryMessages(
    const GeometryMapSnapshot& source, const std::string& map_frame,
    Stamp stamp) {
  if (source.producer_id.size() != 32 || !source.revision ||
      !std::all_of(source.producer_id.begin(), source.producer_id.end(),
                   [](char c) {
                     return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
                   }) ||
      (source.status != GeometryMapStatus::kReady &&
       source.status != GeometryMapStatus::kResourceLimited &&
       source.status != GeometryMapStatus::kError) ||
      (source.pose_version == 0
           ? source.parent_frame != PoseFrame::kOdom ||
                 source.optimized_frame_count != 0
           : source.parent_frame != PoseFrame::kMap ||
                 !source.optimized_frame_count ||
                 source.optimized_frame_count > source.frame_count))
    throw std::invalid_argument("Invalid geometry map identity or header");

  Message message;
  Cloud cloud;
  message.header.frame_id =
      source.parent_frame == PoseFrame::kOdom ? "camera_init" : map_frame;
  message.header.stamp = stamp(0);
  message.producer_id = source.producer_id;
  message.revision = source.revision;
  message.pose_version = source.pose_version;
  message.frame_count = source.frame_count;
  message.optimized_frame_count = source.optimized_frame_count;
  message.submap_count = source.submap_count;
  message.status = static_cast<std::uint8_t>(source.status);
  message.error = source.error.substr(0, 1024);
  message.representation = "voxel_moments_v1";
  message.fusion_policy = "primary_only";
  message.sampling_policy = "geometry_rank_v1";
  const auto& geometry = source.geometry_options;
  message.parameters.voxel_size = geometry.voxel_size;
  message.parameters.min_points_per_voxel = geometry.min_points_per_voxel;
  message.parameters.max_plane_rms = geometry.max_plane_rms;
  message.parameters.max_plane_ratio = geometry.max_plane_ratio;
  message.parameters.max_voxels_per_submap = geometry.max_voxels_per_submap;
  message.parameters.max_points_per_keyframe = geometry.max_points_per_keyframe;
  message.parameters.max_resident_submaps = geometry.max_resident_submaps;
  message.max_keyframes = source.max_keyframes;
  message.max_points = source.options.max_points;
  message.max_cached_points = source.options.max_cached_points;
  message.max_message_bytes = source.options.max_message_bytes;
  message.effective_max_points = source.effective_max_points;
  cloud.height = 1;
  cloud.is_bigendian = false;
  cloud.point_step = 16;
  cloud.is_dense = true;
  cloud.fields.resize(4);
  const char* names[] = {"x", "y", "z", "intensity"};
  for (std::size_t i = 0; i < cloud.fields.size(); ++i) {
    auto& field = cloud.fields[i];
    field.name = names[i];
    field.offset = static_cast<std::uint32_t>(4 * i);
    field.datatype = field.FLOAT32;
    field.count = 1;
  }
  try {
    if (source.status == GeometryMapStatus::kReady) {
      ValidateGeometryOptions(geometry);
      ValidateGeometryMapOptions(source.options);
      if (!source.max_keyframes || source.max_keyframes > 100000 ||
          source.frame_count > source.max_keyframes ||
          source.sources.size() != source.submap_count ||
          source.representation != message.representation ||
          source.fusion_policy != message.fusion_policy ||
          source.sampling_policy != message.sampling_policy ||
          source.effective_max_points !=
              geometry_map_detail::GeometryMapPointLimit(source.options,
                                                         source.submap_count) ||
          (source.frame_count && !source.effective_max_points) ||
          (source.frame_count == 0 && source.timestamp != 0))
        throw std::invalid_argument("Invalid geometry map limits or metadata");
      message.header.stamp = stamp(
          RosNanoseconds(source.timestamp, "Geometry map", "geometry map"));
      message.timestamp = source.timestamp;
      std::uint64_t next_frame = 0, next_point = 0, total_voxels = 0;
      message.sources.resize(source.sources.size());
      for (std::size_t i = 0; i < source.sources.size(); ++i) {
        const auto& input = source.sources[i];
        const bool sealed = input.state == OnlineSubmapState::kSealed;
        if (input.submap_id != i || input.first_keyframe_id != next_frame ||
            input.anchor_keyframe_id != next_frame ||
            !input.primary_keyframe_count ||
            input.primary_keyframe_count > kMaxGeometryKeyframes ||
            next_frame > source.frame_count ||
            input.primary_keyframe_count > source.frame_count - next_frame ||
            (input.state != OnlineSubmapState::kActive && !sealed) ||
            (!sealed && i + 1 != source.sources.size()) ||
            input.membership_revision !=
                input.primary_keyframe_count + (sealed ? 1 : 0) ||
            !input.representation_revision ||
            input.built_pose_version > source.pose_version ||
            input.point_offset != next_point ||
            next_point > source.points.size() ||
            input.point_count > source.points.size() - next_point ||
            input.point_count > input.source_voxel_count ||
            input.source_voxel_count >
                std::numeric_limits<std::uint64_t>::max() - total_voxels)
          throw std::invalid_argument("Invalid geometry map source range");
        next_frame += input.primary_keyframe_count;
        next_point += input.point_count;
        total_voxels += input.source_voxel_count;
        auto& output = message.sources[i];
        output.submap_id = input.submap_id;
        output.anchor_keyframe_id = input.anchor_keyframe_id;
        output.first_keyframe_id = input.first_keyframe_id;
        output.primary_keyframe_count = input.primary_keyframe_count;
        output.membership_revision = input.membership_revision;
        output.state = static_cast<std::uint8_t>(input.state);
        output.representation_revision = input.representation_revision;
        output.built_pose_version = input.built_pose_version;
        output.source_voxel_count = input.source_voxel_count;
        output.input_point_count = input.input_point_count;
        output.point_offset = input.point_offset;
        output.point_count = input.point_count;
      }
      if (next_frame != source.frame_count ||
          next_point != source.points.size() ||
          total_voxels != source.source_voxel_count ||
          source.points.size() != std::min(source.source_voxel_count,
                                           source.effective_max_points) ||
          source.sampled != (source.source_voxel_count > source.points.size()))
        throw std::invalid_argument("Inconsistent geometry map source totals");
      message.source_voxel_count = source.source_voxel_count;
      message.sampled = source.sampled;
      cloud.width = static_cast<std::uint32_t>(source.points.size());
      cloud.row_step = cloud.width * cloud.point_step;
      cloud.data.resize(cloud.row_step);
      for (std::size_t i = 0; i < source.points.size(); ++i) {
        const auto& point = source.points[i];
        auto* bytes = cloud.data.data() + i * cloud.point_step;
        StoreFloat32(point.position.x(), bytes);
        StoreFloat32(point.position.y(), bytes + 4);
        StoreFloat32(point.position.z(), bytes + 8);
        StoreFloat32(point.intensity, bytes + 12);
      }
    }
  } catch (const std::exception& error) {
    message.status = static_cast<std::uint8_t>(GeometryMapStatus::kError);
    message.error = std::string(error.what()).substr(0, 1024);
    message.header.stamp = stamp(0);
    message.timestamp = 0;
    message.source_voxel_count = 0;
    message.sampled = false;
    message.sources.clear();
    cloud.width = 0;
    cloud.row_step = 0;
    cloud.data.clear();
  }
  cloud.header = message.header;
  message.cloud = cloud;
  return {std::move(message), std::move(cloud)};
}

}  // namespace livo_atlas::ros_detail

#endif  // LIVO_ATLAS_SRC_ROS_COMMON_GEOMETRY_MESSAGES_H_
