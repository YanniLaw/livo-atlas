#ifndef LIVO_ATLAS_MAPPING_GEOMETRY_MAP_SNAPSHOT_H_
#define LIVO_ATLAS_MAPPING_GEOMETRY_MAP_SNAPSHOT_H_

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "livo_atlas/mapping/geometry_submap.h"

namespace livo_atlas {

enum class GeometryMapStatus : std::uint8_t {
  kReady = 0,
  kResourceLimited = 1,
  kError = 2,
};

struct GeometryMapOptions {
  int max_points = 100000;
  int max_cached_points = 300000;
  int max_message_bytes = 8388608;
};

void ValidateGeometryMapOptions(const GeometryMapOptions& options);

struct GeometryMapPoint {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  double intensity = 0;
};

struct GeometryMapSourceInfo {
  std::uint64_t submap_id = 0;
  std::uint64_t anchor_keyframe_id = 0;
  std::uint64_t first_keyframe_id = 0;
  std::uint64_t primary_keyframe_count = 0;
  std::uint64_t membership_revision = 0;
  OnlineSubmapState state = OnlineSubmapState::kActive;
  std::uint64_t representation_revision = 0;
  std::uint64_t built_pose_version = 0;
  std::uint64_t source_voxel_count = 0;
  std::uint64_t input_point_count = 0;
  std::uint64_t point_offset = 0;
  std::uint64_t point_count = 0;
};

// Display data is derived from immutable observations; its revisions identify
// one output instance and are not persistent dataset identifiers.
struct GeometryMapSnapshot {
  std::string producer_id;
  std::uint64_t revision = 0;
  std::uint64_t pose_version = 0;
  std::uint64_t frame_count = 0;
  std::uint64_t optimized_frame_count = 0;
  std::uint64_t submap_count = 0;
  PoseFrame parent_frame = PoseFrame::kOdom;
  double timestamp = 0;
  GeometryMapStatus status = GeometryMapStatus::kReady;
  std::string error;
  std::string representation = "voxel_moments_v1";
  std::string fusion_policy = "primary_only";
  std::string sampling_policy = "geometry_rank_v1";
  GeometryOptions geometry_options;
  GeometryMapOptions options;
  // The live controller sets max_keyframes; zero is unspecified for a builder.
  std::uint64_t max_keyframes = 0;
  std::uint64_t effective_max_points = 0;
  std::uint64_t source_voxel_count = 0;
  bool sampled = false;
  std::vector<GeometryMapSourceInfo> sources;
  std::vector<GeometryMapPoint> points;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_MAPPING_GEOMETRY_MAP_SNAPSHOT_H_
