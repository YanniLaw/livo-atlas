#ifndef LIVO_ATLAS_MAPPING_GEOMETRY_SUBMAP_H_
#define LIVO_ATLAS_MAPPING_GEOMETRY_SUBMAP_H_

#include <Eigen/Core>
#include <cstdint>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

#include "livo_atlas/mapping/online_submap.h"
#include "livo_atlas/mapping/pose_snapshot.h"
#include "livo_atlas/sensor/point_types.h"

namespace livo_atlas {

constexpr std::size_t kMaxGeometryKeyframes = 64;

struct GeometryOptions {
  bool enabled = false;
  double voxel_size = 0.2;
  int min_points_per_voxel = 6;
  double max_plane_rms = 0.02;
  double max_plane_ratio = 0.1;
  int max_voxels_per_submap = 100000;
  int max_points_per_keyframe = 1000000;
  int max_resident_submaps = 3;
};

void ValidateGeometryOptions(const GeometryOptions& options);

struct GeometryVoxelKey {
  std::int64_t x = 0, y = 0, z = 0;
  bool operator==(const GeometryVoxelKey& other) const;
  bool operator<(const GeometryVoxelKey& other) const;
};

struct GeometryVoxel {
  GeometryVoxelKey key;
  std::uint64_t point_count = 0;
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  // Point distribution, not measurement or plane-parameter uncertainty.
  Eigen::Matrix3d spatial_covariance = Eigen::Matrix3d::Zero();
  double mean_intensity = 0;
  std::optional<Eigen::Vector3d> normal;
  std::vector<std::uint64_t> keyframe_ids;
};

struct GeometrySource {
  std::uint64_t keyframe_id = 0;
  RigidTransform submap_from_body;
};

struct GeometryData {
  GeometryOptions options;
  std::vector<GeometryVoxel> voxels;
  std::vector<GeometrySource> sources;
  std::uint64_t input_point_count = 0;
};

struct GeometrySubmap {
  OnlineSubmap membership;
  std::uint64_t pose_version = 0;
  std::uint64_t frame_count = 0;
  std::uint64_t optimized_frame_count = 0;
  PoseFrame parent_frame = PoseFrame::kOdom;
  RigidTransform parent_from_submap;
  std::uint64_t representation_revision = 0;
  std::shared_ptr<const GeometryData> data;
};

struct GeometryCacheStats {
  std::uint64_t resident_submaps = 0, resident_voxels = 0;
  std::uint64_t full_builds = 0, incremental_updates = 0, rigid_updates = 0;
  std::uint64_t evictions = 0, cloud_reads = 0;
  double last_update_seconds = 0, last_cloud_read_seconds = 0;
  double last_full_build_seconds = 0, last_snapshot_seconds = 0;
};

class GeometrySubmapBuilder {
 public:
  explicit GeometrySubmapBuilder(GeometryOptions options);
  void AddKeyframe(std::uint64_t id, const PointCloud& body_cloud,
                   const RigidTransform& submap_from_body);
  std::shared_ptr<const GeometryData> Snapshot() const;

 private:
  struct KeyHash {
    std::size_t operator()(const GeometryVoxelKey& key) const;
  };
  struct Moments {
    std::uint64_t count = 0;
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    Eigen::Matrix3d m2 = Eigen::Matrix3d::Zero();
    double mean_intensity = 0;
    std::vector<std::uint64_t> ids;
  };

  GeometryOptions options_;
  std::unordered_map<GeometryVoxelKey, Moments, KeyHash> voxels_;
  std::vector<GeometrySource> sources_;
  std::uint64_t input_point_count_ = 0;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_MAPPING_GEOMETRY_SUBMAP_H_
