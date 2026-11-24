#ifndef LIVO_ATLAS_MAPPING_GEOMETRY_MAPPER_H_
#define LIVO_ATLAS_MAPPING_GEOMETRY_MAPPER_H_

#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <vector>

#include "livo_atlas/mapping/geometry_submap.h"
#include "livo_atlas/mapping/keyframe.h"
#include "livo_atlas/sensor/point_types.h"

namespace livo_atlas {

// Calls are serialized by the owner. The reader accesses committed body clouds.
class GeometryMapper {
 public:
  using CloudReader = std::function<PointCloud::ConstPtr(std::uint64_t)>;

  GeometryMapper(GeometryOptions options, CloudReader cloud_reader);
  void Update(const std::vector<OnlineSubmap>& revisions,
              std::shared_ptr<const PoseSnapshot> poses,
              const Keyframe* new_keyframe = nullptr);
  // Append or seal at the current pose version. A new keyframe carries its
  // resolved pose; a seal carries neither. Pose corrections use Update().
  void UpdateDelta(const std::vector<OnlineSubmap>& revisions,
                   const PoseFrontier& frontier,
                   const Keyframe* new_keyframe = nullptr,
                   const RigidTransform* new_pose = nullptr);
  GeometrySubmap GetSubmap(std::uint64_t id);
  GeometryCacheStats GetStats() const;

 private:
  struct Entry {
    OnlineSubmap membership;
    std::unique_ptr<GeometrySubmapBuilder> builder;
    std::shared_ptr<const GeometryData> data;
    std::vector<GeometrySource> built_sources;
    std::uint64_t representation_revision = 0;
    std::uint64_t last_access = 0;
  };

  void ResetTimings();
  const RigidTransform& PoseAt(std::uint64_t id) const;
  RigidTransform SubmapFromBody(const OnlineSubmap& group,
                                std::uint64_t id) const;
  void UpdateNewKeyframe(const Keyframe& keyframe);
  void ReleaseData(std::uint64_t id);
  void EvictFor(std::uint64_t id);
  void Build(std::uint64_t id, const Keyframe* new_keyframe);
  void Append(std::uint64_t id, const Keyframe& keyframe);

  GeometryOptions options_;
  CloudReader cloud_reader_;
  std::deque<Entry> entries_;
  std::vector<std::uint64_t> resident_ids_;
  std::shared_ptr<const PoseSnapshot> poses_;
  std::deque<RigidTransform> tail_poses_;
  PoseFrontier frontier_;
  GeometryCacheStats stats_;
  std::uint64_t access_clock_ = 0;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_MAPPING_GEOMETRY_MAPPER_H_
