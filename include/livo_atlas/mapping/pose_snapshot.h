#ifndef LIVO_ATLAS_MAPPING_POSE_SNAPSHOT_H_
#define LIVO_ATLAS_MAPPING_POSE_SNAPSHOT_H_

#include <cstdint>
#include <vector>

#include "livo_atlas/sensor/extrinsics.h"

namespace livo_atlas {

enum class PoseTailPolicy {
  kLegacyLastOptimizedAnchor,
  kProjectedLastOptimizedAnchor,
};

struct KeyframePose {
  std::uint64_t id = 0;
  // Legacy storage name. The containing snapshot defines the parent frame.
  RigidTransform map_from_body;

  const RigidTransform& parent_from_body() const { return map_from_body; }
};

enum class PoseFrame { kOdom, kMap };

struct PoseFrontier {
  std::uint64_t version = 0;
  std::uint64_t base_version = 0;
  std::uint64_t frame_count = 0;
  std::uint64_t optimized_frame_count = 0;
  PoseFrame parent_frame = PoseFrame::kOdom;
  PoseTailPolicy tail_policy = PoseTailPolicy::kLegacyLastOptimizedAnchor;

  bool operator==(const PoseFrontier& other) const {
    return version == other.version && base_version == other.base_version &&
           frame_count == other.frame_count &&
           optimized_frame_count == other.optimized_frame_count &&
           parent_frame == other.parent_frame &&
           tail_policy == other.tail_policy;
  }
};

struct PoseUpdate {
  std::uint64_t base_version = 0;
  std::uint64_t optimized_frame_count = 0;
  std::vector<KeyframePose> poses;
  PoseTailPolicy tail_policy = PoseTailPolicy::kLegacyLastOptimizedAnchor;
};

// Share snapshots as const. In version zero map_from_body holds odom_from_body;
// parent_frame, not the legacy field name, defines the coordinate frame.
struct PoseSnapshot {
  std::uint64_t version = 0;
  std::uint64_t base_version = 0;
  std::uint64_t frame_count = 0;
  std::uint64_t optimized_frame_count = 0;
  PoseFrame parent_frame = PoseFrame::kOdom;
  std::vector<KeyframePose> poses;
  PoseTailPolicy tail_policy = PoseTailPolicy::kLegacyLastOptimizedAnchor;

  PoseFrontier frontier() const {
    return {version,      base_version, frame_count, optimized_frame_count,
            parent_frame, tail_policy};
  }
};
void ValidatePoseSnapshot(const PoseSnapshot& snapshot);
PoseSnapshot MaterializePoseSnapshot(
    const PoseSnapshot& frozen, const std::vector<RigidTransform>& raw_poses,
    std::uint64_t frame_count);
PoseSnapshot MakePoseSnapshot(const PoseUpdate& update,
                              const PoseSnapshot& previous,
                              const std::vector<RigidTransform>& raw_poses);
}  // namespace livo_atlas
#endif  // LIVO_ATLAS_MAPPING_POSE_SNAPSHOT_H_
