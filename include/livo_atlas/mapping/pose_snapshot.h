#ifndef LIVO_ATLAS_MAPPING_POSE_SNAPSHOT_H_
#define LIVO_ATLAS_MAPPING_POSE_SNAPSHOT_H_
#include <cstdint>
#include <vector>

#include "livo_atlas/sensor/extrinsics.h"
namespace livo_atlas {
struct KeyframePose {
  std::uint64_t id = 0;
  RigidTransform map_from_body;
};
enum class PoseFrame { kOdom, kMap };
struct PoseUpdate {
  std::uint64_t base_version = 0;
  std::uint64_t optimized_frame_count = 0;
  std::vector<KeyframePose> poses;
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
