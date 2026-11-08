#ifndef LIVO_ATLAS_MAPPING_SUBMAP_POSE_CHANGE_H_
#define LIVO_ATLAS_MAPPING_SUBMAP_POSE_CHANGE_H_
#include "livo_atlas/mapping/keyframe.h"
#include "livo_atlas/mapping/online_submap.h"
#include "livo_atlas/mapping/pose_snapshot.h"
namespace livo_atlas {
struct SubmapPoseChange {
  std::uint64_t submap_id = 0;
  std::uint64_t old_version = 0;
  std::uint64_t new_version = 0;
  RigidTransform old_parent_from_submap;
  RigidTransform new_parent_from_submap;
  std::vector<std::uint64_t> changed_keyframe_ids;
};
// existing_members describes assets already owned by the consumer. Newly added
// observations are initialized separately, never treated as old geometry.
SubmapPoseChange CompareSubmapPoses(const OnlineSubmap& existing_members,
                                    const PoseSnapshot& old_poses,
                                    const PoseSnapshot& new_poses);
RigidTransform CameraPose(const Keyframe& keyframe, const PoseSnapshot& poses);
}  // namespace livo_atlas
#endif  // LIVO_ATLAS_MAPPING_SUBMAP_POSE_CHANGE_H_
