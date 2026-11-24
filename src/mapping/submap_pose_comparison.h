#ifndef LIVO_ATLAS_SRC_MAPPING_SUBMAP_POSE_COMPARISON_H_
#define LIVO_ATLAS_SRC_MAPPING_SUBMAP_POSE_COMPARISON_H_

#include "livo_atlas/mapping/submap_pose_change.h"

namespace livo_atlas {

// Both snapshots must stay alive and unchanged throughout the comparisons.
class SubmapPoseComparison {
 public:
  SubmapPoseComparison(const PoseSnapshot& old_poses,
                       const PoseSnapshot& new_poses);

  SubmapPoseChange Compare(const OnlineSubmap& existing_members) const;

 private:
  const PoseSnapshot& old_poses_;
  const PoseSnapshot& new_poses_;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_SRC_MAPPING_SUBMAP_POSE_COMPARISON_H_
