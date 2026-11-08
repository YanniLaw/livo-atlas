#ifndef LIVO_ATLAS_CORE_ONLINE_SUBMAP_UPDATE_H_
#define LIVO_ATLAS_CORE_ONLINE_SUBMAP_UPDATE_H_

#include <memory>
#include <vector>

#include "livo_atlas/mapping/keyframe.h"
#include "livo_atlas/mapping/submap_pose_change.h"

namespace livo_atlas {
// Owning snapshots. Consumers must not mutate cv::Mat/PCL assets through
// aliases. keyframe is empty for sealing or pose-only updates. Retain
// values/shared_ptrs, not references to this notification, beyond the callback.
struct OnlineSubmapUpdate {
  std::shared_ptr<const Keyframe> keyframe;
  std::vector<OnlineSubmap> revisions;
  std::shared_ptr<const PoseSnapshot> poses;
  std::vector<SubmapPoseChange> pose_changes;
};
}  // namespace livo_atlas
#endif  // LIVO_ATLAS_CORE_ONLINE_SUBMAP_UPDATE_H_
