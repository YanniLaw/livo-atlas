#include "livo_atlas/mapping/submap_pose_change.h"

#include <algorithm>
#include <stdexcept>

#include "pose_math.h"
#include "submap_pose_comparison.h"

namespace livo_atlas {

SubmapPoseComparison::SubmapPoseComparison(const PoseSnapshot& old_poses,
                                           const PoseSnapshot& new_poses)
    : old_poses_(old_poses), new_poses_(new_poses) {
  ValidatePoseSnapshot(old_poses_);
  ValidatePoseSnapshot(new_poses_);
  if (old_poses_.frame_count != new_poses_.frame_count) {
    throw std::invalid_argument("Pose comparisons require the same frontier");
  }
}

SubmapPoseChange SubmapPoseComparison::Compare(
    const OnlineSubmap& existing_members) const {
  ValidateSubmap(existing_members.primary);
  auto ids = existing_members.primary.keyframe_ids;
  const auto& support = existing_members.support_keyframe_ids;
  if (!std::is_sorted(support.begin(), support.end()) ||
      std::adjacent_find(support.begin(), support.end()) != support.end())
    throw std::invalid_argument("Support IDs must be ordered and unique");
  for (auto id : support) {
    if (std::binary_search(ids.begin(), ids.end(), id))
      throw std::invalid_argument("Support and primary IDs must be disjoint");
  }
  ids.insert(ids.end(), support.begin(), support.end());
  std::sort(ids.begin(), ids.end());
  if (ids.back() >= old_poses_.frame_count)
    throw std::invalid_argument(
        "Pose snapshot does not cover existing members");
  SubmapPoseChange result;
  result.submap_id = existing_members.primary.id;
  result.old_version = old_poses_.version;
  result.new_version = new_poses_.version;
  const auto anchor = existing_members.primary.anchor_keyframe_id;
  result.old_parent_from_submap = old_poses_.poses[anchor].parent_from_body();
  result.new_parent_from_submap = new_poses_.poses[anchor].parent_from_body();
  const auto old_submap_from_parent =
      pose_math::Inverse(result.old_parent_from_submap);
  const auto new_submap_from_parent =
      pose_math::Inverse(result.new_parent_from_submap);
  for (auto id : ids) {
    const auto old_local = pose_math::Compose(
        old_submap_from_parent, old_poses_.poses[id].parent_from_body());
    const auto new_local = pose_math::Compose(
        new_submap_from_parent, new_poses_.poses[id].parent_from_body());
    if (!pose_math::Equal(old_local, new_local))
      result.changed_keyframe_ids.push_back(id);
  }
  return result;
}

SubmapPoseChange CompareSubmapPoses(const OnlineSubmap& existing_members,
                                    const PoseSnapshot& old_poses,
                                    const PoseSnapshot& new_poses) {
  return SubmapPoseComparison(old_poses, new_poses).Compare(existing_members);
}

RigidTransform CameraPose(const Keyframe& keyframe, const PoseSnapshot& poses) {
  ValidatePoseSnapshot(poses);
  if (!keyframe.image_timestamp || !keyframe.camera_from_odom ||
      keyframe.image.empty() || keyframe.id >= poses.frame_count)
    throw std::invalid_argument(
        "Camera pose requires a covered image observation");
  pose_math::Validate(keyframe.odom_from_body);
  pose_math::Validate(*keyframe.camera_from_odom);
  return pose_math::Compose(
      poses.poses[keyframe.id].parent_from_body(),
      pose_math::Compose(pose_math::Inverse(keyframe.odom_from_body),
                         pose_math::Inverse(*keyframe.camera_from_odom)));
}

}  // namespace livo_atlas
