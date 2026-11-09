#include "livo_atlas/mapping/pose_snapshot.h"

#include <limits>
#include <stdexcept>

#include "pose_math.h"
namespace livo_atlas {
void ValidatePoseSnapshot(const PoseSnapshot& snapshot) {
  if (snapshot.frame_count != snapshot.poses.size() ||
      (snapshot.version == 0
           ? snapshot.base_version != 0 ||
                 snapshot.optimized_frame_count != 0 ||
                 snapshot.parent_frame != PoseFrame::kOdom
           : snapshot.base_version != snapshot.version - 1 ||
                 snapshot.optimized_frame_count == 0 ||
                 snapshot.optimized_frame_count > snapshot.frame_count ||
                 snapshot.parent_frame != PoseFrame::kMap))
    throw std::invalid_argument("Invalid pose snapshot header or coverage");
  for (std::size_t i = 0; i < snapshot.poses.size(); ++i) {
    if (snapshot.poses[i].id != i)
      throw std::invalid_argument(
          "Pose snapshot IDs must be a complete ordered prefix");
    pose_math::Validate(snapshot.poses[i].map_from_body);
  }
}

PoseSnapshot MaterializePoseSnapshot(
    const PoseSnapshot& frozen, const std::vector<RigidTransform>& raw_poses,
    std::uint64_t frame_count) {
  ValidatePoseSnapshot(frozen);
  if (frozen.frame_count > raw_poses.size() || frame_count > raw_poses.size() ||
      (frozen.version && frame_count < frozen.frame_count))
    throw std::invalid_argument("Requested pose frontier is out of range");
  for (const auto& pose : raw_poses) pose_math::Validate(pose);
  RigidTransform correction;
  if (frozen.version) {
    const auto anchor = frozen.optimized_frame_count - 1;
    correction = pose_math::Compose(frozen.poses[anchor].map_from_body,
                                    pose_math::Inverse(raw_poses[anchor]));
    for (auto i = frozen.optimized_frame_count; i < frozen.frame_count; ++i) {
      if (!pose_math::Equal(frozen.poses[i].map_from_body,
                            pose_math::Compose(correction, raw_poses[i])))
        throw std::invalid_argument(
            "Frozen pose tail does not follow its optimized anchor");
    }
  } else {
    for (std::size_t i = 0; i < frozen.poses.size(); ++i)
      if (!pose_math::Equal(frozen.poses[i].map_from_body, raw_poses[i]))
        throw std::invalid_argument("Version zero must match raw poses");
  }
  PoseSnapshot result = frozen;
  result.frame_count = frame_count;
  if (!frozen.version) result.poses.clear();
  result.poses.reserve(frame_count);
  for (auto i = result.poses.size(); i < frame_count; ++i)
    result.poses.push_back({i, pose_math::Compose(correction, raw_poses[i])});
  ValidatePoseSnapshot(result);
  return result;
}

PoseSnapshot MakePoseSnapshot(const PoseUpdate& update,
                              const PoseSnapshot& previous,
                              const std::vector<RigidTransform>& raw_poses) {
  // Validate the old frozen tail even when replacing it with a new
  // optimization.
  (void)MaterializePoseSnapshot(previous, raw_poses, raw_poses.size());
  if (update.base_version != previous.version ||
      update.optimized_frame_count < previous.optimized_frame_count ||
      update.optimized_frame_count == 0 ||
      update.optimized_frame_count > raw_poses.size() ||
      update.poses.size() != update.optimized_frame_count)
    throw std::invalid_argument(
        "Stale pose update or invalid optimized prefix");
  if (previous.version == std::numeric_limits<std::uint64_t>::max())
    throw std::overflow_error("Pose version exhausted");
  PoseSnapshot optimized;
  optimized.version = previous.version + 1;
  optimized.base_version = previous.version;
  optimized.frame_count = update.optimized_frame_count;
  optimized.optimized_frame_count = update.optimized_frame_count;
  optimized.parent_frame = PoseFrame::kMap;
  optimized.poses = update.poses;
  return MaterializePoseSnapshot(optimized, raw_poses, raw_poses.size());
}
}  // namespace livo_atlas
