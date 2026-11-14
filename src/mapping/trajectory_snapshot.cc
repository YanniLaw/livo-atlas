#include "livo_atlas/mapping/trajectory_snapshot.h"

#include <cmath>
#include <stdexcept>

#include "pose_math.h"

namespace livo_atlas {

TrajectorySnapshot BuildTrajectorySnapshot(
    const PoseSnapshot& poses, const std::vector<double>& timestamps) {
  ValidatePoseSnapshot(poses);
  if (timestamps.size() != poses.frame_count)
    throw std::invalid_argument(
        "Trajectory timestamps must cover its frontier");
  if (poses.version &&
      poses.tail_policy != PoseTailPolicy::kProjectedLastOptimizedAnchor)
    throw std::invalid_argument("Live trajectory requires projected pose tail");

  TrajectorySnapshot result;
  result.pose_version = poses.version;
  result.frame_count = poses.frame_count;
  result.optimized_frame_count = poses.optimized_frame_count;
  result.parent_frame = poses.parent_frame;
  result.keyframes.reserve(poses.frame_count);
  for (std::size_t i = 0; i < poses.frame_count; ++i) {
    if (!std::isfinite(timestamps[i]))
      throw std::invalid_argument("Nonfinite trajectory timestamp");
    const auto& pose = poses.poses[i].map_from_body;
    result.keyframes.push_back(
        {i, timestamps[i],
         poses.version ? pose : pose_math::ProjectPose(pose)});
  }
  return result;
}

}  // namespace livo_atlas
