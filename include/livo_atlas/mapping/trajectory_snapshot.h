#ifndef LIVO_ATLAS_MAPPING_TRAJECTORY_SNAPSHOT_H_
#define LIVO_ATLAS_MAPPING_TRAJECTORY_SNAPSHOT_H_

#include <cstdint>
#include <string>
#include <vector>

#include "livo_atlas/mapping/pose_snapshot.h"

namespace livo_atlas {

enum class TrajectoryStatus : std::uint8_t {
  kReady = 0,
  kResourceLimited = 1,
  kError = 2,
};

struct TimedKeyframePose {
  std::uint64_t id = 0;
  double timestamp = 0;
  RigidTransform parent_from_body;
};

// Derived output, never a second pose authority. Non-ready results retain the
// source frontier but have no keyframes. producer_id/revision identify a live
// output instance rather than a persistent dataset or pose version.
struct TrajectorySnapshot {
  std::string producer_id;
  std::uint64_t revision = 0;
  std::uint64_t pose_version = 0;
  std::uint64_t frame_count = 0;
  std::uint64_t optimized_frame_count = 0;
  PoseFrame parent_frame = PoseFrame::kOdom;
  TrajectoryStatus status = TrajectoryStatus::kReady;
  std::string error;
  std::vector<TimedKeyframePose> keyframes;
};

// Times correspond to the complete ID prefix, including repeated/backward
// acquisition times. The caller assigns the output's producer and revision.
TrajectorySnapshot BuildTrajectorySnapshot(
    const PoseSnapshot& poses, const std::vector<double>& timestamps);

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_MAPPING_TRAJECTORY_SNAPSHOT_H_
