#include "livo_atlas/mapping/pose_snapshot.h"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "common/rotation_projection.h"
#include "pose_math.h"

namespace livo_atlas {
namespace pose_math {

RigidTransform ProjectPose(const RigidTransform& pose) {
  Validate(pose);
  return {ProjectRotation(pose.rotation), pose.translation};
}

RigidTransform ProjectedCorrection(const RigidTransform& map_anchor,
                                   const RigidTransform& raw_anchor) {
  Validate(map_anchor);
  return Compose(map_anchor, Inverse(ProjectPose(raw_anchor)));
}

}  // namespace pose_math

namespace {

bool IsProjected(PoseTailPolicy policy) {
  return policy == PoseTailPolicy::kProjectedLastOptimizedAnchor;
}

bool IsRigidRotation(const Eigen::Matrix3d& rotation) {
  return (rotation.transpose() * rotation - Eigen::Matrix3d::Identity())
                 .cwiseAbs()
                 .maxCoeff() <= 1e-9 &&
         std::abs(rotation.determinant() - 1) <= 1e-9;
}

bool EqualProjectedPose(const RigidTransform& a, const RigidTransform& b) {
  return (a.rotation - b.rotation).cwiseAbs().maxCoeff() <= 1e-9 &&
         (a.translation - b.translation).norm() <= 1e-9;
}

RigidTransform TailCorrection(const PoseSnapshot& frozen,
                              const std::vector<RigidTransform>& raw_poses) {
  if (!frozen.version) return {};
  const auto anchor = frozen.optimized_frame_count - 1;
  const auto& map_anchor = frozen.poses[anchor].parent_from_body();
  const auto& raw_anchor = raw_poses[anchor];
  return IsProjected(frozen.tail_policy)
             ? pose_math::ProjectedCorrection(map_anchor, raw_anchor)
             : pose_math::Compose(map_anchor, pose_math::Inverse(raw_anchor));
}

RigidTransform MaterializeTailPose(const RigidTransform& correction,
                                   const RigidTransform& raw,
                                   PoseTailPolicy policy) {
  return pose_math::Compose(
      correction, IsProjected(policy) ? pose_math::ProjectPose(raw) : raw);
}

}  // namespace

void pose_math::ValidateSnapshotPose(const RigidTransform& pose,
                                     PoseTailPolicy policy) {
  Validate(pose);
  if (IsProjected(policy) && !IsRigidRotation(pose.rotation)) {
    throw std::invalid_argument("Projected pose rotation must be rigid");
  }
}

RigidTransform pose_math::MaterializeAppendedPose(
    const PoseSnapshot& frozen, const std::vector<RigidTransform>& raw_poses) {
  const auto& raw = raw_poses.back();
  Validate(raw);
  auto pose = MaterializeTailPose(TailCorrection(frozen, raw_poses), raw,
                                  frozen.tail_policy);
  ValidateSnapshotPose(pose, frozen.tail_policy);
  return pose;
}

void ValidatePoseSnapshot(const PoseSnapshot& snapshot) {
  if ((snapshot.tail_policy != PoseTailPolicy::kLegacyLastOptimizedAnchor &&
       !IsProjected(snapshot.tail_policy)) ||
      snapshot.frame_count != snapshot.poses.size() ||
      (snapshot.version == 0
           ? snapshot.base_version != 0 ||
                 snapshot.optimized_frame_count != 0 ||
                 snapshot.parent_frame != PoseFrame::kOdom ||
                 IsProjected(snapshot.tail_policy)
           : snapshot.base_version != snapshot.version - 1 ||
                 snapshot.optimized_frame_count == 0 ||
                 snapshot.optimized_frame_count > snapshot.frame_count ||
                 snapshot.parent_frame != PoseFrame::kMap)) {
    throw std::invalid_argument("Invalid pose snapshot header or coverage");
  }
  for (std::size_t i = 0; i < snapshot.poses.size(); ++i) {
    if (snapshot.poses[i].id != i) {
      throw std::invalid_argument(
          "Pose snapshot IDs must be a complete ordered prefix");
    }
    pose_math::ValidateSnapshotPose(snapshot.poses[i].parent_from_body(),
                                    snapshot.tail_policy);
  }
}

PoseSnapshot MaterializePoseSnapshot(
    const PoseSnapshot& frozen, const std::vector<RigidTransform>& raw_poses,
    std::uint64_t frame_count) {
  ValidatePoseSnapshot(frozen);
  if (frozen.frame_count > raw_poses.size() || frame_count > raw_poses.size() ||
      (frozen.version && frame_count < frozen.frame_count)) {
    throw std::invalid_argument("Requested pose frontier is out of range");
  }
  for (const auto& pose : raw_poses) pose_math::Validate(pose);
  const auto correction = TailCorrection(frozen, raw_poses);
  if (frozen.version) {
    for (auto i = frozen.optimized_frame_count; i < frozen.frame_count; ++i) {
      const auto expected =
          MaterializeTailPose(correction, raw_poses[i], frozen.tail_policy);
      const bool equal =
          IsProjected(frozen.tail_policy)
              ? EqualProjectedPose(frozen.poses[i].parent_from_body(), expected)
              : pose_math::Equal(frozen.poses[i].parent_from_body(), expected);
      if (!equal) {
        throw std::invalid_argument(
            "Frozen pose tail does not follow its optimized anchor");
      }
    }
  } else {
    for (std::size_t i = 0; i < frozen.poses.size(); ++i) {
      if (!pose_math::Equal(frozen.poses[i].parent_from_body(), raw_poses[i])) {
        throw std::invalid_argument("Version zero must match raw poses");
      }
    }
  }
  PoseSnapshot result = frozen;
  result.frame_count = frame_count;
  if (!frozen.version) result.poses.clear();
  result.poses.reserve(frame_count);
  for (auto i = result.poses.size(); i < frame_count; ++i) {
    result.poses.push_back(
        {i, MaterializeTailPose(correction, raw_poses[i], frozen.tail_policy)});
  }
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
      update.poses.size() != update.optimized_frame_count) {
    throw std::invalid_argument(
        "Stale pose update or invalid optimized prefix");
  }
  if (previous.version == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("Pose version exhausted");
  }
  PoseSnapshot optimized;
  optimized.version = previous.version + 1;
  optimized.base_version = previous.version;
  optimized.frame_count = update.optimized_frame_count;
  optimized.optimized_frame_count = update.optimized_frame_count;
  optimized.parent_frame = PoseFrame::kMap;
  optimized.poses = update.poses;
  optimized.tail_policy = update.tail_policy;
  if (IsProjected(optimized.tail_policy)) {
    for (auto& pose : optimized.poses) {
      pose.map_from_body = pose_math::ProjectPose(pose.parent_from_body());
    }
  }
  return MaterializePoseSnapshot(optimized, raw_poses, raw_poses.size());
}

}  // namespace livo_atlas
