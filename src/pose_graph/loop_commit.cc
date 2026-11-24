#include "pose_graph/loop_commit.h"

#include <cmath>
#include <stdexcept>

#include "io/loop_admission_io.h"

namespace livo_atlas {
namespace {
bool SamePose(const RigidTransform& a, const RigidTransform& b) {
  return (a.rotation - b.rotation).cwiseAbs().maxCoeff() <= 1e-9 &&
         (a.translation - b.translation).cwiseAbs().maxCoeff() <= 1e-9;
}

bool SameLoop(const AcceptedLoop& a, const AcceptedLoop& b) {
  return loop_admission_io::SameMembership(a.target, b.target) &&
         loop_admission_io::SameMembership(a.source, b.source) &&
         a.constraint.from_id == b.constraint.from_id &&
         a.constraint.to_id == b.constraint.to_id &&
         a.constraint.rotation_sigma_rad == b.constraint.rotation_sigma_rad &&
         a.constraint.translation_sigma_m == b.constraint.translation_sigma_m &&
         SamePose(a.constraint.from_body_from_to_body,
                  b.constraint.from_body_from_to_body);
}

void ValidateProposal(const LoopTask& task, const LoopAdmissionResult& result) {
  const auto& input = task.input;
  const auto n = input.poses.frame_count;
  if (!result.proposal || result.status != LoopAdmissionStatus::kProposed ||
      !n || input.raw_poses.size() != n || task.timestamps.size() != n ||
      task.frames.frame_count() != n)
    throw std::invalid_argument("Invalid loop proposal or task frontier");
  const auto& proposal = *result.proposal;
  const auto& update = proposal.update;
  if (update.base_version != input.poses.version ||
      update.optimized_frame_count != n ||
      update.tail_policy != PoseTailPolicy::kProjectedLastOptimizedAnchor)
    throw std::invalid_argument(
        "Loop proposal does not cover its dispatch frontier");
  (void)MakePoseSnapshot(update, input.poses, input.raw_poses);
  const auto& loop = proposal.loop;
  const auto& c = loop.constraint;
  if (loop.target.primary.id >= loop.source.primary.id ||
      loop.source.primary.id >= input.submaps.size() ||
      loop.source.primary.id != input.query_id ||
      !loop_admission_io::SameMembership(
          loop.target, input.submaps[loop.target.primary.id].membership) ||
      !loop_admission_io::SameMembership(
          loop.source, input.submaps[loop.source.primary.id].membership) ||
      c.from_id != loop.target.primary.anchor_keyframe_id ||
      c.to_id != loop.source.primary.anchor_keyframe_id ||
      c.to_id <= c.from_id || c.to_id - c.from_id <= 1 ||
      !std::isfinite(c.rotation_sigma_rad) || c.rotation_sigma_rad <= 0 ||
      !std::isfinite(c.translation_sigma_m) || c.translation_sigma_m <= 0 ||
      proposal.verification.status != LoopStatus::kVerified ||
      !proposal.verification.target_from_source ||
      !SamePose(c.from_body_from_to_body,
                *proposal.verification.target_from_source))
    throw std::invalid_argument(
        "Loop proposal constraint or provenance is invalid");
}
}  // namespace

LoopCommitStatus ValidateLoopCommit(
    const LoopTask& task, const LoopAdmissionResult& result,
    const LoopAdmissionInput& current,
    const std::vector<double>& current_timestamps) {
  ValidateProposal(task, result);
  const auto& previous = task.input;
  const auto& a = previous.poses;
  const auto& b = current.poses;
  const auto n = a.frame_count;
  if (b.version != a.version || b.base_version != a.base_version ||
      b.optimized_frame_count != a.optimized_frame_count ||
      b.parent_frame != a.parent_frame || b.tail_policy != a.tail_policy ||
      b.frame_count < n || current.raw_poses.size() != b.frame_count ||
      b.poses.size() != b.frame_count ||
      current_timestamps.size() != b.frame_count ||
      current.submaps.size() < previous.submaps.size() ||
      current.accepted_loops.size() != previous.accepted_loops.size())
    return LoopCommitStatus::kStale;
  for (std::size_t id = 0; id < n; ++id) {
    if (current_timestamps[id] != task.timestamps[id] ||
        (current.raw_poses[id].rotation.array() !=
         previous.raw_poses[id].rotation.array())
            .any() ||
        (current.raw_poses[id].translation.array() !=
         previous.raw_poses[id].translation.array())
            .any() ||
        b.poses[id].id != a.poses[id].id ||
        !SamePose(b.poses[id].parent_from_body(),
                  a.poses[id].parent_from_body()))
      return LoopCommitStatus::kStale;
  }
  for (std::size_t i = 0; i < previous.submaps.size(); ++i) {
    const auto& old = previous.submaps[i];
    const auto& now = current.submaps[i];
    if (!loop_admission_io::SameMembership(old.membership, now.membership) ||
        old.first_timestamp != now.first_timestamp ||
        old.last_timestamp != now.last_timestamp ||
        now.pose_version != b.version || now.frame_count != b.frame_count ||
        now.optimized_frame_count != b.optimized_frame_count ||
        !SamePose(old.parent_from_submap, now.parent_from_submap))
      return LoopCommitStatus::kStale;
  }
  for (std::size_t i = 0; i < previous.accepted_loops.size(); ++i)
    if (!SameLoop(previous.accepted_loops[i], current.accepted_loops[i]))
      return LoopCommitStatus::kStale;
  return LoopCommitStatus::kEligible;
}

LoopCommitCheck CheckLoopCommit(const LoopTask& task,
                                const LoopAdmissionResult& result,
                                const LoopAdmissionInput& current,
                                const std::vector<double>& current_timestamps) {
  if (ValidateLoopCommit(task, result, current, current_timestamps) ==
      LoopCommitStatus::kStale)
    return {};
  return {LoopCommitStatus::kEligible,
          MakePoseSnapshot(result.proposal->update, current.poses,
                           current.raw_poses)};
}
}  // namespace livo_atlas
