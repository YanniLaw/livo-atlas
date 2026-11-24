#include "livo_atlas/backend/loop_admission.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>
#include <stdexcept>
#include <utility>

#include <Eigen/Geometry>

#include "loop_closure/loop_internal.h"
#include "loop_closure/loop_selection.h"
#include "common/rotation_projection.h"
#include "mapping/pose_math.h"

namespace livo_atlas {
namespace {

bool Positive(double value) {
  return std::isfinite(value) && value > 0 && std::isfinite(1 / value);
}

bool Sigma(double value) {
  return Positive(value) && std::isfinite(1 / value / value) &&
         1 / value / value > 0;
}

bool SamePose(const RigidTransform& a, const RigidTransform& b) {
  return (a.rotation - b.rotation).cwiseAbs().maxCoeff() <= 1e-9 &&
         (a.translation - b.translation).stableNorm() <= 1e-9;
}

bool SameMembership(const OnlineSubmap& a, const OnlineSubmap& b) {
  return a.primary.id == b.primary.id &&
         a.primary.anchor_keyframe_id == b.primary.anchor_keyframe_id &&
         a.primary.keyframe_ids == b.primary.keyframe_ids &&
         a.support_keyframe_ids == b.support_keyframe_ids &&
         a.membership_revision == b.membership_revision && a.state == b.state;
}

const LoopSubmapInfo& FindInfo(const std::vector<LoopSubmapInfo>& submaps,
                               std::uint64_t id) {
  const auto found = std::find_if(
      submaps.begin(), submaps.end(),
      [id](const auto& info) { return info.membership.primary.id == id; });
  if (found == submaps.end())
    throw std::invalid_argument("Unknown loop submap");
  return *found;
}

void ValidateInput(const LoopAdmissionInput& input,
                   const LoopAdmissionOptions& options) {
  if (input.raw_poses.empty() ||
      input.raw_poses.size() != input.poses.frame_count ||
      input.poses.version != input.accepted_loops.size() ||
      (input.poses.version &&
       input.poses.tail_policy !=
           PoseTailPolicy::kProjectedLastOptimizedAnchor)) {
    throw std::invalid_argument("Invalid loop admission frontier or history");
  }
  (void)MaterializePoseSnapshot(input.poses, input.raw_poses,
                                input.poses.frame_count);
  // Version zero is the actual frontend prefix. An angle-only comparison can
  // miss a difference in the slightly nonorthogonal raw rotation matrices.
  if (input.poses.version == 0) {
    for (std::size_t i = 0; i < input.raw_poses.size(); ++i) {
      const auto& raw = input.raw_poses[i];
      const auto& current = input.poses.poses[i].parent_from_body();
      if ((raw.rotation.array() != current.rotation.array()).any() ||
          (raw.translation.array() != current.translation.array()).any()) {
        throw std::invalid_argument("Version zero differs from raw poses");
      }
    }
  }
  std::set<std::uint64_t> ids;
  for (const auto& info : input.submaps) {
    loop_internal::ValidateLoopSubmapInfo(info);
    if (!ids.insert(info.membership.primary.id).second ||
        info.pose_version != input.poses.version ||
        info.frame_count != input.poses.frame_count ||
        info.optimized_frame_count != input.poses.optimized_frame_count ||
        !SamePose(info.parent_from_submap,
                  input.poses.poses[info.membership.primary.anchor_keyframe_id]
                      .parent_from_body())) {
      throw std::invalid_argument("Loop metadata does not match the pose view");
    }
  }
  (void)FindInfo(input.submaps, input.query_id);
  std::set<std::pair<std::uint64_t, std::uint64_t>> endpoints;
  for (const auto& loop : input.accepted_loops) {
    const auto& c = loop.constraint;
    const auto& target = FindInfo(input.submaps, loop.target.primary.id);
    const auto& source = FindInfo(input.submaps, loop.source.primary.id);
    if (!SameMembership(loop.target, target.membership) ||
        !SameMembership(loop.source, source.membership) ||
        loop.target.primary.id >= loop.source.primary.id ||
        loop.target.primary.keyframe_ids.back() >=
            loop.source.primary.keyframe_ids.front() ||
        c.from_id != loop.target.primary.anchor_keyframe_id ||
        c.to_id != loop.source.primary.anchor_keyframe_id ||
        c.from_id >= c.to_id || c.to_id - c.from_id <= 1 ||
        c.rotation_sigma_rad != options.loop_rotation_sigma_rad ||
        c.translation_sigma_m != options.loop_translation_sigma_m ||
        !endpoints.emplace(c.from_id, c.to_id).second) {
      throw std::invalid_argument("Invalid accepted loop history");
    }
    pose_math::Validate(c.from_body_from_to_body);
  }
}

GeometrySubmap ReadGeometry(const OnlineSubmap& membership,
                            const PoseSnapshot& poses,
                            const LoopGeometryReader& reader) {
  auto geometry = reader(membership.primary.id, poses);
  const auto& anchor =
      poses.poses[membership.primary.anchor_keyframe_id].parent_from_body();
  if (!SameMembership(geometry.membership, membership) ||
      geometry.pose_version != poses.version ||
      geometry.frame_count != poses.frame_count ||
      geometry.optimized_frame_count != poses.optimized_frame_count ||
      geometry.parent_frame != poses.parent_frame || !geometry.data ||
      !SamePose(geometry.parent_from_submap, anchor) ||
      geometry.data->sources.size() != membership.primary.keyframe_ids.size()) {
    throw std::invalid_argument(
        "Loop reader returned a different geometry view");
  }
  const auto inverse = pose_math::Inverse(anchor);
  for (std::size_t i = 0; i < geometry.data->sources.size(); ++i) {
    const auto id = membership.primary.keyframe_ids[i];
    const auto expected =
        pose_math::Compose(inverse, poses.poses[id].parent_from_body());
    if (geometry.data->sources[i].keyframe_id != id ||
        !SamePose(geometry.data->sources[i].submap_from_body, expected)) {
      throw std::invalid_argument("Loop geometry contains stale source poses");
    }
  }
  return geometry;
}

bool ResidualsPass(const PoseGraphResult& trial, std::size_t accepted_count,
                   const LoopAdmissionOptions& options) {
  for (const auto* residuals :
       {&trial.odometry_residuals, &trial.loop_residuals}) {
    for (const auto& residual : *residuals) {
      if (residual.final_whitened_norm > options.max_factor_residual)
        return false;
    }
  }
  for (std::size_t i = 0; i < accepted_count; ++i) {
    const auto& residual = trial.loop_residuals[i];
    if (residual.final_whitened_norm - residual.initial_whitened_norm >
        options.max_accepted_residual_increase)
      return false;
  }
  return true;
}

bool CorrectionsPass(const PoseUpdate& update, const PoseSnapshot& current,
                     const LoopAdmissionOptions& options) {
  for (const auto& pose : update.poses) {
    const auto& before = current.poses[pose.id].parent_from_body();
    const Eigen::Vector3d translation =
        pose.parent_from_body().translation - before.translation;
    // Rounded frontend rotations must not scale a physical correction bound.
    const Eigen::Matrix3d rotation =
        ProjectRotation(before.rotation).transpose() *
        pose.parent_from_body().rotation;
    const double angle = Eigen::AngleAxisd(rotation).angle();
    if (!translation.allFinite() || !std::isfinite(angle) ||
        translation.stableNorm() > options.max_pose_correction_m ||
        angle >
            options.max_pose_correction_deg * (3.14159265358979323846 / 180)) {
      return false;
    }
  }
  return true;
}

bool CheckGeometry(const AcceptedLoop& loop, bool new_constraint,
                   const PoseSnapshot& preview,
                   const LoopGeometryReader& reader,
                   const LoopAdmissionOptions& options,
                   std::vector<LoopGeometryReport>* reports) {
  const auto target = ReadGeometry(loop.target, preview, reader);
  const auto source = ReadGeometry(loop.source, preview, reader);
  reports->push_back({loop.target.primary.id, loop.source.primary.id,
                      new_constraint, std::nullopt, std::nullopt});
  auto& report = reports->back();
  report.measurement = EvaluateLoopGeometry(
      target, source, loop.constraint.from_body_from_to_body,
      options.registration);
  if (report.measurement->status != LoopStatus::kVerified) return false;
  const auto prediction = pose_math::Compose(
      pose_math::Inverse(target.parent_from_submap), source.parent_from_submap);
  report.prediction =
      EvaluateLoopGeometry(target, source, prediction, options.registration);
  return report.prediction->status == LoopStatus::kVerified;
}

}  // namespace

void ValidateLoopAdmissionOptions(const LoopAdmissionOptions& o) {
  ValidateLoopSearchOptions(o.search);
  ValidateLoopRegistrationOptions(o.registration);
  ValidateGeometryOptions(o.geometry);
  if (!o.geometry.enabled)
    throw std::invalid_argument("Loop geometry must be enabled");
  (void)Optimizer(o.graph);
  if (o.search.max_candidates > 5 || !Positive(o.graph.loop_huber_delta) ||
      !Sigma(o.loop_translation_sigma_m) || !Sigma(o.loop_rotation_sigma_rad) ||
      !Positive(o.max_hypothesis_translation_m) ||
      !Positive(o.max_hypothesis_rotation_deg) ||
      o.max_hypothesis_rotation_deg > 180 || !Positive(o.max_factor_residual) ||
      !std::isfinite(o.max_accepted_residual_increase) ||
      o.max_accepted_residual_increase < 0 ||
      !Positive(o.max_pose_correction_m) ||
      !Positive(o.max_pose_correction_deg) || o.max_pose_correction_deg > 180 ||
      !o.max_keyframes || !o.max_loops || !o.max_pending_keyframes ||
      o.registration.min_points > o.registration.max_voxels ||
      o.registration.min_inliers > o.registration.max_voxels ||
      o.registration.min_normal_pairs > o.registration.max_voxels) {
    throw std::invalid_argument("Invalid loop admission options");
  }
}

LoopAdmissionResult ProposeLoopUpdate(const LoopAdmissionInput& input,
                                      const LoopGeometryReader& reader,
                                      const LoopAdmissionOptions& options) {
  ValidateLoopAdmissionOptions(options);
  if (!reader) throw std::invalid_argument("Missing loop geometry reader");
  ValidateInput(input, options);
  LoopAdmissionResult result;
  if (input.raw_poses.size() > options.max_keyframes ||
      input.accepted_loops.size() >= options.max_loops) {
    result.status = LoopAdmissionStatus::kResourceLimit;
    return result;
  }
  auto submaps = input.submaps;
  for (auto& info : submaps) {
    info.parent_from_submap =
        input.poses.poses[info.membership.primary.anchor_keyframe_id]
            .parent_from_body();
    info.pose_version = input.poses.version;
    info.frame_count = input.poses.frame_count;
    info.optimized_frame_count = input.poses.optimized_frame_count;
  }
  const auto& query = FindInfo(submaps, input.query_id);
  const auto candidates = FindLoopCandidates(query, submaps, options.search);
  if (candidates.empty()) return result;
  std::vector<loop_internal::LoopHypothesis> hypotheses;
  for (const auto& candidate : candidates) {
    const auto& target_info = FindInfo(submaps, candidate.target_id);
    const auto target =
        ReadGeometry(target_info.membership, input.poses, reader);
    const auto source = ReadGeometry(query.membership, input.poses, reader);
    result.candidates.push_back(
        {candidate, VerifyLoop(target, source, options.registration)});
    const auto& report = result.candidates.back();
    if (report.verification.status == LoopStatus::kVerified) {
      AcceptedLoop loop;
      loop.target = target_info.membership;
      loop.source = query.membership;
      loop.constraint = {loop.target.primary.anchor_keyframe_id,
                         loop.source.primary.anchor_keyframe_id,
                         *report.verification.target_from_source,
                         options.loop_rotation_sigma_rad,
                         options.loop_translation_sigma_m};
      hypotheses.push_back({std::move(loop), report});
    }
  }
  const auto selection = loop_internal::SelectLoopCandidate(
      hypotheses, input.accepted_loops, input.poses, options);
  result.status = selection.status;
  if (!selection.index) return result;
  auto& chosen = hypotheses[*selection.index];
  if (!options.pose_graph_enabled) {
    result.status = LoopAdmissionStatus::kDetected;
    result.detected_target_id = chosen.loop.target.primary.id;
    return result;
  }
  const auto build_start = std::chrono::steady_clock::now();
  Optimizer graph(options.graph);
  for (std::size_t i = 0; i < input.raw_poses.size(); ++i)
    graph.AddKeyframe(i, input.raw_poses[i]);
  for (const auto& accepted : input.accepted_loops)
    graph.AddLoopConstraint(accepted.constraint);
  graph.AddLoopConstraint(chosen.loop.constraint);
  const double input_build_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    build_start)
          .count();
  PoseGraphResult trial;
  try {
    trial = graph.Optimize(input.poses);
  } catch (const PoseGraphOptimizationError& error) {
    result.status = LoopAdmissionStatus::kOptimizationRejected;
    result.optimization_error = error.what();
    return result;
  }
  result.trial = {trial.initial_error,
                  trial.final_error,
                  trial.iterations,
                  trial.iteration_limit_reached,
                  trial.odometry_residuals,
                  trial.loop_residuals,
                  trial.graph_build_seconds + input_build_seconds,
                  trial.solve_seconds};
  if (trial.iteration_limit_reached) {
    result.status = LoopAdmissionStatus::kOptimizationRejected;
    return result;
  }
  if (!ResidualsPass(trial, input.accepted_loops.size(), options)) {
    result.status = LoopAdmissionStatus::kConstraintResidual;
    return result;
  }
  if (!CorrectionsPass(trial.update, input.poses, options)) {
    result.status = LoopAdmissionStatus::kPoseCorrection;
    return result;
  }
  const auto preview =
      MakePoseSnapshot(trial.update, input.poses, input.raw_poses);
  for (const auto& accepted : input.accepted_loops) {
    if (!CheckGeometry(accepted, false, preview, reader, options,
                       &result.geometry_checks)) {
      result.status = LoopAdmissionStatus::kGeometryChanged;
      return result;
    }
  }
  if (!CheckGeometry(chosen.loop, true, preview, reader, options,
                     &result.geometry_checks)) {
    result.status = LoopAdmissionStatus::kGeometryChanged;
    return result;
  }
  result.status = LoopAdmissionStatus::kProposed;
  result.proposal = {std::move(chosen.loop),
                     std::move(chosen.report.verification),
                     std::move(trial.update)};
  return result;
}

const char* LoopAdmissionStatusName(LoopAdmissionStatus status) {
  switch (status) {
    case LoopAdmissionStatus::kNoCandidate:
      return "no_candidate";
    case LoopAdmissionStatus::kGeometryRejected:
      return "geometry_rejected";
    case LoopAdmissionStatus::kAmbiguous:
      return "ambiguous";
    case LoopAdmissionStatus::kCorrelated:
      return "correlated";
    case LoopAdmissionStatus::kResourceLimit:
      return "resource_limit";
    case LoopAdmissionStatus::kOptimizationRejected:
      return "optimization_rejected";
    case LoopAdmissionStatus::kConstraintResidual:
      return "constraint_residual";
    case LoopAdmissionStatus::kPoseCorrection:
      return "pose_correction";
    case LoopAdmissionStatus::kGeometryChanged:
      return "geometry_changed";
    case LoopAdmissionStatus::kProposed:
      return "proposed";
    case LoopAdmissionStatus::kDetected:
      return "detected";
  }
  throw std::invalid_argument("Unknown loop admission status");
}

}  // namespace livo_atlas
