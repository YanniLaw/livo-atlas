#ifndef LIVO_ATLAS_BACKEND_LOOP_ADMISSION_H_
#define LIVO_ATLAS_BACKEND_LOOP_ADMISSION_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "livo_atlas/backend/loop_closure.h"
#include "livo_atlas/backend/pose_graph.h"
#include "livo_atlas/mapping/geometry_submap.h"

namespace livo_atlas {

struct AcceptedLoop {
  LoopConstraint constraint;
  OnlineSubmap target, source;
};

struct LoopAdmissionInput {
  PoseSnapshot poses;
  std::vector<RigidTransform> raw_poses;
  std::vector<LoopSubmapInfo> submaps;
  std::uint64_t query_id = 0;
  std::vector<AcceptedLoop> accepted_loops;
};

struct LoopAdmissionOptions {
  LoopAdmissionOptions() {
    graph.loop_huber_delta = 1.345;
    geometry.enabled = true;
  }

  LoopSearchOptions search;
  LoopRegistrationOptions registration;
  PoseGraphOptions graph;
  double loop_translation_sigma_m = 0.2;
  double loop_rotation_sigma_rad = 0.05235987755982989;
  double max_hypothesis_translation_m = 0.5;
  double max_hypothesis_rotation_deg = 5;
  std::uint64_t correlated_submap_gap = 2;
  double max_factor_residual = 3;
  double max_accepted_residual_increase = 0.5;
  double max_pose_correction_m = 3;
  double max_pose_correction_deg = 20;
  std::size_t max_keyframes = 5000;
  std::size_t max_loops = 200;
  bool pose_graph_enabled = true;
  GeometryOptions geometry;
  std::size_t max_pending_keyframes = 64;
};

enum class LoopAdmissionStatus {
  kNoCandidate,
  kGeometryRejected,
  kAmbiguous,
  kCorrelated,
  kResourceLimit,
  kOptimizationRejected,
  kConstraintResidual,
  kPoseCorrection,
  kGeometryChanged,
  kProposed,
  kDetected,
};

struct LoopCandidateReport {
  LoopCandidate candidate;
  LoopVerification verification;
};

struct LoopGeometryReport {
  std::uint64_t target_id = 0, source_id = 0;
  bool new_constraint = false;
  std::optional<LoopGeometryCheck> measurement, prediction;
};

struct LoopTrialDiagnostics {
  double initial_error = 0, final_error = 0;
  std::size_t iterations = 0;
  bool iteration_limit_reached = false;
  std::vector<FactorResidual> odometry_residuals, loop_residuals;
  double graph_build_seconds = 0;
  double solve_seconds = 0;
};

struct LoopProposal {
  AcceptedLoop loop;
  LoopVerification verification;
  PoseUpdate update;
};

struct LoopAdmissionResult {
  LoopAdmissionStatus status = LoopAdmissionStatus::kNoCandidate;
  std::vector<LoopCandidateReport> candidates;
  std::optional<LoopTrialDiagnostics> trial;
  std::vector<LoopGeometryReport> geometry_checks;
  // Rejected trials never expose a committable update.
  std::optional<LoopProposal> proposal;
  std::optional<std::string> optimization_error;
  std::optional<std::uint64_t> detected_target_id;
};

using LoopGeometryReader = std::function<GeometrySubmap(
    std::uint64_t submap_id, const PoseSnapshot& poses)>;

void ValidateLoopAdmissionOptions(const LoopAdmissionOptions& options);
const char* LoopAdmissionStatusName(LoopAdmissionStatus status);
// The reader owns its cache and must honor the requested pose view. This call
// neither writes assets nor changes the input or a live frontend's state.
LoopAdmissionResult ProposeLoopUpdate(const LoopAdmissionInput& input,
                                      const LoopGeometryReader& geometry_reader,
                                      const LoopAdmissionOptions& options = {});

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_BACKEND_LOOP_ADMISSION_H_
