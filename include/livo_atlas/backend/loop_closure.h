#ifndef LIVO_ATLAS_BACKEND_LOOP_CLOSURE_H_
#define LIVO_ATLAS_BACKEND_LOOP_CLOSURE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "livo_atlas/mapping/geometry_submap.h"

namespace livo_atlas {

struct LoopSearchOptions {
  std::uint64_t min_submap_gap = 3;
  std::uint64_t min_keyframe_gap = 50;
  double min_time_gap_s = 30;
  double search_radius_m = 15;
  std::size_t max_candidates = 5;
};

// Metadata only: candidate retrieval never retains a cloud or image.
struct LoopSubmapInfo {
  OnlineSubmap membership;
  double first_timestamp = 0;
  double last_timestamp = 0;
  RigidTransform parent_from_submap;
  std::uint64_t pose_version = 0;
  std::uint64_t frame_count = 0;
  std::uint64_t optimized_frame_count = 0;
};

struct LoopCandidate {
  std::uint64_t target_id = 0;
  std::uint64_t source_id = 0;
  double anchor_distance_m = 0;
};

void ValidateLoopSearchOptions(const LoopSearchOptions& options);
std::vector<LoopCandidate> FindLoopCandidates(
    const LoopSubmapInfo& query, const std::vector<LoopSubmapInfo>& history,
    const LoopSearchOptions& options = {});

struct LoopRegistrationOptions {
  double voxel_size_m = 0.5;
  std::size_t min_points = 100;
  std::size_t max_points = 20000;
  std::size_t max_voxels = 100000;
  int neighbors = 20;
  int max_iterations = 50;
  int max_optimizer_iterations = 10;
  double correspondence_distance_m = 2;
  double translation_epsilon = 1e-4;
  double rotation_epsilon = 1e-4;
  double match_distance_m = 0.5;
  std::size_t min_inliers = 100;
  double min_coverage = 0.3;
  double max_rmse_m = 0.2;
  double max_p95_m = 0.3;
  double max_cycle_translation_m = 0.1;
  double max_cycle_rotation_deg = 2;
  double max_correction_m = 3;
  double max_correction_deg = 20;
  std::size_t min_normal_pairs = 100;
  double min_eigenvalue_ratio = 1e-4;
};

enum class LoopStatus {
  kVerified,
  kInsufficientPoints,
  kResourceLimit,
  kNotConverged,
  kIterationLimit,
  kLowOverlap,
  kHighResidual,
  kInconsistentTransform,
  kLargeCorrection,
  kDegenerate,
};

struct LoopProvenance {
  std::uint64_t submap_id = 0;
  std::uint64_t anchor_keyframe_id = 0;
  std::uint64_t membership_revision = 0;
  std::uint64_t representation_revision = 0;
};

struct LoopMetrics {
  std::size_t inliers = 0;
  std::size_t normal_pairs = 0;
  double source_coverage = 0;
  double target_coverage = 0;
  std::optional<double> rmse_m;
  std::optional<double> p95_m;
  std::optional<double> eigenvalue_ratio;
};

struct LoopGeometryCheck {
  LoopStatus status = LoopStatus::kInsufficientPoints;
  LoopMetrics forward, reverse;
};

struct LoopAlignment {
  int iterations = 0;
  bool converged = false;
  std::optional<RigidTransform> transform;
  std::optional<LoopMetrics> metrics;
};

struct LoopTransformError {
  double translation_m = 0;
  double rotation_deg = 0;
};

struct LoopVerification {
  LoopStatus status = LoopStatus::kNotConverged;
  LoopProvenance target, source;
  std::uint64_t pose_version = 0;
  std::uint64_t frame_count = 0;
  std::uint64_t optimized_frame_count = 0;
  std::size_t target_points = 0, source_points = 0;
  RigidTransform initial_target_from_source;
  LoopAlignment forward, reverse;
  // Target/source cycles, then forward/reverse initial corrections.
  std::optional<std::array<LoopTransformError, 4>> transform_errors;
  // Only supplied for kVerified; the measurement maps source into target.
  std::optional<RigidTransform> target_from_source;
  double elapsed_seconds = 0;
};

void ValidateLoopRegistrationOptions(const LoopRegistrationOptions& options);
const char* LoopStatusName(LoopStatus status);
LoopVerification VerifyLoop(const GeometrySubmap& target,
                            const GeometrySubmap& source,
                            const LoopRegistrationOptions& options = {});
// Checks the supplied measurement and its inverse without fitting a new one.
LoopGeometryCheck EvaluateLoopGeometry(
    const GeometrySubmap& target, const GeometrySubmap& source,
    const RigidTransform& target_from_source,
    const LoopRegistrationOptions& options = {});

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_BACKEND_LOOP_CLOSURE_H_
