#ifndef LIVO_ATLAS_OPTIMIZATION_OPTIMIZER_H_
#define LIVO_ATLAS_OPTIMIZATION_OPTIMIZER_H_

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "livo_atlas/mapping/pose_snapshot.h"
#include "livo_atlas/sensor/extrinsics.h"

namespace livo_atlas {

struct PoseGraphOptions {
  double prior_rotation_sigma_rad = 0.001;
  double prior_translation_sigma_m = 0.001;
  double odometry_rotation_sigma_rad = 0.05;
  double odometry_translation_sigma_m = 0.1;
  int max_iterations = 100;
  // Zero preserves Gaussian loops; positive values use block Huber weighting.
  double loop_huber_delta = 0;
};

// Valid inputs can still produce a numerically unsolvable trial graph.
class PoseGraphOptimizationError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// The measurement maps coordinates from the later body frame into the earlier
// one. Standard deviations are in radians and meters, respectively.
struct LoopConstraint {
  std::uint64_t from_id = 0;
  std::uint64_t to_id = 0;
  RigidTransform from_body_from_to_body;
  double rotation_sigma_rad = 0;
  double translation_sigma_m = 0;
};

struct FactorResidual {
  std::uint64_t from_id = 0;
  std::uint64_t to_id = 0;
  // These norms use the original sigmas, without robust downweighting.
  double initial_whitened_norm = 0;
  double final_whitened_norm = 0;
  double final_rotation_rad = 0;
  double final_translation_m = 0;
};

struct PoseGraphResult {
  PoseUpdate update;
  double initial_error = 0;
  double final_error = 0;
  std::size_t iterations = 0;
  bool iteration_limit_reached = false;
  std::vector<FactorResidual> odometry_residuals;
  std::vector<FactorResidual> loop_residuals;
  double graph_build_seconds = 0;
  double solve_seconds = 0;
};

// A graph has one serial owner. Optimize creates a candidate without changing
// the graph or the supplied snapshot; its caller decides whether to commit it.
class Optimizer {
 public:
  explicit Optimizer(PoseGraphOptions options);

  void AddKeyframe(std::uint64_t id, const RigidTransform& odom_from_body);
  void AddLoopConstraint(const LoopConstraint& constraint);
  PoseGraphResult Optimize(const PoseSnapshot& current) const;

 private:
  PoseGraphOptions options_;
  std::vector<RigidTransform> raw_poses_;
  std::vector<LoopConstraint> constraints_;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_OPTIMIZATION_OPTIMIZER_H_
