#ifndef LIVO_ATLAS_BACKEND_POSE_GRAPH_H_
#define LIVO_ATLAS_BACKEND_POSE_GRAPH_H_

#include <cstddef>
#include <cstdint>
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

struct PoseGraphResult {
  PoseUpdate update;
  double initial_error = 0;
  double final_error = 0;
  std::size_t iterations = 0;
  bool iteration_limit_reached = false;
};

// A graph has one serial owner. Optimize creates a candidate without changing
// the graph or the supplied snapshot; its caller decides whether to commit it.
class PoseGraph {
 public:
  explicit PoseGraph(PoseGraphOptions options);

  void AddKeyframe(std::uint64_t id, const RigidTransform& odom_from_body);
  void AddLoopConstraint(const LoopConstraint& constraint);
  PoseGraphResult Optimize(const PoseSnapshot& current) const;

 private:
  PoseGraphOptions options_;
  std::vector<RigidTransform> raw_poses_;
  std::vector<LoopConstraint> constraints_;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_BACKEND_POSE_GRAPH_H_
