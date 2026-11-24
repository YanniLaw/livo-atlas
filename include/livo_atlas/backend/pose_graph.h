#ifndef LIVO_ATLAS_BACKEND_POSE_GRAPH_H_
#define LIVO_ATLAS_BACKEND_POSE_GRAPH_H_

#include "livo_atlas/optimization/optimizer.h"

namespace livo_atlas {

// Legacy SDK solver entry point. The running graph lives in pose_graph.
class PoseGraph {
 public:
  explicit PoseGraph(PoseGraphOptions options);

  void AddKeyframe(std::uint64_t id, const RigidTransform& odom_from_body);
  void AddLoopConstraint(const LoopConstraint& constraint);
  PoseGraphResult Optimize(const PoseSnapshot& current) const;

 private:
  Optimizer optimizer_;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_BACKEND_POSE_GRAPH_H_
