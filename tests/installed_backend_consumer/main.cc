#include "livo_atlas/backend/pose_graph.h"

#include <cmath>
#include <cstddef>
#include <vector>

int main() {
  std::vector<livo_atlas::RigidTransform> raw(3);
  raw[1].translation.x() = 1;
  raw[2].translation.x() = 0.2;
  livo_atlas::PoseGraph graph({});
  for (std::size_t i = 0; i < raw.size(); ++i) graph.AddKeyframe(i, raw[i]);
  graph.AddLoopConstraint({0, 2, {}, 0.01, 0.01});
  const auto snapshot =
      livo_atlas::MaterializePoseSnapshot({}, raw, raw.size());
  const auto result = graph.Optimize(snapshot);
  if (result.iteration_limit_reached || !std::isfinite(result.final_error) ||
      result.final_error >= result.initial_error ||
      result.update.optimized_frame_count != raw.size()) {
    return 1;
  }
  return 0;
}
