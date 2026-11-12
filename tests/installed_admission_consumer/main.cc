#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

#include "livo_atlas/mapping/pose_snapshot.h"
#ifdef EXPECT_GTSAM
#include "livo_atlas/backend/pose_graph.h"
#endif
#ifdef EXPECT_LOOP
#include "livo_atlas/backend/loop_closure.h"
#endif
#ifdef EXPECT_ADMISSION
#include "livo_atlas/backend/loop_admission.h"
#endif

int main() {
  using namespace livo_atlas;
  try {
    const std::vector<RigidTransform> raw(3);
    const auto poses = MaterializePoseSnapshot({}, raw, raw.size());
#ifdef EXPECT_GTSAM
    PoseGraph graph({});
    for (std::uint64_t i = 0; i < raw.size(); ++i) graph.AddKeyframe(i, raw[i]);
    const auto optimized = graph.Optimize(poses);
    if (optimized.update.poses.size() != 3 || optimized.final_error != 0)
      return 1;
#endif
#ifdef EXPECT_LOOP
    GeometrySubmap target;
    target.membership = {{0, 0, {0}}, {}, 2, OnlineSubmapState::kSealed};
    target.frame_count = 3;
    target.representation_revision = 1;
    auto target_data = std::make_shared<GeometryData>();
    target_data->sources.push_back({0, {}});
    target.data = target_data;
    auto source = target;
    source.membership.primary = {3, 2, {2}};
    auto source_data = std::make_shared<GeometryData>();
    source_data->sources.push_back({2, {}});
    source.data = source_data;
    if (VerifyLoop(target, source).status != LoopStatus::kInsufficientPoints)
      return 1;
#endif
#ifdef EXPECT_ADMISSION
    LoopAdmissionInput input;
    input.raw_poses = raw;
    input.poses = poses;
    LoopSubmapInfo info;
    info.membership = target.membership;
    info.frame_count = 3;
    input.submaps.push_back(info);
    const auto result = ProposeLoopUpdate(
        input, [](std::uint64_t, const PoseSnapshot&) -> GeometrySubmap {
          throw std::logic_error("No candidate should read geometry");
        });
    if (result.status != LoopAdmissionStatus::kNoCandidate || result.proposal)
      return 1;
#endif
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
