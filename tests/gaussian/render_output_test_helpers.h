#ifndef LIVO_ATLAS_TESTS_GAUSSIAN_RENDER_OUTPUT_TEST_HELPERS_H_
#define LIVO_ATLAS_TESTS_GAUSSIAN_RENDER_OUTPUT_TEST_HELPERS_H_

#include <memory>
#include <utility>

#include "gaussian/render_test_helpers.h"
#include "livo_atlas/gaussian/gaussian_render_snapshot.h"

namespace livo_atlas::test {

inline GaussianRenderSnapshot ReadyRenderSnapshot() {
  const auto request = RenderRequest();
  auto render = std::make_shared<GaussianRender>(DecodeGaussianRenderReply(
      RenderReply(EncodeGaussianRenderRequest(request)), request));
  GaussianRenderSnapshot result;
  result.session_id = request.session_id;
  result.generation = request.generation;
  result.map_sequence = request.map_sequence;
  result.pose_version = request.pose_version;
  result.frame_count = request.frame_count;
  result.optimized_frame_count = request.optimized_frame_count;
  result.parent_frame = request.parent_frame;
  result.policy = request.policy;
  result.status = GaussianRenderStatus::kReady;
  result.reason.clear();
  result.render = std::move(render);
  return result;
}

}  // namespace livo_atlas::test
#endif  // LIVO_ATLAS_TESTS_GAUSSIAN_RENDER_OUTPUT_TEST_HELPERS_H_
