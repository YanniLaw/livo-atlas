#ifndef LIVO_ATLAS_TESTS_GAUSSIAN_RENDER_METADATA_TEST_HELPERS_H_
#define LIVO_ATLAS_TESTS_GAUSSIAN_RENDER_METADATA_TEST_HELPERS_H_

#include "livo_atlas/gaussian/gaussian_map.h"

namespace livo_atlas::test {

inline GaussianRenderMetadata RenderMetadata(
    const GaussianContentId& content,
    const std::vector<std::uint64_t>& frames) {
  GaussianRenderMetadata result;
  result.content = content;
  for (const auto id : frames)
    result.extents.push_back({id, {0, 0, 4}, {0, 0, 4}, .03});
  result.camera.keyframe_id = frames.back();
  result.camera.width = 8;
  result.camera.height = 6;
  result.camera.K << 4, 0, 3.5, 0, 4, 2.5, 0, 0, 1;
  result.camera.capture_time = 100 + frames.back();
  return result;
}

}  // namespace livo_atlas::test
#endif  // LIVO_ATLAS_TESTS_GAUSSIAN_RENDER_METADATA_TEST_HELPERS_H_
