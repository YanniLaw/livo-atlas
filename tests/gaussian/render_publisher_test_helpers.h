#ifndef LIVO_ATLAS_TESTS_GAUSSIAN_RENDER_PUBLISHER_TEST_HELPERS_H_
#define LIVO_ATLAS_TESTS_GAUSSIAN_RENDER_PUBLISHER_TEST_HELPERS_H_

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "gaussian/render_output_test_helpers.h"

namespace livo_atlas::test {

inline GaussianRenderSnapshot PublicationSnapshot() {
  auto result = ReadyRenderSnapshot();
  result.producer_id = std::string(32, 'b');
  result.revision = 1;
  return result;
}

// Literal expectations catch channel swaps, half-pixel shifts, inverted poses
// and truncation of job IDs above double's exact integer range.
template <typename Message>
void CheckPublishedRender(const Message& message) {
  EXPECT_EQ(message.producer_id, std::string(32, 'b'));
  EXPECT_EQ(message.session_id, std::string(32, 'a'));
  EXPECT_EQ(message.status, Message::READY);
  EXPECT_TRUE(message.reason.empty());
  EXPECT_EQ(message.job_id, 9007199254740993ULL);
  EXPECT_EQ(message.generation, 4u);
  EXPECT_EQ(message.map_sequence, 9u);
  EXPECT_EQ(message.parent_frame, "camera_init");
  EXPECT_EQ(message.policy, "temporal");
  EXPECT_EQ(message.header.frame_id, "atlas/gaussian_optical");
  EXPECT_EQ(message.header, message.image.header);
  EXPECT_EQ(message.image.width, 8u);
  EXPECT_EQ(message.image.height, 6u);
  EXPECT_EQ(message.image.step, 24u);
  EXPECT_EQ(message.image.encoding, "rgb8");
  EXPECT_FALSE(message.image.is_bigendian);
  EXPECT_EQ(message.image.data.size(), 144u);
  ASSERT_GE(message.image.data.size(), 3u);
  EXPECT_EQ(message.image.data[0], 12u);
  EXPECT_EQ(message.image.data[1], 34u);
  EXPECT_EQ(message.image.data[2], 56u);
  EXPECT_DOUBLE_EQ(message.k[0], 4);
  EXPECT_DOUBLE_EQ(message.k[2], 3.5);
  EXPECT_DOUBLE_EQ(message.k[5], 2.5);
  EXPECT_DOUBLE_EQ(message.k[8], 1);
  EXPECT_DOUBLE_EQ(message.capture_time, 100.01);
  EXPECT_DOUBLE_EQ(message.parent_from_camera.position.x, 5);
  EXPECT_DOUBLE_EQ(message.parent_from_camera.position.y, 0);
  EXPECT_DOUBLE_EQ(message.parent_from_camera.orientation.w, 1);
  EXPECT_EQ(message.query_keyframe_id, 5u);
  EXPECT_EQ(message.query_submap_id, 2u);
  ASSERT_EQ(message.sources.size(), 2u);
  EXPECT_EQ(message.sources[0].submap_id, 1u);
  EXPECT_EQ(message.sources[0].binding_generation, 2u);
  EXPECT_EQ(message.sources[0].source_generation, 2u);
  EXPECT_EQ(message.sources[0].content_job_id, 11u);
  EXPECT_EQ(message.sources[0].membership_revision, 3u);
  EXPECT_EQ(message.sources[0].asset_id, 2u);
  EXPECT_EQ(message.sources[0].manifest_sha256, std::string(64, 'b'));
  EXPECT_EQ(message.sources[0].frame_ids, (std::vector<std::uint64_t>{2, 3}));
  EXPECT_EQ(message.sources[0].gaussian_count, 2u);
  EXPECT_FALSE(message.sources[0].resident);
}

template <typename Message>
void CheckClearedRender(const Message& message) {
  EXPECT_EQ(message.header.frame_id, "atlas/gaussian_optical");
  EXPECT_EQ(message.header, message.image.header);
  EXPECT_EQ(message.image.width, 0u);
  EXPECT_EQ(message.image.height, 0u);
  EXPECT_EQ(message.image.step, 0u);
  EXPECT_TRUE(message.image.data.empty());
  EXPECT_TRUE(message.sources.empty());
  EXPECT_EQ(message.job_id, 0u);
  EXPECT_EQ(message.query_keyframe_id, 0u);
  EXPECT_EQ(message.query_submap_id, 0u);
  EXPECT_EQ(message.changed_pixels, 0u);
  EXPECT_EQ(message.candidate_count, 0u);
  EXPECT_EQ(message.candidate_pixels, 0u);
  EXPECT_FALSE(message.projection_uncertain);
  EXPECT_DOUBLE_EQ(message.capture_time, 0);
  for (double value : message.k) EXPECT_DOUBLE_EQ(value, 0);
  EXPECT_DOUBLE_EQ(message.parent_from_camera.position.x, 0);
  EXPECT_DOUBLE_EQ(message.parent_from_camera.position.y, 0);
  EXPECT_DOUBLE_EQ(message.parent_from_camera.position.z, 0);
  EXPECT_DOUBLE_EQ(message.parent_from_camera.orientation.w, 1);
}

inline GaussianRenderSnapshot ColoredPublication() {
  auto result = PublicationSnapshot();
  auto render = std::make_shared<GaussianRender>(*result.render);
  render->rgb[0] = 12;
  render->rgb[1] = 34;
  render->rgb[2] = 56;
  result.render = render;
  return result;
}

inline GaussianRenderSnapshot MalformedPublication(int scenario) {
  auto result = PublicationSnapshot();
  auto render = std::make_shared<GaussianRender>(*result.render);
  switch (scenario) {
    case 0:
      render->rgb.pop_back();
      break;
    case 1:
      render->K(0, 0) = -1;
      break;
    case 2:
      render->capture_time = -1;
      break;
    case 3:
      render->capture_time = 2147483648.0;
      break;
    case 4:
      render->capture_time = std::numeric_limits<double>::quiet_NaN();
      break;
    case 5:
      render->width = 1000001;
      break;
    case 6:
      render->camera_from_parent.translation.x() += 1;
      break;
    case 7:
      render->request.blocks[0].content.session_id.assign(32, 'c');
      break;
    case 8:
      ++render->request.generation;
      break;
    case 9: {
      const double c = std::sqrt(.5);
      render->camera_from_body.rotation << c, -c, 0, c, c, 0, 0, 0, 1;
      render->camera_from_body.translation = Eigen::Vector3d::Constant(1.7e308);
      render->camera_from_parent = pose_math::Compose(
          render->camera_from_body,
          pose_math::Inverse(render->request.parent_from_body));
      break;
    }
    case 10:
      render->request.blocks[0].content.source_generation = 5;
      render->request.blocks[0].asset->content =
          render->request.blocks[0].content;
      break;
    case 11:
      render->request.blocks[0].content.content_job_id =
          render->request.job_id + 1;
      render->request.blocks[0].asset->content =
          render->request.blocks[0].content;
      break;
    default:
      throw std::invalid_argument("Unknown malformed fixture");
  }
  result.render = render;
  return result;
}

}  // namespace livo_atlas::test
#endif  // LIVO_ATLAS_TESTS_GAUSSIAN_RENDER_PUBLISHER_TEST_HELPERS_H_
