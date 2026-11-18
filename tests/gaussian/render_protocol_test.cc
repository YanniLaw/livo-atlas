#include <gtest/gtest.h>

#include <stdexcept>

#include "gaussian/render_test_helpers.h"

namespace livo_atlas {
namespace {

TEST(RenderProtocol, EarlierRetainedOwnerCameraIsRejected) {
  auto request = test::RenderRequest();
  request.query_keyframe_id = 4;
  request.parent_from_body.translation.x() = 4;
  EXPECT_THROW(EncodeGaussianRenderRequest(request), std::invalid_argument);
}

TEST(RenderProtocol, OwnedRequestPreservesLargeIdsAndAllCanonicalBodies) {
  const auto expected = test::RenderRequest();
  const auto message = EncodeGaussianRenderRequest(expected);
  const auto decoded = DecodeGaussianRenderRequest(message);
  EXPECT_EQ(decoded.job_id, 9007199254740993ULL);
  EXPECT_EQ(message.body.size(), 640u);
  EXPECT_EQ(decoded.blocks.size(), 2u);
  EXPECT_EQ(decoded.blocks[1].target.submap_from_body[1].translation.x(), 1);
  EXPECT_EQ(decoded.parent_from_body.translation.x(), 5);
}

TEST(RenderProtocol, ResidentBindingCanPredateDiscardedActivationFence) {
  auto request = test::RenderRequest();
  request.blocks.erase(request.blocks.begin());
  request.blocks[0].resident = true;
  request.blocks[0].asset.reset();
  request.neighbor_reason = "neighbor_unavailable";
  const auto decoded =
      DecodeGaussianRenderRequest(EncodeGaussianRenderRequest(request));
  EXPECT_EQ(decoded.generation, 4u);
  EXPECT_EQ(decoded.blocks[0].target.generation, 3u);
}

TEST(RenderProtocol, ReplyHasOwnedPixelsMetricsAndExactEcho) {
  const auto request = test::RenderRequest();
  auto message = test::RenderReply(EncodeGaussianRenderRequest(request));
  const auto result = DecodeGaussianRenderReply(message, request);
  EXPECT_EQ(result.rgb.size(), 144u);
  EXPECT_EQ(result.joint.image_pixels, 48);
  ASSERT_TRUE(result.joint.rgb_psnr_db);
  EXPECT_NEAR(*result.joint.rgb_psnr_db, 13.979400086720375, 1e-9);
  auto changed = request;
  ++changed.map_sequence;
  EXPECT_THROW(DecodeGaussianRenderReply(message, changed), std::exception);
  changed = request;
  ++changed.blocks[0].asset->asset_id;
  EXPECT_THROW(DecodeGaussianRenderReply(message, changed), std::exception);
  message.body.pop_back();
  EXPECT_THROW(DecodeGaussianRenderReply(message, request), std::exception);
}

TEST(RenderProtocol, BoundsAndCameraRelationRejectCorruptPayload) {
  auto request = test::RenderRequest();
  auto message = EncodeGaussianRenderRequest(request);
  message.body.pop_back();
  EXPECT_THROW(DecodeGaussianRenderRequest(message), std::exception);
  message = EncodeGaussianRenderRequest(request);
  // Query tx no longer equals its primary body pose.
  std::fill(message.body.begin() + 24, message.body.begin() + 32, 0);
  EXPECT_THROW(DecodeGaussianRenderRequest(message), std::exception);
  request.frame_count = 513;
  EXPECT_THROW(EncodeGaussianRenderRequest(request), std::exception);
}

}  // namespace
}  // namespace livo_atlas
