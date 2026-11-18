#include <gtest/gtest.h>

#include <stdexcept>

#include "gaussian/render_test_helpers.h"
#include "livo_atlas/core/online_gaussian_options.h"

namespace livo_atlas {
namespace {

TEST(GaussianRenderPolicyTest, SpatialPolicyRequiresEveryRenderPrerequisite) {
  OnlineGaussianOptions options;
  options.joint_render_policy = GaussianRenderPolicy::kSpatial;
  EXPECT_THROW(ValidateOnlineGaussianOptions(options), std::invalid_argument);
  options.enabled = true;
  options.socket_path = "/tmp/spatial-options.sock";
  options.output_directory = "/tmp/spatial-options";
  EXPECT_THROW(ValidateOnlineGaussianOptions(options), std::invalid_argument);
  options.persist_submaps = true;
  EXPECT_THROW(ValidateOnlineGaussianOptions(options), std::invalid_argument);
  options.joint_render_enabled = true;
  EXPECT_NO_THROW(ValidateOnlineGaussianOptions(options));
  options = {};
  options.joint_render_policy = static_cast<GaussianRenderPolicy>(2);
  EXPECT_THROW(ValidateOnlineGaussianOptions(options), std::invalid_argument);
}

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

TEST(RenderProtocol, PublicRequestRejectsMissingOrContradictoryOmission) {
  auto request = test::RenderRequest();
  request.neighbor_reason = "neighbor_unavailable";
  EXPECT_THROW(ValidateGaussianRenderRequest(request), std::invalid_argument);
  request.neighbor_reason.clear();
  request.blocks.erase(request.blocks.begin());
  EXPECT_THROW(ValidateGaussianRenderRequest(request), std::invalid_argument);
}

TEST(RenderProtocol, PublicRequestRejectsZeroBindingGeneration) {
  auto request = test::RenderRequest();
  request.blocks.front().target.generation = 0;
  EXPECT_THROW(ValidateGaussianRenderRequest(request), std::invalid_argument);
}

TEST(RenderProtocol, RenderBoundaryUsesStrictRotationTolerance) {
  auto request = test::RenderRequest();
  // Valid under the legacy 1e-4 extrinsic tolerance, but not on render wire.
  request.blocks.front().target.submap_from_body.back().rotation(0, 0) += 2e-6;
  EXPECT_THROW(ValidateGaussianRenderRequest(request), std::invalid_argument);
}

TEST(RenderProtocol, AcceptedIdentityOwnsTheWholeSafeRequestHeader) {
  auto request = test::RenderRequest();
  const auto identity = GaussianRenderSourceIdentity(request);
  ASSERT_FALSE(identity.empty());
  ASSERT_EQ(identity.size() % 2, 0u);
  ASSERT_LE(identity.size(), 131072u);
  ASSERT_EQ(identity.find_first_not_of("0123456789abcdef"), std::string::npos);
  std::string text;
  for (std::size_t i = 0; i < identity.size(); i += 2)
    text.push_back(static_cast<char>(std::stoi(identity.substr(i, 2),
                                              nullptr, 16)));
  cv::FileStorage json(text, cv::FileStorage::READ | cv::FileStorage::MEMORY);
  EXPECT_EQ(static_cast<std::string>(json["type"]), "joint_render");
  EXPECT_EQ(static_cast<std::string>(json["job_id"]), "9007199254740993");
  EXPECT_EQ(static_cast<std::string>(
                json["blocks"][0]["asset"]["manifest_sha256"]),
            std::string(64, 'b'));
  request.blocks.front().asset->manifest_sha256 = std::string(64, 'c');
  EXPECT_NE(GaussianRenderSourceIdentity(request), identity);
}

TEST(RenderProtocol, SpatialWirePreservesNonadjacentSourcesAndSummary) {
  const auto request = test::SpatialRenderRequest();
  const auto message = EncodeGaussianRenderRequest(request);
  const auto decoded = DecodeGaussianRenderRequest(message);
  EXPECT_EQ(decoded.policy, GaussianRenderPolicy::kSpatial);
  EXPECT_EQ(decoded.blocks.front().content.submap_id, 0u);
  EXPECT_EQ(decoded.blocks.back().content.submap_id, 2u);
  EXPECT_EQ(decoded.blocks.front().render_metadata,
            request.blocks.front().render_metadata);
  EXPECT_EQ(decoded.candidate_count, 1u);
  EXPECT_EQ(decoded.candidate_pixels, 48u);
  EXPECT_EQ(message.body.size(), 640u);
  EXPECT_LT(message.header_json.size(), 65536u);
}

TEST(RenderProtocol, SpatialReplyCannotReplaceTheRetainedQueryCamera) {
  const auto request = test::SpatialRenderRequest();
  auto reply = test::RenderReply(EncodeGaussianRenderRequest(request));
  EXPECT_NO_THROW(DecodeGaussianRenderReply(reply, request));
  cv::FileStorage input(reply.header_json,
                        cv::FileStorage::READ | cv::FileStorage::MEMORY);
  reply.header_json = gaussian_mapper_io::Json([&](auto& out) {
    for (const auto& child : input.root()) {
      out << child.name();
      if (child.name() == "fx") out << 5.;
      else test::CopyJson(out, child);
    }
  });
  EXPECT_THROW(DecodeGaussianRenderReply(reply, request), std::exception);
}

}  // namespace
}  // namespace livo_atlas
