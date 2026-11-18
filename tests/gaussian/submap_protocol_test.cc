#include "gaussian/submap_protocol.h"

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "gaussian/render_metadata_protocol.h"
#include "gaussian/render_metadata_test_helpers.h"
#include "gaussian/render_test_helpers.h"
#include "gaussian/wire_protocol.h"

namespace livo_atlas {
namespace {

TEST(GaussianSubmapProtocolTest, SpatialMetadataOwnsCanonicalValues) {
  const auto value = test::RenderMetadata({std::string(32, 'a'), 0, 1, 2},
                                         {0, 2});
  const auto text = gaussian_mapper_io::Json([&](auto& out) {
    out << "render_metadata";
    WriteGaussianRenderMetadata(out, value);
  });
  cv::FileStorage json(text, cv::FileStorage::READ | cv::FileStorage::MEMORY);
  const auto decoded = DecodeGaussianRenderMetadata(
      json["render_metadata"], value.content, {0, 2});
  EXPECT_EQ(decoded.content.content_job_id, 2u);
  EXPECT_EQ(decoded.camera.keyframe_id, 2u);
  EXPECT_EQ(decoded.camera.K(0, 2), 3.5);
  EXPECT_EQ(decoded.extents[1].origin_keyframe_id, 2u);
  EXPECT_EQ(decoded.extents[0].min.z(), 4);
  auto wrong_content = value.content;
  ++wrong_content.content_job_id;
  EXPECT_THROW(DecodeGaussianRenderMetadata(json["render_metadata"],
                   wrong_content, {0, 2}), std::invalid_argument);
  EXPECT_THROW(DecodeGaussianRenderMetadata(json["render_metadata"],
                   value.content, {0, 1}), std::invalid_argument);
  const auto extra = gaussian_mapper_io::Json([&](auto& out) {
    for (const auto& child : json["render_metadata"]) {
      out << child.name();
      test::CopyJson(out, child);
    }
    out << "unknown" << 1;
  });
  cv::FileStorage invalid(extra,
                         cv::FileStorage::READ | cv::FileStorage::MEMORY);
  EXPECT_THROW(DecodeGaussianRenderMetadata(invalid.root(), value.content,
                                           {0, 2}), std::invalid_argument);
}

GaussianTarget Target(int count = 512) {
  GaussianTarget target;
  target.session_id = std::string(32, 'a');
  target.generation = 1;
  target.frame_count = count;
  target.group.membership_revision = count;
  target.parent_from_submap.rotation << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  target.parent_from_submap.translation = {4, -2, 1};
  for (int i = 0; i < count; ++i) {
    target.group.primary.keyframe_ids.push_back(i);
    RigidTransform local;
    local.translation.x() = i;
    target.submap_from_body.push_back(local);
  }
  return target;
}

GaussianMessage Activation() {
  const auto target = Target();
  cv::FileStorage json("", cv::FileStorage::WRITE | cv::FileStorage::MEMORY |
                               cv::FileStorage::FORMAT_JSON);
  json << "schema" << 2 << "type"
       << "activate"
       << "session_id" << target.session_id << "job_id"
       << "9007199254740993"
       << "action"
       << "empty";
  WriteGaussianTargetHeader(json, target);
  return {json.releaseAndGetString(), EncodeGaussianPrimaryPoses(target)};
}

TEST(GaussianSubmapProtocolTest,
     EncodesFullPrimaryAsBoundedLittleEndianFloat64) {
  const auto message = Activation();
  ASSERT_EQ(message.body.size(), 512u * 128);
  ASSERT_LT(message.header_json.size(), 64u * 1024);
  const std::vector<std::uint8_t> one = {0, 0, 0, 0, 0, 0, 0xf0, 0x3f};
  EXPECT_EQ(
      std::vector<std::uint8_t>(message.body.begin(), message.body.begin() + 8),
      one);
  const auto offset = 511 * 128 + 24;
  const std::vector<std::uint8_t> last = {0, 0, 0, 0, 0, 0xf0, 0x7f, 0x40};
  EXPECT_EQ(std::vector<std::uint8_t>(message.body.begin() + offset,
                                      message.body.begin() + offset + 8),
            last);
  EXPECT_NO_THROW(EncodeGaussianMessage(message));
}

TEST(GaussianSubmapProtocolTest, RejectsIncorrectBodySizeAnchorAndUnknownKeys) {
  auto message = Activation();
  message.body.pop_back();
  EXPECT_THROW(EncodeGaussianMessage(message), std::invalid_argument);
  auto target = Target(2);
  target.submap_from_body.front().translation.x() = 1;
  EXPECT_THROW(EncodeGaussianPrimaryPoses(target), std::invalid_argument);
  message = Activation();
  const auto position = message.header_json.find('{');
  message.header_json.insert(position + 1, "\"unknown\":1,");
  EXPECT_THROW(EncodeGaussianMessage(message), std::invalid_argument);
}

TEST(GaussianSubmapProtocolTest,
     ReferenceAcknowledgementAcceptsLargeIdsButNotDuplicates) {
  GaussianMessage message;
  message.header_json =
      R"({"schema":2,"type":"map_committed","session_id":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",)"
      R"("job_id":"9007199254740993","generation":"1","pose_version":"0",)"
      R"("map_sequence":"1","asset_ids":["1","18446744073709551615"]})";
  EXPECT_NO_THROW(EncodeGaussianMessage(message));
  const auto position = message.header_json.find("18446744073709551615");
  message.header_json.replace(position, 20, "1");
  EXPECT_THROW(EncodeGaussianMessage(message), std::invalid_argument);
}

}  // namespace
}  // namespace livo_atlas
