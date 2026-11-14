#include "core/online_submap_manager.h"

#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "core/fixtures.h"
#include "mapping/keyframe_fixtures.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;

std::string ReadBytes(const fs::path& path) {
  std::ifstream stream(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(stream), {}};
}

Keyframe ApproximateFrame(std::uint64_t id) {
  auto frame =
      CaptureKeyframe(id, test_data::KeyframeResult(UpdateKind::kLidarOnly));
  frame.timestamp += id;
  frame.cloud_timestamp += id;
  frame.odom_from_body = {};
  frame.odom_from_body.rotation(0, 0) = 1.000045;
  frame.odom_from_body.translation.x() = 1000 * id;
  return frame;
}

class OnlineSubmapManagerTailTest : public testing::Test {
 protected:
  void SetUp() override {
    options_ = test::Options(FrontendMode::kLidarOnly);
    root_ = options_.exporter.directory / "keyframes";
  }

  void TearDown() override { fs::remove_all(options_.exporter.directory); }

  SystemOptions options_;
  fs::path root_;
};

TEST_F(OnlineSubmapManagerTailTest, LegacyTailFailurePreservesCommittedData) {
  KeyframeStore frames(root_, options_.frontend.calibration);
  OnlineSubmapManager manager(root_ / "online_submaps", frames, 2, {}, {});
  manager.SetCallback([](const auto&) {});
  auto first = std::make_shared<Keyframe>(ApproximateFrame(0));
  frames.Append(*first);
  manager.AppendCommittedKeyframe(first);
  PoseUpdate update;
  update.optimized_frame_count = 1;
  update.poses.push_back({0, {}});
  manager.ApplyPoseUpdate(update);
  const auto pose_file = root_ / "online_submaps/poses/1.yml";
  const auto bytes = ReadBytes(pose_file);

  auto second = std::make_shared<Keyframe>(ApproximateFrame(1));
  frames.Append(*second);
  EXPECT_THROW(manager.AppendCommittedKeyframe(second), std::invalid_argument);
  EXPECT_THROW(manager.Finish(), std::invalid_argument);
  EXPECT_EQ(ReadBytes(pose_file), bytes);
  EXPECT_DOUBLE_EQ(frames.ReadRawBodyPose(1).rotation(0, 0), 1.000045);
  const OnlineSubmapStore reader(root_ / "online_submaps", frames);
  EXPECT_EQ(reader.indexed_frame_count(), 2u);
  EXPECT_EQ(reader.ReadPoseSnapshot(1).frame_count, 1u);
  EXPECT_THROW(reader.MaterializePoseSnapshot(1, 2), std::invalid_argument);
}

TEST_F(OnlineSubmapManagerTailTest, ProjectedTailContinuesAndReloads) {
  KeyframeStore frames(root_, options_.frontend.calibration);
  OnlineSubmapManager manager(root_ / "online_submaps", frames, 2, {}, {});
  int delivered = 0;
  manager.SetCallback([&](const auto& update) {
    if (update.keyframe) ++delivered;
  });
  auto first = std::make_shared<Keyframe>(ApproximateFrame(0));
  frames.Append(*first);
  manager.AppendCommittedKeyframe(first);
  PoseUpdate update;
  update.optimized_frame_count = 1;
  update.poses.push_back({0, {}});
  update.tail_policy = PoseTailPolicy::kProjectedLastOptimizedAnchor;
  manager.ApplyPoseUpdate(update);

  auto second = std::make_shared<Keyframe>(ApproximateFrame(1));
  frames.Append(*second);
  manager.AppendCommittedKeyframe(second);
  const auto live = manager.GetPoseSnapshot();
  ASSERT_EQ(live->frame_count, 2u);
  EXPECT_EQ(live->tail_policy, PoseTailPolicy::kProjectedLastOptimizedAnchor);
  EXPECT_NEAR(live->poses[1].map_from_body.translation.x(), 1000, 1e-10);
  EXPECT_EQ(delivered, 2);
  const auto correction = manager.GetMapCorrection();
  EXPECT_EQ(correction.pose_version, 1u);
  EXPECT_EQ(correction.frame_count, 2u);
  EXPECT_EQ(correction.optimized_frame_count, 1u);
  EXPECT_TRUE(correction.map_from_odom.rotation.isIdentity(1e-12));
  EXPECT_TRUE(correction.map_from_odom.translation.isZero(1e-12));
  manager.Finish();

  const OnlineSubmapStore reader(root_ / "online_submaps", frames);
  const auto reloaded = reader.MaterializePoseSnapshot(1, 2);
  EXPECT_EQ(reloaded.tail_policy, live->tail_policy);
  EXPECT_NEAR(reloaded.poses[1].map_from_body.translation.x(), 1000, 1e-10);
  EXPECT_DOUBLE_EQ(frames.ReadRawBodyPose(1).rotation(0, 0), 1.000045);
}

}  // namespace
}  // namespace livo_atlas
