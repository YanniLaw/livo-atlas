#include "core/online_submap_manager.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

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

class OnlineSubmapManagerTest : public testing::Test {
 protected:
  void SetUp() override {
    options_ = test::Options(FrontendMode::kLidarOnly);
    root_ = options_.exporter.directory / "keyframes";
  }

  void TearDown() override { fs::remove_all(options_.exporter.directory); }

  SystemOptions options_;
  fs::path root_;
};

TEST_F(OnlineSubmapManagerTest, LegacyTailFailurePreservesCommittedData) {
  for (const bool incremental : {false, true}) {
    SCOPED_TRACE(incremental);
    const auto directory = root_ / (incremental ? "delta" : "full");
    KeyframeStore frames(directory, options_.frontend.calibration);
    OnlineSubmapManager manager(directory / "online_submaps", frames, 2, {},
                                {});
    int notifications = 0;
    if (incremental) {
      manager.SetDeltaCallback([&](const auto&) { ++notifications; });
    } else {
      manager.SetCallback([&](const auto&) { ++notifications; });
    }
    auto first = std::make_shared<Keyframe>(ApproximateFrame(0));
    frames.Append(*first);
    manager.AppendCommittedKeyframe(first);
    PoseUpdate update;
    update.optimized_frame_count = 1;
    update.poses.push_back({0, {}});
    manager.ApplyPoseUpdate(update);
    const auto pose_file = directory / "online_submaps/poses/1.yml";
    const auto bytes = ReadBytes(pose_file);

    auto second = std::make_shared<Keyframe>(ApproximateFrame(1));
    frames.Append(*second);
    EXPECT_THROW(manager.AppendCommittedKeyframe(second),
                 std::invalid_argument);
    EXPECT_EQ(notifications, 2);
    EXPECT_THROW(manager.Finish(), std::invalid_argument);
    EXPECT_EQ(ReadBytes(pose_file), bytes);
    EXPECT_DOUBLE_EQ(frames.ReadRawBodyPose(1).rotation(0, 0), 1.000045);
    const OnlineSubmapStore reader(directory / "online_submaps", frames);
    EXPECT_EQ(reader.indexed_frame_count(), 2u);
    EXPECT_EQ(reader.ReadPoseSnapshot(1).frame_count, 1u);
    EXPECT_THROW(reader.MaterializePoseSnapshot(1, 2), std::invalid_argument);
  }
}

TEST_F(OnlineSubmapManagerTest, DeltasCarryPosesOnlyAtPoseCommits) {
  KeyframeStore frames(root_, options_.frontend.calibration);
  OnlineSubmapOptions options;
  options.translation_threshold = 2000;
  OnlineSubmapManager manager(root_ / "online_submaps", frames, 4, options, {});
  std::vector<OnlineSubmapDelta> deltas;
  std::vector<OnlineSubmapUpdate> full;
  manager.SetDeltaCallback(
      [&](const auto& update) { deltas.push_back(update); });
  manager.SetCallback([&](const auto& update) { full.push_back(update); });
  for (std::uint64_t id = 0; id < 2; ++id) {
    auto frame = std::make_shared<Keyframe>(ApproximateFrame(id));
    frames.Append(*frame);
    manager.AppendCommittedKeyframe(frame);
  }
  manager.SealInput();
  const auto previous = manager.GetPoseSnapshot();
  const PoseUpdate update{0, previous->frame_count, previous->poses,
                          PoseTailPolicy::kProjectedLastOptimizedAnchor};
  manager.ApplyPoseUpdate(update);
  manager.Finish();

  ASSERT_EQ(deltas.size(), 4u);
  ASSERT_EQ(full.size(), deltas.size());
  for (std::size_t i = 0; i < deltas.size(); ++i) {
    SCOPED_TRACE(i);
    const auto& delta = deltas[i];
    ASSERT_TRUE(full[i].poses);
    EXPECT_EQ(delta.frontier.version, full[i].poses->version);
    EXPECT_EQ(delta.frontier.frame_count, i == 0 ? 1u : 2u);
    EXPECT_EQ(delta.frontier.frame_count, full[i].poses->frame_count);
    EXPECT_EQ(delta.keyframe, full[i].keyframe);
    EXPECT_EQ(delta.revisions.size(), full[i].revisions.size());
    if (i < 3) {
      EXPECT_FALSE(delta.poses);
      EXPECT_TRUE(delta.pose_changes.empty());
    }
  }
  EXPECT_TRUE(deltas[0].keyframe);
  EXPECT_TRUE(deltas[1].keyframe);
  EXPECT_FALSE(deltas[2].keyframe);
  ASSERT_EQ(deltas[2].revisions.size(), 1u);
  EXPECT_EQ(deltas[2].revisions[0].state, OnlineSubmapState::kSealed);
  EXPECT_EQ(deltas.back().poses, full.back().poses);
  EXPECT_EQ(deltas.back().frontier.version, 1u);
  ASSERT_EQ(deltas.back().pose_changes.size(), 1u);
}

TEST_F(OnlineSubmapManagerTest, ProjectedTailContinuesAndReloads) {
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

TEST_F(OnlineSubmapManagerTest, PoseCommitReportsChangesAcrossSubmaps) {
  constexpr std::uint64_t kFrameCount = 64;
  for (const int submap_count : {2, 8, 32}) {
    SCOPED_TRACE(submap_count);
    const int frames_per_submap = kFrameCount / submap_count;
    const auto directory = root_ / std::to_string(submap_count);
    KeyframeStore frames(directory, options_.frontend.calibration);
    OnlineSubmapOptions options;
    options.translation_threshold = 100;
    options.time_threshold = 100;
    options.support_keyframes = 1;
    const auto submaps = directory / "online_submaps";
    OnlineSubmapManager manager(submaps, frames, frames_per_submap, options,
                                {});
    OnlineSubmapUpdate notification;
    int notification_count = 0;
    manager.SetCallback([&](const OnlineSubmapUpdate& update) {
      ++notification_count;
      if (!update.pose_changes.empty()) notification = update;
    });
    for (std::uint64_t id = 0; id < kFrameCount; ++id) {
      auto frame = std::make_shared<Keyframe>(ApproximateFrame(id));
      frame->odom_from_body.rotation.setIdentity();
      frame->odom_from_body.translation.x() = id;
      frames.Append(*frame);
      manager.AppendCommittedKeyframe(frame);
    }

    const auto before = manager.GetPoseSnapshot();
    PoseUpdate update{before->version, kFrameCount, before->poses,
                      PoseTailPolicy::kProjectedLastOptimizedAnchor};
    const Eigen::Vector3d offset(10, -3, 2);
    for (auto& pose : update.poses) pose.map_from_body.translation += offset;
    const std::uint64_t changed_id = frames_per_submap - 1;
    update.poses[changed_id].map_from_body.translation.y() += 2;

    auto invalid = update;
    invalid.poses.back().id = 0;
    const auto previous_notifications = notification_count;
    EXPECT_THROW(manager.ApplyPoseUpdate(invalid), std::invalid_argument);
    EXPECT_EQ(manager.GetPoseSnapshot()->version, 0u);
    EXPECT_FALSE(fs::exists(submaps / "poses/1.yml"));
    EXPECT_EQ(notification_count, previous_notifications);

    manager.ApplyPoseUpdate(update);
    EXPECT_EQ(notification_count, previous_notifications + 1);
    ASSERT_TRUE(notification.poses);
    EXPECT_EQ(notification.poses->version, 1u);
    EXPECT_EQ(notification.poses->frame_count, kFrameCount);
    ASSERT_EQ(notification.pose_changes.size(), submap_count);
    for (int id = 0; id < submap_count; ++id) {
      SCOPED_TRACE(id);
      const auto& change = notification.pose_changes[id];
      EXPECT_EQ(change.submap_id, id);
      EXPECT_EQ(change.old_version, 0u);
      EXPECT_EQ(change.new_version, 1u);
      const Eigen::Vector3d anchor(id * frames_per_submap, 0, 0);
      EXPECT_TRUE(change.old_parent_from_submap.translation.isApprox(anchor));
      EXPECT_TRUE(
          change.new_parent_from_submap.translation.isApprox(anchor + offset));
      EXPECT_TRUE(change.old_parent_from_submap.rotation.isIdentity());
      EXPECT_TRUE(change.new_parent_from_submap.rotation.isIdentity());
      if (id < 2) {
        EXPECT_EQ(change.changed_keyframe_ids,
                  (std::vector<std::uint64_t>{changed_id}));
      } else {
        EXPECT_TRUE(change.changed_keyframe_ids.empty());
      }
    }
    manager.Finish();
    const OnlineSubmapStore reader(submaps, frames);
    const auto stored = reader.ReadPoseSnapshot(1);
    ASSERT_EQ(stored.frame_count, kFrameCount);
    EXPECT_TRUE(stored.poses[changed_id].map_from_body.translation.isApprox(
        notification.poses->poses[changed_id].map_from_body.translation));
  }
}

}  // namespace
}  // namespace livo_atlas
