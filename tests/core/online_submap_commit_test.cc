#include "livo_atlas/core/livo_system.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "core/loop_backend_fixtures.h"
#include "livo_atlas/io/keyframe_store.h"
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

class OnlineSubmapCommitTest : public testing::Test {
 protected:
  void SetUp() override {
    options_ = loop_test::OnlineOptions();
    root_ = options_.exporter.directory / "keyframes";
  }

  void TearDown() override { fs::remove_all(options_.exporter.directory); }

  SystemOptions options_;
  fs::path root_;
};

TEST_F(OnlineSubmapCommitTest, LegacyTailFailurePreservesCommittedData) {
  for (const bool incremental : {false, true}) {
    SCOPED_TRACE(incremental);
    auto config = options_;
    config.exporter.directory /= incremental ? "delta" : "full";
    config.submaps.max_keyframes = 2;
    const auto directory = config.exporter.directory / "keyframes";
    LivoSystem system(config);
    int notifications = 0;
    if (incremental) {
      system.SetOnlineSubmapDeltaCallback(
          [&](const auto&) { ++notifications; });
    } else {
      system.SetOnlineSubmapCallback([&](const auto&) { ++notifications; });
    }
    LoopBackendTestPeer::Append(system, ApproximateFrame(0));
    PoseUpdate update;
    update.optimized_frame_count = 1;
    update.poses.push_back({0, {}});
    system.ApplyPoseUpdate(update);
    const auto pose_file = directory / "online_submaps/poses/1.yml";
    const auto bytes = ReadBytes(pose_file);

    EXPECT_THROW(LoopBackendTestPeer::Append(system, ApproximateFrame(1)),
                 std::invalid_argument);
    EXPECT_EQ(system.GetKeyframeReadView().frame_count(), 1u);
    EXPECT_EQ(notifications, 2);
    EXPECT_THROW(system.Finish(), std::invalid_argument);
    EXPECT_EQ(ReadBytes(pose_file), bytes);
    const KeyframeStore frames(directory);
    EXPECT_DOUBLE_EQ(frames.ReadRawBodyPose(1).rotation(0, 0), 1.000045);
    const OnlineSubmapStore reader(directory / "online_submaps", frames);
    EXPECT_EQ(reader.indexed_frame_count(), 2u);
    EXPECT_EQ(reader.ReadPoseSnapshot(1).frame_count, 1u);
    EXPECT_THROW(reader.MaterializePoseSnapshot(1, 2), std::invalid_argument);
  }
}

TEST_F(OnlineSubmapCommitTest, DeltasCarryPosesOnlyAtPoseCommits) {
  options_.online_submaps.translation_threshold = 2000;
  options_.submaps.max_keyframes = 4;
  LivoSystem system(options_);
  std::vector<OnlineSubmapDelta> deltas;
  std::vector<OnlineSubmapUpdate> full;
  system.SetOnlineSubmapDeltaCallback(
      [&](const auto& update) { deltas.push_back(update); });
  system.SetOnlineSubmapCallback(
      [&](const auto& update) { full.push_back(update); });
  for (std::uint64_t id = 0; id < 2; ++id) {
    LoopBackendTestPeer::Append(system, ApproximateFrame(id));
  }
  system.SealInput();
  const auto previous = system.GetPoseSnapshot();
  const PoseUpdate update{0, previous->frame_count, previous->poses,
                          PoseTailPolicy::kProjectedLastOptimizedAnchor};
  system.ApplyPoseUpdate(update);
  system.Finish();

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
    if (delta.keyframe) {
      ASSERT_TRUE(delta.appended_pose);
      const auto& pose = full[i].poses->poses.back().map_from_body;
      EXPECT_TRUE(delta.appended_pose->rotation.isApprox(pose.rotation));
      EXPECT_TRUE(
          delta.appended_pose->translation.isApprox(pose.translation));
    } else {
      EXPECT_FALSE(delta.appended_pose);
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

TEST_F(OnlineSubmapCommitTest, ProjectedTailContinuesAndReloads) {
  options_.submaps.max_keyframes = 2;
  LivoSystem system(options_);
  int delivered = 0;
  system.SetOnlineSubmapCallback([&](const auto& update) {
    if (update.keyframe) ++delivered;
  });
  LoopBackendTestPeer::Append(system, ApproximateFrame(0));
  PoseUpdate update;
  update.optimized_frame_count = 1;
  update.poses.push_back({0, {}});
  update.tail_policy = PoseTailPolicy::kProjectedLastOptimizedAnchor;
  system.ApplyPoseUpdate(update);

  LoopBackendTestPeer::Append(system, ApproximateFrame(1));
  const auto live = system.GetPoseSnapshot();
  ASSERT_EQ(live->frame_count, 2u);
  EXPECT_EQ(live->tail_policy, PoseTailPolicy::kProjectedLastOptimizedAnchor);
  EXPECT_NEAR(live->poses[1].map_from_body.translation.x(), 1000, 1e-10);
  EXPECT_EQ(delivered, 2);
  const auto correction = system.GetMapCorrection();
  EXPECT_EQ(correction.pose_version, 1u);
  EXPECT_EQ(correction.frame_count, 2u);
  EXPECT_EQ(correction.optimized_frame_count, 1u);
  EXPECT_TRUE(correction.map_from_odom.rotation.isIdentity(1e-12));
  EXPECT_TRUE(correction.map_from_odom.translation.isZero(1e-12));
  system.Finish();

  const KeyframeStore frames(root_);
  const OnlineSubmapStore reader(root_ / "online_submaps", frames);
  const auto reloaded = reader.MaterializePoseSnapshot(1, 2);
  EXPECT_EQ(reloaded.tail_policy, live->tail_policy);
  EXPECT_NEAR(reloaded.poses[1].map_from_body.translation.x(), 1000, 1e-10);
  EXPECT_DOUBLE_EQ(frames.ReadRawBodyPose(1).rotation(0, 0), 1.000045);
}

TEST_F(OnlineSubmapCommitTest, PoseCommitReportsChangesAcrossSubmaps) {
  constexpr std::uint64_t kFrameCount = 64;
  for (const int submap_count : {2, 8, 32}) {
    SCOPED_TRACE(submap_count);
    const int frames_per_submap = kFrameCount / submap_count;
    auto options = options_;
    options.exporter.directory /= std::to_string(submap_count);
    options.submaps.max_keyframes = frames_per_submap;
    options.online_submaps.translation_threshold = 100;
    options.online_submaps.time_threshold = 100;
    options.online_submaps.support_keyframes = 1;
    const auto directory = options.exporter.directory / "keyframes";
    const auto submaps = directory / "online_submaps";
    LivoSystem system(options);
    OnlineSubmapUpdate notification;
    int notification_count = 0;
    system.SetOnlineSubmapCallback([&](const OnlineSubmapUpdate& update) {
      ++notification_count;
      if (!update.pose_changes.empty()) notification = update;
    });
    for (std::uint64_t id = 0; id < kFrameCount; ++id) {
      auto frame = ApproximateFrame(id);
      frame.odom_from_body.rotation.setIdentity();
      frame.odom_from_body.translation.x() = id;
      LoopBackendTestPeer::Append(system, std::move(frame));
    }

    const auto before = system.GetPoseSnapshot();
    PoseUpdate update{before->version, kFrameCount, before->poses,
                      PoseTailPolicy::kProjectedLastOptimizedAnchor};
    const Eigen::Vector3d offset(10, -3, 2);
    for (auto& pose : update.poses) pose.map_from_body.translation += offset;
    const std::uint64_t changed_id = frames_per_submap - 1;
    update.poses[changed_id].map_from_body.translation.y() += 2;

    auto invalid = update;
    invalid.poses.back().id = 0;
    const auto previous_notifications = notification_count;
    EXPECT_THROW(system.ApplyPoseUpdate(invalid), std::invalid_argument);
    EXPECT_EQ(system.GetPoseSnapshot()->version, 0u);
    EXPECT_FALSE(fs::exists(submaps / "poses/1.yml"));
    EXPECT_EQ(notification_count, previous_notifications);

    system.ApplyPoseUpdate(update);
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
    system.Finish();
    const KeyframeStore frames(directory);
    const OnlineSubmapStore reader(submaps, frames);
    const auto stored = reader.ReadPoseSnapshot(1);
    ASSERT_EQ(stored.frame_count, kFrameCount);
    EXPECT_TRUE(stored.poses[changed_id].map_from_body.translation.isApprox(
        notification.poses->poses[changed_id].map_from_body.translation));
  }
}

}  // namespace
}  // namespace livo_atlas
