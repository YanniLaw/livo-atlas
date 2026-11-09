#include "livo_atlas/mapping/submap_pose_change.h"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <stdexcept>

#include "mapping/keyframe_fixtures.h"

namespace livo_atlas {
namespace {
TEST(SubmapPoseChangeTest, RigidAndLocalChangesSeparate) {
  RigidTransform a, b;
  a.rotation = Eigen::AngleAxisd(std::acos(-1.0) / 2, Eigen::Vector3d::UnitZ())
                   .toRotationMatrix();
  a.translation = {1, 2, 3};
  b = a;
  b.translation = {1, 4, 3};
  const auto old = MaterializePoseSnapshot({}, {a, b}, 2);
  OnlineSubmap group;
  group.primary = {0, 0, {0, 1}};
  const auto rotation =
      Eigen::AngleAxisd(std::acos(-1.0) / 2, Eigen::Vector3d::UnitX())
          .toRotationMatrix();
  PoseUpdate update;
  update.optimized_frame_count = 2;
  for (const auto& pose : old.poses)
    update.poses.push_back({pose.id,
                            {rotation * pose.map_from_body.rotation,
                             rotation * pose.map_from_body.translation +
                                 Eigen::Vector3d(10, -3, 2)}});
  const auto rigid = MakePoseSnapshot(update, {}, {a, b});
  EXPECT_TRUE(
      CompareSubmapPoses(group, old, rigid).changed_keyframe_ids.empty());
  update.poses[0].map_from_body = {};
  update.poses[0].map_from_body.translation = {10, 0, 0};
  update.poses[1].map_from_body = b;
  update.poses[1].map_from_body.translation = {10, 5, 0};
  const auto local = MakePoseSnapshot(update, {}, {a, b});
  EXPECT_EQ(CompareSubmapPoses(group, old, local).changed_keyframe_ids,
            (std::vector<std::uint64_t>{1}));
  group.primary = {1, 1, {1}};
  group.support_keyframe_ids = {0};
  EXPECT_EQ(CompareSubmapPoses(group, old, local).changed_keyframe_ids,
            (std::vector<std::uint64_t>{0}));
}
TEST(SubmapPoseChangeTest, SmallRotationAndMissingCoverageAreDetected) {
  const auto old =
      MaterializePoseSnapshot({}, std::vector<RigidTransform>(2), 2);
  OnlineSubmap group;
  group.primary = {0, 0, {0, 1}};
  auto next = old;
  next.poses[1].map_from_body.rotation =
      Eigen::AngleAxisd(2e-9, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_EQ(CompareSubmapPoses(group, old, next).changed_keyframe_ids.size(),
            1u);
  next.poses[1].map_from_body.rotation =
      Eigen::AngleAxisd(.5e-9, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_TRUE(
      CompareSubmapPoses(group, old, next).changed_keyframe_ids.empty());
  group.primary.keyframe_ids.push_back(2);
  EXPECT_THROW(CompareSubmapPoses(group, old, next), std::invalid_argument);
}
TEST(SubmapPoseChangeTest, SkippedMembershipInitializesOnlyNewObservations) {
  const auto old =
      MaterializePoseSnapshot({}, std::vector<RigidTransform>(3), 3);
  OnlineSubmap existing;
  existing.primary = {0, 0, {0, 1}};
  auto latest = old;
  latest.version = 2;
  latest.base_version = 1;
  latest.optimized_frame_count = 3;
  latest.parent_frame = PoseFrame::kMap;
  latest.poses[2].map_from_body.translation.x() = 5;
  EXPECT_TRUE(
      CompareSubmapPoses(existing, old, latest).changed_keyframe_ids.empty());
  existing.primary.keyframe_ids.push_back(2);
  EXPECT_EQ(CompareSubmapPoses(existing, old, latest).changed_keyframe_ids,
            (std::vector<std::uint64_t>{2}));
}
TEST(SubmapPoseChangeTest, CameraRelativePoseSurvivesCorrection) {
  const auto frame = CaptureKeyframe(0, test_data::KeyframeResult());
  auto corrected = frame.odom_from_body;
  corrected.translation += Eigen::Vector3d(10, 20, -3);
  const auto poses =
      MakePoseSnapshot({0, 1, {{0, corrected}}}, {}, {frame.odom_from_body});
  const auto camera = CameraPose(frame, poses);
  const auto original =
      CameraPose(frame, MaterializePoseSnapshot({}, {frame.odom_from_body}, 1));
  EXPECT_TRUE(original.rotation.isApprox(Eigen::Matrix3d::Identity()));
  EXPECT_TRUE(original.translation.isApprox(Eigen::Vector3d(-7, -8, -9)));
  EXPECT_TRUE(camera.rotation.isApprox(original.rotation));
  EXPECT_TRUE((camera.translation - original.translation)
                  .isApprox(Eigen::Vector3d(10, 20, -3)));
  corrected.rotation.setIdentity();
  const auto rotated =
      MakePoseSnapshot({0, 1, {{0, corrected}}}, {}, {frame.odom_from_body});
  const auto rotated_camera = CameraPose(frame, rotated);
  EXPECT_TRUE(rotated_camera.translation.isApprox(Eigen::Vector3d(1, 30, -12)));
  EXPECT_TRUE(rotated_camera.rotation.isApprox(
      frame.odom_from_body.rotation.transpose()));
  EXPECT_NE(frame.timestamp, *frame.image_timestamp);
  auto missing = frame;
  missing.image_timestamp.reset();
  EXPECT_THROW(CameraPose(missing, poses), std::invalid_argument);
}
}  // namespace
}  // namespace livo_atlas
