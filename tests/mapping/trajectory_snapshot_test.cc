#include "livo_atlas/mapping/trajectory_snapshot.h"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace livo_atlas {
namespace {

PoseSnapshot OptimizedHistory() {
  PoseSnapshot poses;
  poses.version = 2;
  poses.base_version = 1;
  poses.frame_count = 3;
  poses.optimized_frame_count = 2;
  poses.parent_frame = PoseFrame::kMap;
  poses.tail_policy = PoseTailPolicy::kProjectedLastOptimizedAnchor;
  poses.poses = {{0, {}}, {1, {}}, {2, {}}};
  poses.poses[0].map_from_body.translation = {1, 2, 3};
  poses.poses[1].map_from_body.translation = {8, 5, 6};
  poses.poses[2].map_from_body.translation = {9, 0, 1};
  poses.poses[1].map_from_body.rotation << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  return poses;
}

TEST(TrajectorySnapshotTest, UsesIndividualOptimizedPoses) {
  const auto result = BuildTrajectorySnapshot(OptimizedHistory(), {10, 10, 9});
  EXPECT_EQ(result.status, TrajectoryStatus::kReady);
  EXPECT_EQ(result.pose_version, 2);
  EXPECT_EQ(result.frame_count, 3);
  EXPECT_EQ(result.optimized_frame_count, 2);
  EXPECT_EQ(result.parent_frame, PoseFrame::kMap);
  ASSERT_EQ(result.keyframes.size(), 3);
  const Eigen::Vector3d expected[] = {{1, 2, 3}, {8, 5, 6}, {9, 0, 1}};
  const double times[] = {10, 10, 9};
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(result.keyframes[i].id, i);
    EXPECT_DOUBLE_EQ(result.keyframes[i].timestamp, times[i]);
    EXPECT_TRUE(result.keyframes[i].parent_from_body.translation.isApprox(
        expected[i], 1e-12));
  }
  // Raw x positions 0,1,2 cannot become these positions by a single rigid
  // correction: the first segment changes length from 1 to sqrt(67).
  EXPECT_NEAR((result.keyframes[1].parent_from_body.translation -
               result.keyframes[0].parent_from_body.translation)
                  .squaredNorm(),
              67, 1e-12);
  EXPECT_DOUBLE_EQ(result.keyframes[1].parent_from_body.rotation(0, 1), -1);
}

TEST(TrajectorySnapshotTest, ProjectsRawRotationWithoutMovingTranslation) {
  PoseSnapshot poses;
  poses.frame_count = 1;
  poses.poses = {{0, {}}};
  poses.poses[0].map_from_body.rotation(0, 0) = 1.00002;
  poses.poses[0].map_from_body.translation = {4, 5, 6};
  const auto result = BuildTrajectorySnapshot(poses, {0});
  ASSERT_EQ(result.keyframes.size(), 1);
  EXPECT_EQ(result.parent_frame, PoseFrame::kOdom);
  EXPECT_TRUE(result.keyframes[0].parent_from_body.rotation.isApprox(
      Eigen::Matrix3d::Identity(), 1e-12));
  EXPECT_TRUE(result.keyframes[0].parent_from_body.translation.isApprox(
      Eigen::Vector3d(4, 5, 6), 1e-12));
  EXPECT_DOUBLE_EQ(poses.poses[0].map_from_body.rotation(0, 0), 1.00002);
}

TEST(TrajectorySnapshotTest, EmptyHistoryIsReady) {
  const auto result = BuildTrajectorySnapshot({}, {});
  EXPECT_EQ(result.status, TrajectoryStatus::kReady);
  EXPECT_EQ(result.frame_count, 0);
  EXPECT_TRUE(result.keyframes.empty());
}

TEST(TrajectorySnapshotTest, RejectsMismatchedOrNonfiniteTimes) {
  const auto poses = OptimizedHistory();
  EXPECT_THROW(BuildTrajectorySnapshot(poses, {1, 2}), std::invalid_argument);
  EXPECT_THROW(BuildTrajectorySnapshot(poses, {1, 2, 3, 4}),
               std::invalid_argument);
  EXPECT_THROW(BuildTrajectorySnapshot(
                   poses, {1, std::numeric_limits<double>::infinity(), 3}),
               std::invalid_argument);
  EXPECT_THROW(BuildTrajectorySnapshot(
                   poses, {1, 2, std::numeric_limits<double>::quiet_NaN()}),
               std::invalid_argument);
}

TEST(TrajectorySnapshotTest, RejectsMalformedAndLegacyPositiveSnapshots) {
  auto poses = OptimizedHistory();
  poses.tail_policy = PoseTailPolicy::kLegacyLastOptimizedAnchor;
  EXPECT_THROW(BuildTrajectorySnapshot(poses, {1, 2, 3}),
               std::invalid_argument);
  poses = OptimizedHistory();
  poses.poses[2].id = 1;
  EXPECT_THROW(BuildTrajectorySnapshot(poses, {1, 2, 3}),
               std::invalid_argument);
  poses = OptimizedHistory();
  poses.poses[0].map_from_body.rotation(0, 0) = 1.00002;
  EXPECT_THROW(BuildTrajectorySnapshot(poses, {1, 2, 3}),
               std::invalid_argument);
  poses = OptimizedHistory();
  poses.parent_frame = PoseFrame::kOdom;
  EXPECT_THROW(BuildTrajectorySnapshot(poses, {1, 2, 3}),
               std::invalid_argument);
}

}  // namespace
}  // namespace livo_atlas
