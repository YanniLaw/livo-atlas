#include "livo_atlas/mapping/pose_snapshot.h"

#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

namespace livo_atlas {
namespace {
std::vector<RigidTransform> Raw(int count) {
  std::vector<RigidTransform> poses(count);
  for (int i = 0; i < count; ++i) poses[i].translation.x() = i;
  return poses;
}

PoseUpdate Update(std::uint64_t base, int count) {
  PoseUpdate update;
  update.base_version = base;
  update.optimized_frame_count = count;
  for (int i = 0; i < count; ++i) {
    RigidTransform pose;
    pose.translation.x() = 10 + i;
    update.poses.push_back({static_cast<std::uint64_t>(i), pose});
  }
  return update;
}

TEST(PoseSnapshotTest, TailExtensionKeepsFrozenPrefix) {
  const auto frozen = MakePoseSnapshot(Update(0, 2), {}, Raw(3));
  EXPECT_EQ(frozen.frame_count, 3u);
  EXPECT_EQ(frozen.optimized_frame_count, 2u);
  EXPECT_DOUBLE_EQ(frozen.poses[2].map_from_body.translation.x(), 12);
  const auto view = MaterializePoseSnapshot(frozen, Raw(4), 4);
  EXPECT_EQ(view.version, 1u);
  EXPECT_DOUBLE_EQ(view.poses[3].map_from_body.translation.x(), 13);
  EXPECT_EQ(frozen.poses.size(), 3u);
  EXPECT_THROW(MaterializePoseSnapshot(frozen, Raw(4), 2),
               std::invalid_argument);
  EXPECT_THROW(MaterializePoseSnapshot(frozen, Raw(4), 5),
               std::invalid_argument);
}

TEST(PoseSnapshotTest, NewFramesDoNotStaleBaseVersion) {
  const auto frozen = MakePoseSnapshot(Update(0, 2), {}, Raw(3));
  const auto next = MakePoseSnapshot(Update(1, 3), frozen, Raw(5));
  EXPECT_EQ(next.version, 2u);
  EXPECT_EQ(next.frame_count, 5u);
  for (auto invalid : {Update(0, 3), Update(1, 1), Update(1, 0), Update(1, 6)})
    EXPECT_THROW(MakePoseSnapshot(invalid, frozen, Raw(5)),
                 std::invalid_argument);
}

TEST(PoseSnapshotTest, VersionZeroHasNoTailAnchor) {
  ValidatePoseSnapshot({});
  const auto raw = MaterializePoseSnapshot({}, Raw(3), 3);
  EXPECT_EQ(raw.parent_frame, PoseFrame::kOdom);
  EXPECT_EQ(raw.optimized_frame_count, 0u);
  EXPECT_DOUBLE_EQ(raw.poses[2].map_from_body.translation.x(), 2);
  EXPECT_TRUE(MaterializePoseSnapshot(raw, Raw(3), 0).poses.empty());
}

TEST(PoseSnapshotTest, InvalidSnapshotRejected) {
  const auto valid = MakePoseSnapshot(Update(0, 2), {}, Raw(3));
  for (int fault = 0; fault < 9; ++fault) {
    auto invalid = valid;
    if (fault == 0) invalid.poses[1].id = 0;
    if (fault == 1) invalid.poses.pop_back();
    if (fault == 2) std::swap(invalid.poses[0], invalid.poses[1]);
    if (fault == 3) invalid.poses[0].id = std::uint64_t{1} << 40;
    if (fault == 4)
      invalid.frame_count = std::numeric_limits<std::uint64_t>::max();
    if (fault == 5) invalid.parent_frame = PoseFrame::kOdom;
    if (fault == 6) invalid.poses[0].map_from_body.rotation.setZero();
    if (fault == 7)
      invalid.poses[0].map_from_body.translation.x() =
          std::numeric_limits<double>::quiet_NaN();
    if (fault == 8) invalid.base_version = 1;
    EXPECT_THROW(ValidatePoseSnapshot(invalid), std::invalid_argument);
  }
  auto tail = valid;
  tail.poses.back().map_from_body.translation.x() += 1;
  EXPECT_THROW(MaterializePoseSnapshot(tail, Raw(4), 4), std::invalid_argument);
  auto overflow = valid;
  overflow.version = std::numeric_limits<std::uint64_t>::max();
  overflow.base_version = overflow.version - 1;
  auto update = Update(overflow.version, 2);
  EXPECT_THROW(MakePoseSnapshot(update, overflow, Raw(3)), std::overflow_error);
  auto bad_raw = Raw(3);
  bad_raw.back().rotation.setZero();
  EXPECT_THROW(MaterializePoseSnapshot(valid, bad_raw, 3),
               std::invalid_argument);
}

TEST(PoseSnapshotTest, ProjectedTailHandlesValidApproximateRotations) {
  auto raw = Raw(2);
  raw[0].rotation(0, 0) = 1.000045;
  raw[1].rotation(0, 0) = 1.000045;
  raw[1].translation.x() = 1000;
  ASSERT_TRUE(ValidateExtrinsic(raw[0]).empty());
  ASSERT_TRUE(ValidateExtrinsic(raw[1]).empty());

  PoseUpdate update;
  update.optimized_frame_count = 1;
  update.poses.push_back({0, {}});
  EXPECT_THROW(MakePoseSnapshot(update, {}, raw), std::invalid_argument);

  update.tail_policy = PoseTailPolicy::kProjectedLastOptimizedAnchor;
  const auto snapshot = MakePoseSnapshot(update, {}, raw);
  EXPECT_EQ(snapshot.tail_policy, update.tail_policy);
  ASSERT_EQ(snapshot.poses.size(), 2u);
  EXPECT_TRUE(ValidateExtrinsic(snapshot.poses[1].map_from_body).empty());
  EXPECT_NEAR(snapshot.poses[1].map_from_body.translation.x(), 1000, 1e-10);
  EXPECT_NEAR(snapshot.poses[1].map_from_body.rotation(0, 0), 1, 1e-12);
  EXPECT_DOUBLE_EQ(raw[0].rotation(0, 0), 1.000045);
  EXPECT_DOUBLE_EQ(raw[1].rotation(0, 0), 1.000045);

  auto grown_raw = raw;
  grown_raw.push_back(raw[1]);
  grown_raw.back().translation.x() = 2000;
  const auto grown = MaterializePoseSnapshot(snapshot, grown_raw, 3);
  EXPECT_NEAR(grown.poses[2].map_from_body.translation.x(), 2000, 1e-10);
  EXPECT_EQ(snapshot.frame_count, 2u);
}

TEST(PoseSnapshotTest, ProjectedTailUsesOneRigidCorrection) {
  auto raw = Raw(2);
  Eigen::Matrix3d rz;
  rz << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  Eigen::Matrix3d rx;
  rx << 1, 0, 0, 0, 0, -1, 0, 1, 0;
  Eigen::Matrix3d ry;
  ry << 0, 0, 1, 0, 1, 0, -1, 0, 0;
  const Eigen::Matrix3d stretch = Eigen::Vector3d(1.000045, 1, 1).asDiagonal();
  raw[0].rotation = rz * stretch;
  raw[0].translation = Eigen::Vector3d(1, 2, 0);
  raw[1].rotation = rx * stretch;
  raw[1].translation = Eigen::Vector3d(2, 4, 3);

  PoseUpdate update;
  update.optimized_frame_count = 1;
  update.tail_policy = PoseTailPolicy::kProjectedLastOptimizedAnchor;
  update.poses.push_back({0, {ry, Eigen::Vector3d(4, -1, 2)}});
  const auto snapshot = MakePoseSnapshot(update, {}, raw);
  const auto& tail = snapshot.poses[1].map_from_body;
  Eigen::Matrix3d expected_rotation;
  expected_rotation << 0, 1, 0, -1, 0, 0, 0, 0, 1;
  EXPECT_TRUE(tail.rotation.isApprox(expected_rotation, 1e-12));
  EXPECT_TRUE(tail.translation.isApprox(Eigen::Vector3d(7, -2, 0), 1e-12));
}

TEST(PoseSnapshotTest, ProjectedTailRejectsStretchedFrozenPose) {
  auto raw = Raw(2);
  PoseUpdate update;
  update.optimized_frame_count = 1;
  update.tail_policy = PoseTailPolicy::kProjectedLastOptimizedAnchor;
  update.poses.push_back({0, {}});
  auto snapshot = MakePoseSnapshot(update, {}, raw);
  snapshot.poses[1].map_from_body.rotation(0, 0) = 1.00004;
  ASSERT_TRUE(ValidateExtrinsic(snapshot.poses[1].map_from_body).empty());
  EXPECT_THROW(MaterializePoseSnapshot(snapshot, raw, 2),
               std::invalid_argument);

  auto version_zero = MaterializePoseSnapshot({}, raw, 2);
  version_zero.tail_policy = PoseTailPolicy::kProjectedLastOptimizedAnchor;
  EXPECT_THROW(ValidatePoseSnapshot(version_zero), std::invalid_argument);
}

TEST(PoseSnapshotTest, ProjectedUpdateNormalizesItsOwnPoseCopies) {
  const auto raw = Raw(2);
  PoseUpdate update;
  update.optimized_frame_count = 1;
  update.tail_policy = PoseTailPolicy::kProjectedLastOptimizedAnchor;
  update.poses.push_back({0, {}});
  update.poses[0].map_from_body.rotation(0, 0) = 1.000045;
  const auto snapshot = MakePoseSnapshot(update, {}, raw);
  EXPECT_NEAR(snapshot.poses[0].map_from_body.rotation(0, 0), 1, 1e-12);
  EXPECT_NEAR(snapshot.poses[1].map_from_body.translation.x(), 1, 1e-12);
  EXPECT_DOUBLE_EQ(update.poses[0].map_from_body.rotation(0, 0), 1.000045);
}

}  // namespace
}  // namespace livo_atlas
