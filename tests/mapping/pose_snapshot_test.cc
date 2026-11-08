#include "livo_atlas/mapping/pose_snapshot.h"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

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
}  // namespace
}  // namespace livo_atlas
