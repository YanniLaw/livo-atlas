#include "livo_atlas/gaussian/gaussian_map_index.h"

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "gaussian/render_metadata_test_helpers.h"

namespace livo_atlas {
namespace {

constexpr char kSession[] = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";

PoseSnapshot Poses(int count, std::uint64_t version = 0, bool rotated = false) {
  PoseSnapshot result;
  result.version = version;
  result.base_version = version ? version - 1 : 0;
  result.frame_count = count;
  result.optimized_frame_count = version ? count : 0;
  result.parent_frame = version ? PoseFrame::kMap : PoseFrame::kOdom;
  for (int i = 0; i < count; ++i) {
    RigidTransform pose;
    pose.translation = {static_cast<double>(i), 0, 0};
    if (rotated) {
      pose.rotation << 0, -1, 0, 1, 0, 0, 0, 0, 1;
      pose.translation = {4, static_cast<double>(i) - 2, 1};
    }
    result.poses.push_back({static_cast<std::uint64_t>(i), pose});
  }
  return result;
}

OnlineSubmap Group(std::uint64_t id, std::uint64_t first, std::uint64_t count,
                   bool sealed = false) {
  OnlineSubmap result;
  result.primary.id = id;
  result.primary.anchor_keyframe_id = first;
  for (std::uint64_t i = 0; i < count; ++i)
    result.primary.keyframe_ids.push_back(first + i);
  result.membership_revision = count + (sealed ? 1 : 0);
  result.state =
      sealed ? OnlineSubmapState::kSealed : OnlineSubmapState::kActive;
  return result;
}

void Observe(GaussianMapIndex& index, const PoseSnapshot& poses,
             std::vector<OnlineSubmap> revisions = {}) {
  OnlineSubmapUpdate update;
  update.poses = std::make_shared<const PoseSnapshot>(poses);
  update.revisions = std::move(revisions);
  index.Observe(update);
}

void Populate(GaussianMapIndex& index, int count) {
  for (int i = 1; i <= count; ++i) Observe(index, Poses(i), {Group(0, 0, i)});
}

GaussianAssetReference Asset(const GaussianContentId& content,
                             std::uint64_t id = 1) {
  GaussianAssetReference result;
  result.asset_id = id;
  result.content = content;
  result.manifest_sha256 = std::string(64, 'b');
  result.manifest_bytes = 2048;
  result.gaussian_bytes = 1024;
  result.observation_bytes = 4096;
  return result;
}

TEST(GaussianMapIndexTest, MetadataFollowsConfirmedContentAndRigidReuse) {
  GaussianMapIndex index(kSession);
  Populate(index, 3);
  const auto metadata = test::RenderMetadata({kSession, 0, 1, 1}, {0, 2});
  EXPECT_FALSE(index.Snapshot().entries[0].render_metadata);
  index.RecordConfirmed(index.MakeTarget(0, 1), GaussianOperation::kView,
                        1, {0, 2}, 2, metadata);
  ASSERT_EQ(index.Snapshot().entries[0].render_metadata, metadata);
  ASSERT_TRUE(index.RecordCheckpoint(Asset(metadata.content)));
  Observe(index, Poses(3, 1, true));
  index.RecordConfirmed(index.MakeTarget(0, 2), GaussianOperation::kReuse,
                        2, {0, 2}, 2, metadata);
  EXPECT_EQ(index.Snapshot().entries[0].render_metadata, metadata);
  auto altered = metadata;
  altered.extents.front().max.x() = 1;
  EXPECT_THROW(index.RecordConfirmed(index.MakeTarget(0, 3),
                   GaussianOperation::kReuse, 3, {0, 2}, 2, altered),
               std::invalid_argument);
  EXPECT_EQ(index.Snapshot().entries[0].render_metadata, metadata);
  index.RecordResidentReleased();
  EXPECT_EQ(index.Snapshot().entries[0].render_metadata, metadata);
}

TEST(GaussianMapIndexTest, InvalidMetadataCannotPartiallyConfirmNewContent) {
  GaussianMapIndex index(kSession);
  Populate(index, 3);
  const auto original = test::RenderMetadata({kSession, 0, 1, 1}, {0, 2});
  index.RecordConfirmed(index.MakeTarget(0, 1), GaussianOperation::kView,
                        1, {0, 2}, 2, original);
  const auto sequence = index.Snapshot().map_sequence;
  for (int mode = 0; mode < 7; ++mode) {
    auto value = test::RenderMetadata({kSession, 0, 2, 2}, {0, 2});
    if (mode == 0) value.content.content_job_id = 3;
    if (mode == 1) value.camera.keyframe_id = 0;
    if (mode == 2) value.extents.front().max_scale = 0;
    if (mode == 3) value.extents.front().min.x() = 1;
    if (mode == 4) value.camera.K(0, 0) = -1;
    if (mode == 5) value.camera.camera_from_body.rotation(0, 0) += 2e-6;
    if (mode == 6) value.extents.pop_back();
    EXPECT_THROW(index.RecordConfirmed(index.MakeTarget(0, 2),
                     GaussianOperation::kView, 2, {0, 2}, 2, value),
                 std::invalid_argument) << mode;
    EXPECT_EQ(index.Snapshot().map_sequence, sequence);
    EXPECT_EQ(index.Snapshot().entries[0].render_metadata, original);
    EXPECT_EQ(index.Snapshot().entries[0].content->content_job_id, 1u);
  }
}

TEST(GaussianMapIndexTest, RebuildReplacesMetadataWithoutRewritingOldAsset) {
  GaussianMapIndex index(kSession);
  Populate(index, 3);
  const auto old = test::RenderMetadata({kSession, 0, 1, 1}, {0, 2});
  index.RecordConfirmed(index.MakeTarget(0, 1), GaussianOperation::kView,
                        1, {0, 2}, 2, old);
  ASSERT_TRUE(index.RecordCheckpoint(Asset(old.content)));
  auto changed = Poses(3, 1);
  changed.poses[1].map_from_body.translation.y() += .5;
  Observe(index, changed);
  EXPECT_EQ(index.Snapshot().entries[0].render_metadata, old);
  const auto fresh = test::RenderMetadata({kSession, 0, 2, 2}, {0, 2});
  index.RecordConfirmed(index.MakeTarget(0, 2), GaussianOperation::kRebuild,
                        2, {0, 2}, 2, fresh);
  const auto entry = index.Snapshot().entries[0];
  EXPECT_EQ(entry.render_metadata, fresh);
  EXPECT_EQ(entry.asset->content, old.content);
  EXPECT_TRUE(entry.dirty);
}

TEST(GaussianMapIndexTest, ThirdResidentKeepsEarlierConfirmedMetadata) {
  GaussianMapIndex index(kSession);
  for (int i = 0; i < 3; ++i) {
    std::vector<OnlineSubmap> groups;
    if (i) groups.push_back(Group(i - 1, i - 1, 1, true));
    groups.push_back(Group(i, i, 1));
    Observe(index, Poses(i + 1), groups);
    const auto metadata = test::RenderMetadata(
        {kSession, static_cast<std::uint64_t>(i),
         static_cast<std::uint64_t>(i + 1),
         static_cast<std::uint64_t>(i + 1)},
        {static_cast<std::uint64_t>(i)});
    index.RecordConfirmed(index.MakeTarget(i, i + 1),
                          GaussianOperation::kView, i + 1,
                          {static_cast<std::uint64_t>(i)}, 1, metadata);
    ASSERT_TRUE(index.RecordCheckpoint(Asset(metadata.content, i + 1)));
  }
  const auto map = index.Snapshot();
  for (int i = 0; i < 3; ++i) {
    ASSERT_TRUE(map.entries[i].render_metadata);
    EXPECT_EQ(map.entries[i].render_metadata->camera.keyframe_id,
              static_cast<std::uint64_t>(i));
    EXPECT_EQ(map.entries[i].resident, i == 2);
  }
}

TEST(GaussianMapIndexTest, MetadataRejectsNonfiniteAndOverBudgetValues) {
  const auto valid = test::RenderMetadata({kSession, 0, 1, 1}, {0, 1});
  for (int mode = 0; mode < 7; ++mode) {
    auto value = valid;
    if (mode == 0) value.extents[0].max.x() =
        std::numeric_limits<double>::infinity();
    if (mode == 1) value.extents[0].max_scale =
        std::numeric_limits<double>::quiet_NaN();
    if (mode == 2) value.camera.width = 1000001;
    if (mode == 3) value.camera.K(0, 1) = 1;
    if (mode == 4) value.extents.resize(11);
    if (mode == 5) value.camera.calibration_id = 1;
    if (mode == 6) value.content.session_id = "bad";
    EXPECT_THROW(ValidateGaussianRenderMetadata(value, valid.content, {0, 1}),
                 std::invalid_argument) << mode;
  }
  EXPECT_THROW(ValidateGaussianRenderMetadata(valid, valid.content, {}),
               std::invalid_argument);
}

TEST(GaussianMapIndexTest,
     WholeBlockMotionKeepsCanonicalContentAndUpdatesBinding) {
  GaussianMapIndex index(kSession);
  Populate(index, 3);
  index.RecordConfirmed(index.MakeTarget(0, 1), GaussianOperation::kView, 1,
                        {0, 2}, 2);
  const auto before = index.Snapshot().entries.front();
  ASSERT_TRUE(index.RecordCheckpoint(Asset(*before.content)));
  Observe(index, Poses(3, 3, true));
  const auto entry = index.Snapshot().entries.front();
  EXPECT_EQ(entry.availability, GaussianAvailability::kCurrent);
  EXPECT_TRUE(entry.asset_current);
  EXPECT_EQ(entry.binding.pose_version, 3u);
  EXPECT_TRUE(entry.binding.parent_from_submap.translation.isApprox(
      Eigen::Vector3d(4, -2, 1), 1e-15));
  EXPECT_TRUE(entry.content_target.submap_from_body[2].translation.isApprox(
      Eigen::Vector3d(2, 0, 0), 1e-15));
  EXPECT_EQ(entry.content->content_job_id, 1u);
  EXPECT_EQ(entry.asset->manifest_sha256, std::string(64, 'b'));
  index.RecordConfirmed(index.MakeTarget(0, 4), GaussianOperation::kReuse, 2,
                        {0, 2}, 2);
  const auto rebound = index.Snapshot().entries.front();
  EXPECT_EQ(rebound.content->source_generation, 1u);
  EXPECT_EQ(rebound.content->content_job_id, 1u);
  EXPECT_EQ(rebound.binding_job_id, 2u);
  EXPECT_FALSE(rebound.dirty);
}

TEST(GaussianMapIndexTest, ChangedEvictedPrimaryRequiresRebuild) {
  GaussianMapIndex index(kSession);
  Populate(index, 12);
  index.RecordConfirmed(index.MakeTarget(0, 1), GaussianOperation::kView, 1,
                        {10, 11}, 2);
  ASSERT_TRUE(
      index.RecordCheckpoint(Asset(*index.Snapshot().entries[0].content)));
  auto latest = Poses(12, 1);
  latest.poses[1].map_from_body.translation.y() = 0.5;
  Observe(index, latest);
  const auto entry = index.Snapshot().entries.front();
  EXPECT_EQ(entry.availability, GaussianAvailability::kNeedsRebuild);
  EXPECT_FALSE(entry.asset_current);
  EXPECT_EQ(entry.content_target.submap_from_body.size(), 12u);
  EXPECT_EQ(index.NextRepair(), 0u);
}

TEST(GaussianMapIndexTest, SameVersionFrontierGrowthPreservesOlderCoverage) {
  GaussianMapIndex index(kSession);
  Populate(index, 2);
  index.RecordConfirmed(index.MakeTarget(0, 1), GaussianOperation::kView, 1,
                        {1}, 1);
  Observe(index, Poses(3), {Group(0, 0, 3)});
  const auto entry = index.Snapshot().entries.front();
  EXPECT_EQ(entry.availability, GaussianAvailability::kCurrent);
  EXPECT_EQ(entry.group.primary.keyframe_ids.size(), 3u);
  EXPECT_EQ(entry.content_target.group.primary.keyframe_ids.size(), 2u);
  EXPECT_EQ(entry.frame_ids, (std::vector<std::uint64_t>{1}));
  EXPECT_EQ(entry.binding.frame_count, 3u);
}

TEST(GaussianMapIndexTest, VersionJumpUsesAbsoluteTarget) {
  GaussianMapIndex index(kSession);
  Populate(index, 2);
  index.RecordConfirmed(index.MakeTarget(0, 1), GaussianOperation::kView, 1,
                        {1}, 1);
  auto changed = Poses(2, 1);
  changed.poses[1].map_from_body.translation.y() = 1;
  Observe(index, changed);
  ASSERT_EQ(index.Snapshot().entries[0].availability,
            GaussianAvailability::kNeedsRebuild);
  Observe(index, Poses(2, 3, true));
  EXPECT_EQ(index.Snapshot().entries[0].availability,
            GaussianAvailability::kCurrent);
  EXPECT_EQ(index.MakeTarget(0, 5).pose_version, 3u);
}

TEST(GaussianMapIndexTest, RepeatedSmallChangesDoNotMoveCanonicalBaseline) {
  GaussianMapIndex index(kSession);
  Populate(index, 2);
  index.RecordConfirmed(index.MakeTarget(0, 1), GaussianOperation::kView, 1,
                        {1}, 1);
  auto first = Poses(2, 1);
  first.poses[1].map_from_body.translation.y() = 0.75e-9;
  Observe(index, first);
  ASSERT_EQ(index.Snapshot().entries[0].availability,
            GaussianAvailability::kCurrent);
  index.RecordConfirmed(index.MakeTarget(0, 2), GaussianOperation::kReuse, 2,
                        {1}, 1);
  auto second = Poses(2, 2);
  second.poses[1].map_from_body.translation.y() = 1.5e-9;
  Observe(index, second);
  EXPECT_EQ(index.Snapshot().entries[0].availability,
            GaussianAvailability::kNeedsRebuild);
  EXPECT_DOUBLE_EQ(index.Snapshot()
                       .entries[0]
                       .content_target.submap_from_body[1]
                       .translation.y(),
                   0);
}

TEST(GaussianMapIndexTest, LogicalTailAndAssetCoverageRemainDistinct) {
  GaussianMapIndex index(kSession);
  Populate(index, 2);
  index.RecordConfirmed(index.MakeTarget(0, 1), GaussianOperation::kView, 1,
                        {1}, 1);
  ASSERT_TRUE(
      index.RecordCheckpoint(Asset(*index.Snapshot().entries[0].content)));
  Observe(index, Poses(3), {Group(0, 0, 3)});
  index.RecordConfirmed(index.MakeTarget(0, 1), GaussianOperation::kView, 2,
                        {1, 2}, 2);
  const auto entry = index.Snapshot().entries[0];
  EXPECT_EQ(entry.content->content_job_id, 2u);
  EXPECT_EQ(entry.asset->content.content_job_id, 1u);
  EXPECT_EQ(entry.asset_source.frame_count, 2u);
  EXPECT_EQ(entry.asset_frame_ids, (std::vector<std::uint64_t>{1}));
  EXPECT_TRUE(entry.dirty);
}

TEST(GaussianMapIndexTest, LateConfirmationAndCheckpointCannotRegressTarget) {
  GaussianMapIndex index(kSession);
  Populate(index, 2);
  const auto old = index.MakeTarget(0, 1);
  index.RecordConfirmed(old, GaussianOperation::kView, 1, {1}, 1);
  const auto old_asset = Asset(*index.Snapshot().entries[0].content);
  auto latest = Poses(2, 3);
  latest.poses[1].map_from_body.translation.y() = 1;
  Observe(index, latest);
  index.RecordConfirmed(old, GaussianOperation::kView, 2, {1}, 1);
  EXPECT_FALSE(index.RecordCheckpoint(old_asset));
  EXPECT_EQ(index.Snapshot().entries[0].binding.pose_version, 3u);
  EXPECT_EQ(index.Snapshot().entries[0].availability,
            GaussianAvailability::kNeedsRebuild);
}

TEST(GaussianMapIndexTest,
     ThirtyThirdContentSlotIsMissingWithoutEvictingOldAssets) {
  GaussianMapIndex index(kSession);
  for (int i = 0; i < 33; ++i) {
    std::vector<OnlineSubmap> revisions;
    if (i) revisions.push_back(Group(i - 1, i - 1, 1, true));
    revisions.push_back(Group(i, i, 1));
    Observe(index, Poses(i + 1), revisions);
    if (i < 32) {
      ASSERT_TRUE(index.CanTrain(i));
      index.RecordConfirmed(index.MakeTarget(i, i + 1),
                            GaussianOperation::kView, i + 1,
                            {static_cast<std::uint64_t>(i)}, 1);
      ASSERT_TRUE(index.RecordCheckpoint(
          Asset(*index.Snapshot().entries[i].content, i + 1)));
    }
  }
  EXPECT_FALSE(index.CanTrain(32));
  const auto snapshot = index.Snapshot();
  ASSERT_EQ(snapshot.entries.size(), 33u);
  EXPECT_EQ(snapshot.entries.back().availability,
            GaussianAvailability::kMissing);
  EXPECT_EQ(snapshot.entries[0].asset->asset_id, 1u);
  EXPECT_EQ(snapshot.entries[31].asset->asset_id, 32u);
}

TEST(GaussianMapIndexTest, RepeatedUpdatesStayBounded) {
  GaussianMapIndex index(kSession);
  Populate(index, 3);
  index.RecordConfirmed(index.MakeTarget(0, 1), GaussianOperation::kView, 1,
                        {2}, 1);
  for (int version = 1; version < 1000; ++version)
    Observe(index, Poses(3, version, true));
  const auto snapshot = index.Snapshot();
  ASSERT_EQ(snapshot.entries.size(), 1u);
  EXPECT_EQ(snapshot.entries[0].content_target.submap_from_body.size(), 3u);
  EXPECT_EQ(snapshot.entries[0].frame_ids.size(), 1u);
}

TEST(GaussianMapIndexTest, InvalidUpdateDoesNotMutateExistingDirectory) {
  GaussianMapIndex index(kSession);
  Populate(index, 2);
  EXPECT_THROW(Observe(index, Poses(2), {Group(0, 0, 4)}),
               std::invalid_argument);
  EXPECT_EQ(index.Snapshot().entries[0].group.primary.keyframe_ids.size(), 2u);
  EXPECT_THROW(Observe(index, Poses(513)), std::invalid_argument);
  EXPECT_EQ(index.Snapshot().frame_count, 2u);
}

TEST(GaussianMapIndexTest, RepairsRotateAcrossBlocksUnderRepeatedPoseUpdates) {
  GaussianMapIndex index(kSession);
  Populate(index, 2);
  index.RecordConfirmed(index.MakeTarget(0, 1), GaussianOperation::kView, 1,
                        {1}, 1);
  Observe(index, Poses(3), {Group(0, 0, 2, true), Group(1, 2, 1)});
  Observe(index, Poses(4), {Group(1, 2, 2)});
  index.RecordConfirmed(index.MakeTarget(1, 2), GaussianOperation::kView, 2,
                        {3}, 1);
  auto latest = Poses(4, 1);
  latest.poses[1].map_from_body.translation.y() = 1;
  latest.poses[3].map_from_body.translation.y() = 1;
  Observe(index, latest);
  ASSERT_EQ(index.NextRepair(), 0u);
  index.MarkRepairAttempt(0);
  latest.version = 2;
  latest.base_version = 1;
  Observe(index, latest);
  EXPECT_EQ(index.NextRepair(), 1u);
  index.MarkRepairAttempt(1);
  EXPECT_EQ(index.NextRepair(), 0u);
}

TEST(GaussianMapIndexTest, InvalidConfirmationAndAssetLeaveContentUnchanged) {
  GaussianMapIndex index(kSession);
  Populate(index, 2);
  const auto target = index.MakeTarget(0, 1);
  EXPECT_THROW(
      index.RecordConfirmed(target, GaussianOperation::kView, 1, {2}, 1),
      std::invalid_argument);
  EXPECT_FALSE(index.Snapshot().entries[0].content);
  index.RecordConfirmed(target, GaussianOperation::kView, 1, {1}, 1);
  auto invalid = Asset(*index.Snapshot().entries[0].content);
  invalid.observation_bytes = 193 * 1024 * 1024;
  EXPECT_THROW(index.RecordCheckpoint(invalid), std::invalid_argument);
  EXPECT_FALSE(index.Snapshot().entries[0].asset);
  EXPECT_THROW(
      index.RecordConfirmed(target, GaussianOperation::kReuse, 2, {0}, 1),
      std::invalid_argument);
  EXPECT_EQ(index.Snapshot().entries[0].content->content_job_id, 1u);
}

}  // namespace
}  // namespace livo_atlas
