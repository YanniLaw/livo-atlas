#include "livo_atlas/gaussian/gaussian_render.h"

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "livo_atlas/gaussian/gaussian_map_index.h"
#include "gaussian/render_metadata_test_helpers.h"

namespace livo_atlas {
namespace {

constexpr char kSession[] = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";

PoseSnapshot Poses(int count = 6, std::uint64_t version = 0) {
  PoseSnapshot result;
  result.version = version;
  result.base_version = version ? version - 1 : 0;
  result.frame_count = count;
  result.optimized_frame_count = version ? count - 1 : 0;
  result.parent_frame = version ? PoseFrame::kMap : PoseFrame::kOdom;
  result.tail_policy = version ? PoseTailPolicy::kProjectedLastOptimizedAnchor
                               : PoseTailPolicy::kLegacyLastOptimizedAnchor;
  for (int i = 0; i < count; ++i) {
    RigidTransform pose;
    pose.translation = {static_cast<double>(i), version ? 10.0 : 0.0, 0};
    result.poses.push_back({static_cast<std::uint64_t>(i), pose});
  }
  return result;
}

OnlineSubmap Group(int id, int count, bool sealed = false) {
  OnlineSubmap result;
  result.primary.id = id;
  result.primary.anchor_keyframe_id = 2 * id;
  for (int i = 0; i < count; ++i)
    result.primary.keyframe_ids.push_back(2 * id + i);
  result.membership_revision = count + (sealed ? 1 : 0);
  result.state =
      sealed ? OnlineSubmapState::kSealed : OnlineSubmapState::kActive;
  return result;
}

void Observe(GaussianMapIndex& index, const PoseSnapshot& poses,
             std::vector<OnlineSubmap> groups = {}) {
  OnlineSubmapUpdate update;
  update.poses = std::make_shared<const PoseSnapshot>(poses);
  update.revisions = std::move(groups);
  index.Observe(update);
}

GaussianAssetReference Asset(const GaussianContentId& content,
                             std::uint64_t id) {
  GaussianAssetReference result;
  result.asset_id = id;
  result.content = content;
  result.manifest_sha256 = std::string(64, 'b');
  result.manifest_bytes = 2048;
  result.gaussian_bytes = 1024;
  result.observation_bytes = 4096;
  return result;
}

class GaussianRenderTest : public testing::Test {
 protected:
  void SetUp() override {
    for (int count = 1; count <= 6; ++count) {
      const int id = (count - 1) / 2;
      std::vector<OnlineSubmap> revisions;
      if (count % 2 && id) revisions.push_back(Group(id - 1, 2, true));
      revisions.push_back(Group(id, count % 2 ? 1 : 2));
      Observe(index_, Poses(count), revisions);
    }
    Observe(index_, poses_, {Group(2, 2, true)});
    for (int id = 0; id < 3; ++id) {
      index_.RecordConfirmed(index_.MakeTarget(id, id + 1),
                             GaussianOperation::kView, id + 1,
                             {static_cast<std::uint64_t>(2 * id),
                              static_cast<std::uint64_t>(2 * id + 1)}, 2);
      ASSERT_TRUE(index_.RecordCheckpoint(
          Asset(*index_.Snapshot().entries[id].content, id + 1)));
    }
    // Repair/reuse can leave an older block resident than the desired camera.
    index_.RecordConfirmed(index_.MakeTarget(0, 4), GaussianOperation::kReuse,
                           4, {0, 1}, 2);
    map_ = index_.Snapshot();
  }

  GaussianRenderRequest Select() {
    std::string reason;
    auto result = SelectGaussianRender(map_, poses_, 4, &reason);
    if (!result) throw std::logic_error(reason);
    return *result;
  }

  GaussianMapIndex index_{kSession};
  PoseSnapshot poses_ = Poses();
  GaussianMapSnapshot map_;
};

TEST_F(GaussianRenderTest, NewestQueryComesFromAnAssetInsteadOfTheResident) {
  const auto result = Select();
  EXPECT_EQ(result.query_keyframe_id, 5u);
  EXPECT_EQ(result.query_submap_id, 2u);
  ASSERT_EQ(result.blocks.size(), 2u);
  EXPECT_EQ(result.blocks[0].target.group.primary.id, 1u);
  EXPECT_EQ(result.blocks[1].target.group.primary.id, 2u);
  EXPECT_FALSE(result.blocks[0].resident);
  EXPECT_FALSE(result.blocks[1].resident);
  EXPECT_EQ(result.parent_from_body.translation, Eigen::Vector3d(5, 0, 0));
  EXPECT_EQ(result.generation, 4u);
  EXPECT_TRUE(IsCurrentGaussianRender(result, map_, poses_));
}

TEST_F(GaussianRenderTest, UnavailableNewestQueryNeverFallsBackToOlderCameras) {
  map_.entries[2].availability = GaussianAvailability::kNeedsRebuild;
  std::string reason;
  EXPECT_FALSE(SelectGaussianRender(map_, poses_, 4, &reason));
  EXPECT_FALSE(reason.empty());
}

TEST_F(GaussianRenderTest, OldOrMissingQueryAssetsNeverReplaceConfirmation) {
  const auto original = map_;
  for (int mode = 0; mode < 3; ++mode) {
    map_ = original;
    auto& entry = map_.entries[2];
    if (mode == 0) entry.dirty = true;
    if (mode == 1) entry.asset.reset();
    if (mode == 2) ++entry.asset->content.content_job_id;
    EXPECT_FALSE(SelectGaussianRender(map_, poses_, 4, nullptr));
  }
}

TEST_F(GaussianRenderTest,
       CanonicalChangeOutsideRetainedFramesStillBlocksQuery) {
  index_.RecordConfirmed(index_.MakeTarget(2, 5), GaussianOperation::kView,
                         5, {5}, 1);
  auto changed = Poses(6, 1);
  changed.poses[4].map_from_body.translation.z() = 1;
  Observe(index_, changed);
  const auto map = index_.Snapshot();
  ASSERT_EQ(map.entries[2].availability, GaussianAvailability::kNeedsRebuild);
  EXPECT_FALSE(SelectGaussianRender(map, changed, 5, nullptr));
}

TEST_F(GaussianRenderTest, UnusableNeighborProducesOneExplicitBlock) {
  for (int mode = 0; mode < 4; ++mode) {
    auto original = map_;
    auto& entry = map_.entries[1];
    if (mode == 0) entry.availability = GaussianAvailability::kMissing;
    if (mode == 1) entry.dirty = true;
    if (mode == 2) entry.asset->content.content_job_id = 99;
    if (mode == 3) entry.asset_current = false;
    const auto result = Select();
    ASSERT_EQ(result.blocks.size(), 1u);
    EXPECT_EQ(result.blocks.front().target.group.primary.id, 2u);
    EXPECT_FALSE(result.neighbor_reason.empty());
    map_ = std::move(original);
  }
}

TEST_F(GaussianRenderTest, VolatileResidentUsesTheExactCpuConfirmation) {
  index_.RecordConfirmed(index_.MakeTarget(2, 5), GaussianOperation::kView,
                         5, {4, 5}, 2);
  map_ = index_.Snapshot();
  auto result = SelectGaussianRender(map_, poses_, 5, nullptr);
  ASSERT_TRUE(result);
  EXPECT_TRUE(result->blocks.back().resident);
  EXPECT_FALSE(result->blocks.back().asset);
  EXPECT_EQ(result->blocks.back().content.content_job_id, 5u);
}

TEST_F(GaussianRenderTest, OldCaptureKeepsCurrentBindingsAndProjectedTail) {
  poses_ = Poses(6, 1);
  Observe(index_, poses_);
  map_ = index_.Snapshot();
  const auto result = Select();
  EXPECT_EQ(result.pose_version, 1u);
  EXPECT_EQ(result.optimized_frame_count, 5u);
  EXPECT_EQ(result.parent_from_body.translation, Eigen::Vector3d(5, 10, 0));
  EXPECT_EQ(result.blocks.back().target.parent_from_submap.translation,
            Eigen::Vector3d(4, 10, 0));
  EXPECT_EQ(map_.entries[2].asset_source.pose_version, 0u);
}

TEST_F(GaussianRenderTest, EverySourceChangeInvalidatesAResult) {
  const auto request = Select();
  for (int mode = 0; mode < 6; ++mode) {
    auto changed = map_;
    if (mode == 0) ++changed.map_sequence;
    if (mode == 1) ++changed.pose_version;
    if (mode == 2) ++changed.entries[2].content->content_job_id;
    if (mode == 3) ++changed.entries[2].group.membership_revision;
    if (mode == 4) ++changed.entries[2].asset->asset_id;
    if (mode == 5)
      changed.entries[2].binding.parent_from_submap.translation.x() += 1;
    EXPECT_FALSE(IsCurrentGaussianRender(request, changed, poses_));
  }
  auto changed_poses = poses_;
  changed_poses.poses[5].map_from_body.translation.z() = 1;
  EXPECT_FALSE(IsCurrentGaussianRender(request, map_, changed_poses));
}

TEST_F(GaussianRenderTest, RequestOwnsCanonicalPosesAndCoverage) {
  const auto request = Select();
  map_.entries[2].binding.submap_from_body.back().translation.z() = 2;
  map_.entries[2].frame_ids.clear();
  EXPECT_EQ(
      request.blocks.back().target.submap_from_body.back().translation.z(), 0);
  EXPECT_EQ(request.blocks.back().frame_ids,
            (std::vector<std::uint64_t>{4, 5}));
}

TEST_F(GaussianRenderTest, InvalidBudgetsAndSourceIdentitiesAreRejected) {
  const auto request = Select();
  for (int mode = 0; mode < 6; ++mode) {
    auto changed = request;
    if (mode == 0) changed.blocks.push_back(changed.blocks.front());
    if (mode == 1) changed.blocks.back().gaussian_count = 10001;
    if (mode == 2) changed.query_keyframe_id = 6;
    if (mode == 3)
      changed.blocks.back().content.session_id = std::string(32, 'c');
    if (mode == 4) changed.parent_from_body.rotation(0, 0) = 2;
    if (mode == 5) changed.blocks.back().frame_ids = {4, 4};
    EXPECT_THROW(ValidateGaussianRenderRequest(changed), std::invalid_argument);
  }
}

TEST_F(GaussianRenderTest, OversizedFrontierIsRejectedEvenWithoutContent) {
  const auto poses = Poses(513);
  GaussianMapSnapshot map;
  map.session_id = kSession;
  map.map_sequence = 1;
  map.frame_count = 513;
  EXPECT_THROW(SelectGaussianRender(map, poses, 4, nullptr),
               std::invalid_argument);
}

class SpatialGaussianRenderTest : public testing::Test {
 protected:
  void SetUp() override {
    for (int count = 1; count <= 8; ++count) {
      const int id = (count - 1) / 2;
      std::vector<OnlineSubmap> groups;
      if (count % 2 && id) groups.push_back(Group(id - 1, 2, true));
      groups.push_back(Group(id, count % 2 ? 1 : 2));
      Observe(index_, Poses(count), groups);
    }
    Observe(index_, poses_, {Group(3, 2, true)});
    for (std::uint64_t id = 0; id < 4; ++id) {
      const std::vector<std::uint64_t> frames{2 * id, 2 * id + 1};
      auto metadata = test::RenderMetadata({kSession, id, id + 1, id + 1},
                                           frames);
      for (auto& extent : metadata.extents) {
        extent.min = extent.max = Eigen::Vector3d(
            7. - 2. * id, 0, id == 0 || id == 3 ? 4 : -4);
      }
      index_.RecordConfirmed(index_.MakeTarget(id, id + 1),
                             GaussianOperation::kView, id + 1, frames, 2,
                             metadata);
      ASSERT_TRUE(index_.RecordCheckpoint(Asset(metadata.content, id + 1)));
    }
    const auto metadata = index_.Snapshot().entries[2].render_metadata;
    index_.RecordConfirmed(index_.MakeTarget(2, 5), GaussianOperation::kReuse,
                           5, {4, 5}, 2, metadata);
    map_ = index_.Snapshot();
  }

  GaussianRenderRequest Select() {
    std::string reason;
    const auto result = SelectGaussianRender(
        map_, poses_, 5, GaussianRenderPolicy::kSpatial, &reason);
    if (!result) throw std::logic_error(reason);
    return *result;
  }

  void Place(std::size_t id, double x, double z) {
    for (auto& extent : map_.entries[id].render_metadata->extents)
      extent.min = extent.max = Eigen::Vector3d(x, 0, z);
  }

  GaussianMapIndex index_{kSession};
  PoseSnapshot poses_ = Poses(8);
  GaussianMapSnapshot map_;
};

TEST_F(SpatialGaussianRenderTest, SelectsNonadjacentAssetWithThirdResident) {
  const auto result = Select();
  ASSERT_EQ(result.blocks.size(), 2u);
  EXPECT_EQ(result.query_submap_id, 3u);
  EXPECT_EQ(result.query_keyframe_id, 7u);
  EXPECT_EQ(result.blocks[0].content.submap_id, 0u);
  EXPECT_EQ(result.blocks[1].content.submap_id, 3u);
  EXPECT_FALSE(result.blocks[0].resident);
  EXPECT_FALSE(result.blocks[1].resident);
  EXPECT_EQ(result.candidate_count, 1u);
  EXPECT_EQ(result.candidate_pixels, 48u);
  EXPECT_TRUE(result.blocks[0].render_metadata);
  EXPECT_TRUE(result.blocks[1].render_metadata);
  EXPECT_TRUE(IsCurrentGaussianRender(result, map_, poses_));
}

TEST_F(SpatialGaussianRenderTest, TemporalDefaultKeepsPreviousLogicalGroup) {
  const auto result = SelectGaussianRender(map_, poses_, 5, nullptr);
  ASSERT_TRUE(result);
  ASSERT_EQ(result->blocks.size(), 2u);
  EXPECT_EQ(result->policy, GaussianRenderPolicy::kTemporal);
  EXPECT_EQ(result->blocks[0].content.submap_id, 2u);
  EXPECT_FALSE(result->blocks[0].render_metadata);
  EXPECT_EQ(result->candidate_pixels, 0u);
}

TEST_F(SpatialGaussianRenderTest, AreaDistanceThenIdDefineOneExactOrder) {
  Place(1, 5, 4);
  Place(2, 3, 4);
  auto result = Select();
  EXPECT_EQ(result.candidate_count, 3u);
  EXPECT_EQ(result.blocks[0].content.submap_id, 0u);
  Place(1, 5, 3);
  result = Select();
  EXPECT_EQ(result.blocks[0].content.submap_id, 1u);
  // The closer old0 footprint touches only the right edge; old1 covers more.
  Place(0, 9, 1);
  Place(1, 5, 4);
  Place(2, 3, -4);
  result = Select();
  EXPECT_EQ(result.blocks[0].content.submap_id, 1u);
  EXPECT_EQ(result.candidate_count, 2u);
}

TEST_F(SpatialGaussianRenderTest, NoCandidateHasAnExplicitSingleBlockReason) {
  Place(0, 7, -4);
  const auto result = Select();
  ASSERT_EQ(result.blocks.size(), 1u);
  EXPECT_EQ(result.neighbor_reason, "no_spatial_candidate");
  EXPECT_EQ(result.candidate_count, 0u);
  EXPECT_EQ(result.candidate_pixels, 0u);
}

TEST_F(SpatialGaussianRenderTest, UnavailableLatestMetadataNeverFallsBack) {
  map_.entries[3].render_metadata.reset();
  std::string reason;
  EXPECT_FALSE(SelectGaussianRender(map_, poses_, 5,
                                    GaussianRenderPolicy::kSpatial, &reason));
  EXPECT_EQ(reason, "query_unavailable");
}

TEST_F(SpatialGaussianRenderTest, ChangedMetadataInvalidatesEvenTheSameChoice) {
  const auto result = Select();
  auto timing_only = result;
  timing_only.selection_seconds += 10;
  EXPECT_TRUE(IsCurrentGaussianRender(timing_only, map_, poses_));
  map_.entries[0].render_metadata->extents[0].max.x() += 1;
  EXPECT_FALSE(IsCurrentGaussianRender(result, map_, poses_));
  map_.entries[0].render_metadata = result.blocks[0].render_metadata;
  map_.entries[3].render_metadata->camera.capture_time += .1;
  EXPECT_FALSE(IsCurrentGaussianRender(result, map_, poses_));
}

TEST_F(SpatialGaussianRenderTest, UnselectedMetadataStillBelongsToThisMap) {
  auto& entry = map_.entries[1];  // Behind the query, so it is never selected.
  entry.content->session_id = std::string(32, 'c');
  entry.render_metadata->content = *entry.content;
  entry.asset->content = *entry.content;
  EXPECT_THROW(Select(), std::invalid_argument);
}

TEST(SpatialGaussianRenderBudget, OneAndThirtyTwoGroupsFitButThirtyThreeFail) {
  for (int groups : {1, 32, 33}) {
    const auto poses = Poses(2 * groups);
    GaussianMapSnapshot map;
    map.session_id = kSession;
    map.map_sequence = 1;
    map.frame_count = poses.frame_count;
    for (int id = 0; id < groups; ++id) {
      GaussianSubmapEntry entry;
      entry.group = Group(id, 2, true);
      entry.availability = GaussianAvailability::kCurrent;
      auto& target = entry.binding;
      target.session_id = kSession;
      target.generation = id + 1;
      target.frame_count = poses.frame_count;
      target.group = entry.group;
      target.parent_from_submap.translation.x() = 2 * id;
      target.submap_from_body.resize(2);
      target.submap_from_body[1].translation.x() = 1;
      entry.content_target = entry.asset_source = target;
      entry.content = GaussianContentId{
          kSession, static_cast<std::uint64_t>(id),
          static_cast<std::uint64_t>(id + 1),
          static_cast<std::uint64_t>(id + 1)};
      entry.frame_ids = entry.asset_frame_ids =
          target.group.primary.keyframe_ids;
      entry.gaussian_count = entry.asset_gaussian_count = 2;
      entry.asset = Asset(*entry.content, id + 1);
      entry.asset_current = true;
      entry.render_metadata = test::RenderMetadata(*entry.content,
                                                   entry.frame_ids);
      for (auto& extent : entry.render_metadata->extents)
        extent.min = extent.max = Eigen::Vector3d(
            1, 0, id + 1 == groups ? 4 : -4);
      map.entries.push_back(entry);
    }
    if (groups == 33) {
      EXPECT_THROW(SelectGaussianRender(map, poses, 34,
                       GaussianRenderPolicy::kSpatial, nullptr),
                   std::invalid_argument);
    } else {
      const auto result = SelectGaussianRender(map, poses, groups + 1,
          GaussianRenderPolicy::kSpatial, nullptr);
      ASSERT_TRUE(result);
      EXPECT_EQ(result->blocks.size(), 1u);
      EXPECT_EQ(result->neighbor_reason, "no_spatial_candidate");
    }
  }
}

TEST_F(SpatialGaussianRenderTest, BadPartitionsAndForgedMetadataAreErrors) {
  const auto original = map_;
  map_.entries[1].group.primary.keyframe_ids = {1, 3};
  EXPECT_THROW(Select(), std::invalid_argument);
  map_ = original;
  ++map_.entries[0].render_metadata->content.content_job_id;
  EXPECT_THROW(Select(), std::invalid_argument);
}

TEST_F(SpatialGaussianRenderTest, RequestValidatesSpatialSummaryAndSources) {
  const auto original = Select();
  for (int mode = 0; mode < 5; ++mode) {
    auto request = original;
    if (mode == 0) request.candidate_count = 32;
    if (mode == 1) request.candidate_pixels = 49;
    if (mode == 2) request.blocks[0].render_metadata.reset();
    if (mode == 3) request.neighbor_reason = "no_spatial_candidate";
    if (mode == 4) request.policy = static_cast<GaussianRenderPolicy>(2);
    EXPECT_THROW(ValidateGaussianRenderRequest(request),
                 std::invalid_argument) << mode;
  }
}

}  // namespace
}  // namespace livo_atlas
