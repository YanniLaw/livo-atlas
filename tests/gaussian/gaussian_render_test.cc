#include "livo_atlas/gaussian/gaussian_render.h"

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "livo_atlas/gaussian/gaussian_map_index.h"

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

}  // namespace
}  // namespace livo_atlas
