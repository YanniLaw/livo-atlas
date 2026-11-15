#include "livo_atlas/mapping/geometry_map_builder.h"

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

namespace livo_atlas {
namespace {

PointCloud::ConstPtr Cloud(float x, float intensity = 4) {
  PointCloud::Ptr cloud(new PointCloud);
  Point point;
  point.x = x;
  point.y = point.z = 0;
  point.intensity = intensity;
  cloud->push_back(point);
  return cloud;
}

std::shared_ptr<PoseSnapshot> Poses(std::size_t count,
                                    std::uint64_t version = 0) {
  auto poses = std::make_shared<PoseSnapshot>();
  poses->version = version;
  poses->base_version = version ? version - 1 : 0;
  poses->frame_count = count;
  poses->optimized_frame_count = version ? count : 0;
  poses->parent_frame = version ? PoseFrame::kMap : PoseFrame::kOdom;
  if (version)
    poses->tail_policy = PoseTailPolicy::kProjectedLastOptimizedAnchor;
  for (std::size_t i = 0; i < count; ++i) {
    RigidTransform pose;
    pose.translation.x() = i / 2 * 3 + i % 2 * 0.5;
    poses->poses.push_back({i, pose});
  }
  return poses;
}

OnlineSubmap Group(std::uint64_t id, std::vector<std::uint64_t> members,
                   bool sealed, std::vector<std::uint64_t> support = {}) {
  OnlineSubmap group;
  group.primary.id = id;
  group.primary.anchor_keyframe_id = members.front();
  group.primary.keyframe_ids = std::move(members);
  group.support_keyframe_ids = std::move(support);
  group.state =
      sealed ? OnlineSubmapState::kSealed : OnlineSubmapState::kActive;
  group.membership_revision = group.primary.keyframe_ids.size() + sealed;
  return group;
}

std::vector<OnlineSubmap> ThreeGroups() {
  return {Group(0, {0, 1}, true), Group(1, {2, 3}, true, {1}),
          Group(2, {4}, false, {3})};
}

TEST(GeometryMapBuilderTest, CachesFullLocalFragmentsAndUsesPrimaryOnly) {
  GeometryOptions geometry;
  geometry.max_resident_submaps = 2;
  int reads = 0;
  GeometryMapBuilder builder(geometry, {}, [&](std::uint64_t id) {
    ++reads;
    return Cloud(0.05f, static_cast<float>(id + 1));
  });
  const auto raw = Poses(5);
  const auto first = builder.Build(ThreeGroups(), raw);
  ASSERT_EQ(first.status, GeometryMapStatus::kReady);
  EXPECT_EQ(first.frame_count, 5u);
  EXPECT_EQ(first.submap_count, 3u);
  EXPECT_EQ(first.source_voxel_count, 5u);
  EXPECT_EQ(first.representation, "voxel_moments_v1");
  EXPECT_EQ(first.fusion_policy, "primary_only");
  EXPECT_EQ(first.sampling_policy, "geometry_rank_v1");
  ASSERT_EQ(first.sources.size(), 3u);
  ASSERT_EQ(first.points.size(), 5u);
  EXPECT_EQ(first.sources[0].input_point_count, 2u);
  EXPECT_EQ(first.sources[1].input_point_count, 2u);
  EXPECT_EQ(first.sources[2].input_point_count, 1u);
  EXPECT_EQ(reads, 5);
  const double positions[] = {0.05, 0.55, 3.05, 3.55, 6.05};
  for (std::size_t i = 0; i < first.points.size(); ++i)
    EXPECT_NEAR(first.points[i].position.x(), positions[i], 1e-6);
  const auto second = builder.Build(ThreeGroups(), raw);
  EXPECT_EQ(reads, 5);
  EXPECT_EQ(builder.GetStats().cache_hits, 3u);
  ASSERT_EQ(second.points.size(), first.points.size());
  EXPECT_LE(builder.GetStats().peak_cached_points, 300000u);

  auto rigid = Poses(5, 1);
  const auto rotation =
      Eigen::AngleAxisd(1.5707963267948966, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  for (auto& pose : rigid->poses) {
    pose.map_from_body.translation =
        rotation * pose.map_from_body.translation + Eigen::Vector3d(3, -2, 1);
    pose.map_from_body.rotation = rotation;
  }
  const auto moved = builder.Build(ThreeGroups(), rigid);
  EXPECT_EQ(reads, 5);
  EXPECT_EQ(moved.pose_version, 1u);
  EXPECT_EQ(moved.parent_frame, PoseFrame::kMap);
  EXPECT_EQ(moved.sources[0].built_pose_version, 0u);
  EXPECT_EQ(moved.sources[0].representation_revision,
            first.sources[0].representation_revision);
  EXPECT_NEAR(moved.points[0].position.x(), 3, 1e-6);
  EXPECT_NEAR(moved.points[0].position.y(), -1.95, 1e-6);

  auto relative = std::make_shared<PoseSnapshot>(*rigid);
  relative->version = 2;
  relative->base_version = 1;
  relative->poses[1].map_from_body.translation.x() += 0.3;
  const auto rebuilt = builder.Build(ThreeGroups(), relative);
  EXPECT_EQ(reads, 7);
  EXPECT_GT(rebuilt.sources[0].representation_revision,
            moved.sources[0].representation_revision);
  EXPECT_EQ(rebuilt.sources[0].built_pose_version, 2u);
  EXPECT_EQ(rebuilt.sources[1].built_pose_version, 0u);
}

TEST(GeometryMapBuilderTest, SealReusesAndAppendRebuildsOnlyAffectedBlock) {
  int reads = 0;
  GeometryMapBuilder builder({}, {}, [&](std::uint64_t) {
    ++reads;
    return Cloud(0.05f);
  });
  const auto first = builder.Build({Group(0, {0, 1}, false)}, Poses(2));
  const auto sealed = builder.Build(
      {Group(0, {0, 1}, true), Group(1, {2}, false, {1})}, Poses(3));
  EXPECT_EQ(reads, 3);
  EXPECT_EQ(sealed.sources[0].state, OnlineSubmapState::kSealed);
  EXPECT_EQ(sealed.sources[0].representation_revision,
            first.sources[0].representation_revision);
  const auto appended = builder.Build(
      {Group(0, {0, 1}, true), Group(1, {2, 3}, false, {1})}, Poses(4));
  EXPECT_EQ(reads, 5);
  EXPECT_GT(appended.sources[1].representation_revision,
            sealed.sources[1].representation_revision);
  EXPECT_EQ(appended.points.size(), 4u);
}

TEST(GeometryMapBuilderTest, EvictionAndDisabledCachePreserveFixedCandidates) {
  GeometryOptions geometry;
  geometry.max_resident_submaps = 2;
  GeometryMapOptions tiny;
  tiny.max_cached_points = 1;
  int reads = 0;
  GeometryMapBuilder builder(geometry, tiny, [&](std::uint64_t) {
    ++reads;
    return Cloud(0.05f);
  });
  const auto first = builder.Build(ThreeGroups(), Poses(5));
  const auto second = builder.Build(ThreeGroups(), Poses(5));
  EXPECT_GT(reads, 5);
  EXPECT_LE(builder.GetStats().peak_cached_points, 1u);
  ASSERT_EQ(first.points.size(), second.points.size());
  for (std::size_t i = 0; i < first.points.size(); ++i)
    EXPECT_TRUE(first.points[i].position.isApprox(second.points[i].position));
  tiny.max_cached_points = 0;
  GeometryMapBuilder no_cache(geometry, tiny,
                              [](std::uint64_t) { return Cloud(0.05f); });
  const auto uncached = no_cache.Build(ThreeGroups(), Poses(5));
  EXPECT_EQ(no_cache.GetStats().peak_cached_points, 0u);
  ASSERT_EQ(uncached.points.size(), first.points.size());
  for (std::size_t i = 0; i < first.points.size(); ++i)
    EXPECT_TRUE(first.points[i].position.isApprox(uncached.points[i].position));
}

TEST(GeometryMapBuilderTest, RawGeometryKeepsExistingNonorthogonalMath) {
  auto raw = Poses(1);
  raw->poses[0].map_from_body.rotation(0, 0) = 1.00002;
  GeometryMapBuilder builder({}, {}, [](std::uint64_t) { return Cloud(1.0f); });
  const auto result = builder.Build({Group(0, {0}, false)}, raw);
  ASSERT_EQ(result.points.size(), 1u);
  EXPECT_EQ(result.parent_frame, PoseFrame::kOdom);
  EXPECT_NEAR(result.points[0].position.x(), 1.00002 * 1.00002 * 1.00002,
              1e-12);
}

TEST(GeometryMapBuilderTest, RetainsBuildVersionAfterCompactCacheEviction) {
  GeometryOptions geometry;
  geometry.max_resident_submaps = 3;
  GeometryMapOptions options;
  options.max_cached_points = 1;
  int reads = 0;
  GeometryMapBuilder builder(geometry, options, [&](std::uint64_t) {
    ++reads;
    return Cloud(0.05f);
  });
  const std::vector<OnlineSubmap> groups{Group(0, {0}, true),
                                         Group(1, {1}, false, {0})};
  const auto raw = builder.Build(groups, Poses(2));
  ASSERT_EQ(reads, 2);
  auto corrected = Poses(2, 1);
  for (auto& pose : corrected->poses) pose.map_from_body.translation.y() += 1.0;
  const auto reused = builder.Build(groups, corrected);
  EXPECT_EQ(reads, 2);
  EXPECT_EQ(reused.sources[0].built_pose_version, 0u);
  EXPECT_EQ(reused.sources[0].representation_revision,
            raw.sources[0].representation_revision);
}

TEST(GeometryMapBuilderTest, ToleranceBoundaryRetainsActualBuildProvenance) {
  GeometryOptions geometry;
  geometry.max_resident_submaps = 2;
  GeometryMapOptions no_cache;
  no_cache.max_cached_points = 0;
  auto reader = [](std::uint64_t id) { return Cloud(id == 0 ? 0.2f : 0.0f); };
  GeometryMapBuilder cached(geometry, {}, reader);
  GeometryMapBuilder uncached(geometry, no_cache, reader);
  const std::vector<OnlineSubmap> groups{Group(0, {0, 1}, true),
                                         Group(1, {2}, true, {1}),
                                         Group(2, {3}, false, {2})};
  auto raw = Poses(4);
  raw->poses[1].map_from_body.translation.x() = 0.2 - 2.5e-10;
  EXPECT_EQ(cached.Build(groups, raw).sources[0].source_voxel_count, 2u);
  EXPECT_EQ(uncached.Build(groups, raw).sources[0].source_voxel_count, 2u);
  auto optimized = Poses(4, 1);
  optimized->poses[1].map_from_body.translation.x() = 0.2 + 2.5e-10;
  const auto reused = cached.Build(groups, optimized);
  const auto rebuilt = uncached.Build(groups, optimized);
  EXPECT_EQ(reused.sources[0].source_voxel_count, 2u);
  EXPECT_EQ(reused.sources[0].built_pose_version, 0u);
  EXPECT_EQ(rebuilt.sources[0].source_voxel_count, 1u);
  EXPECT_EQ(rebuilt.sources[0].built_pose_version, 1u);
  EXPECT_GT(rebuilt.sources[0].representation_revision,
            reused.sources[0].representation_revision);
}

TEST(GeometryMapBuilderTest, HandlesEmptyAndBudgetBeforeCloudReads) {
  GeometryMapOptions options;
  options.max_message_bytes = 4096;
  int reads = 0;
  GeometryMapBuilder builder({}, options, [&](std::uint64_t) {
    ++reads;
    return Cloud(0);
  });
  const auto empty = builder.Build({}, Poses(0));
  EXPECT_EQ(empty.status, GeometryMapStatus::kReady);
  EXPECT_TRUE(empty.points.empty());
  const auto limited = builder.Build({Group(0, {0}, false)}, Poses(1));
  EXPECT_EQ(limited.status, GeometryMapStatus::kResourceLimited);
  EXPECT_EQ(limited.frame_count, 1u);
  EXPECT_EQ(limited.submap_count, 1u);
  EXPECT_TRUE(limited.points.empty());
  EXPECT_TRUE(limited.sources.empty());
  EXPECT_EQ(reads, 0);
}

TEST(GeometryMapBuilderTest, RejectsMissingCoverageAndLegacyPositiveTail) {
  GeometryMapBuilder builder({}, {}, [](std::uint64_t) { return Cloud(0); });
  EXPECT_THROW(builder.Build({Group(0, {0, 1}, true)}, Poses(5)),
               std::invalid_argument);
  auto legacy = Poses(1, 1);
  legacy->tail_policy = PoseTailPolicy::kLegacyLastOptimizedAnchor;
  EXPECT_THROW(builder.Build({Group(0, {0}, false)}, legacy),
               std::invalid_argument);
}

TEST(GeometryMapBuilderTest, CancellationStopsBeforeReadingAnotherBlock) {
  int reads = 0;
  bool cancelled = true;
  GeometryMapBuilder builder(
      {}, {},
      [&](std::uint64_t) {
        ++reads;
        return Cloud(0);
      },
      [&] { return cancelled; });
  EXPECT_THROW(builder.Build({Group(0, {0}, false)}, Poses(1)),
               std::runtime_error);
  EXPECT_EQ(reads, 0);
}

}  // namespace
}  // namespace livo_atlas
