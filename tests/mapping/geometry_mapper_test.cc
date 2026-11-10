#include "livo_atlas/mapping/geometry_mapper.h"

#include <memory>
#include <stdexcept>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

namespace livo_atlas {
namespace {

PointCloud::ConstPtr Cloud(float x) {
  PointCloud::Ptr cloud(new PointCloud);
  for (int i = 0; i < 8; ++i) {
    Point p;
    p.x = x + 0.01f * (i % 2);
    p.y = 0.01f * (i / 2);
    p.z = 0.1f;
    p.intensity = 4;
    cloud->push_back(p);
  }
  return cloud;
}

std::shared_ptr<const PoseSnapshot> Poses(int count, int version = 0) {
  auto poses = std::make_shared<PoseSnapshot>();
  poses->version = version;
  poses->base_version = version ? version - 1 : 0;
  poses->frame_count = count;
  poses->optimized_frame_count = version ? count : 0;
  poses->parent_frame = version ? PoseFrame::kMap : PoseFrame::kOdom;
  for (int i = 0; i < count; ++i) {
    RigidTransform pose;
    pose.translation.x() = i;
    poses->poses.push_back({static_cast<std::uint64_t>(i), pose});
  }
  return poses;
}

OnlineSubmap Group(std::uint64_t id, std::vector<std::uint64_t> ids,
                   bool sealed, std::vector<std::uint64_t> support = {}) {
  OnlineSubmap group;
  group.primary.id = id;
  group.primary.anchor_keyframe_id = ids.front();
  group.primary.keyframe_ids = std::move(ids);
  group.support_keyframe_ids = std::move(support);
  group.state =
      sealed ? OnlineSubmapState::kSealed : OnlineSubmapState::kActive;
  group.membership_revision = group.primary.keyframe_ids.size() + sealed;
  return group;
}

TEST(GeometryMapperTest, IncrementalMatchesRebuildAndSupportDoesNotFuse) {
  std::vector<PointCloud::ConstPtr> clouds{Cloud(0), Cloud(0), Cloud(0)};
  int reads = 0;
  auto reader = [&](std::uint64_t id) {
    ++reads;
    return clouds.at(id);
  };
  GeometryMapper mapper({}, reader);
  Keyframe a;
  a.id = 0;
  a.cloud = clouds[0];
  mapper.Update({Group(0, {0}, false)}, Poses(1), &a);
  Keyframe b;
  b.id = 1;
  b.cloud = clouds[1];
  mapper.Update({Group(0, {0, 1}, false), Group(0, {0, 1}, true)}, Poses(2),
                &b);
  const auto first = mapper.GetSubmap(0);
  EXPECT_EQ(first.data->input_point_count, 16u);
  mapper.Update({Group(0, {0, 1}, true)}, Poses(2), &b);
  EXPECT_EQ(mapper.GetSubmap(0).representation_revision,
            first.representation_revision);
  Keyframe c;
  c.id = 2;
  c.cloud = clouds[2];
  mapper.Update({Group(1, {2}, false, {0, 1})}, Poses(3), &c);
  const auto second = mapper.GetSubmap(1);
  EXPECT_EQ(second.data->input_point_count, 8u);
  EXPECT_EQ(second.membership.support_keyframe_ids,
            (std::vector<std::uint64_t>{0, 1}));
  GeometryMapper rebuilt({}, reader);
  rebuilt.Update({Group(0, {0, 1}, true), Group(1, {2}, false, {0, 1})},
                 Poses(3));
  const auto fresh = rebuilt.GetSubmap(0);
  EXPECT_EQ(fresh.data->input_point_count, first.data->input_point_count);
  ASSERT_EQ(fresh.data->voxels.size(), first.data->voxels.size());
  for (std::size_t i = 0; i < fresh.data->voxels.size(); ++i) {
    EXPECT_TRUE(fresh.data->voxels[i].key == first.data->voxels[i].key);
    EXPECT_EQ(fresh.data->voxels[i].keyframe_ids,
              first.data->voxels[i].keyframe_ids);
    EXPECT_NEAR(
        (fresh.data->voxels[i].centroid - first.data->voxels[i].centroid)
            .norm(),
        0, 1e-12);
  }
}

TEST(GeometryMapperTest,
     RigidCorrectionReusesDataRelativeCorrectionRebuildsOwner) {
  int reads = 0;
  GeometryMapper mapper({}, [&](std::uint64_t) {
    ++reads;
    return Cloud(0);
  });
  mapper.Update({Group(0, {0, 1}, true), Group(1, {2}, false, {0, 1})},
                Poses(3));
  const auto a = mapper.GetSubmap(0);
  const auto b = mapper.GetSubmap(1);
  auto corrected = std::make_shared<PoseSnapshot>(*Poses(3, 1));
  const auto r = Eigen::AngleAxisd(1.5707963267948966, Eigen::Vector3d::UnitZ())
                     .toRotationMatrix();
  for (auto& pose : corrected->poses) {
    pose.map_from_body.translation =
        r * pose.map_from_body.translation + Eigen::Vector3d(3, -2, 1);
    pose.map_from_body.rotation = r * pose.map_from_body.rotation;
  }
  const auto before_reads = reads;
  mapper.Update({}, corrected);
  const auto rigid = mapper.GetSubmap(0);
  EXPECT_EQ(rigid.data.get(), a.data.get());
  EXPECT_EQ(rigid.representation_revision, a.representation_revision);
  EXPECT_EQ(reads, before_reads);
  auto relative = std::make_shared<PoseSnapshot>(*corrected);
  relative->version = 2;
  relative->base_version = 1;
  relative->poses[1].map_from_body.translation.x() += 0.3;
  mapper.Update({}, relative);
  const auto affected = mapper.GetSubmap(0);
  const auto unaffected = mapper.GetSubmap(1);
  EXPECT_GT(affected.representation_revision, rigid.representation_revision);
  EXPECT_NE(affected.data.get(), rigid.data.get());
  EXPECT_EQ(unaffected.data.get(), b.data.get());
  EXPECT_EQ(a.data->input_point_count, 16u);
}

TEST(GeometryMapperTest,
     InvalidInputsDoNotAdvanceAndOldSnapshotsSurviveEviction) {
  GeometryOptions options;
  options.max_resident_submaps = 2;
  GeometryMapper mapper(options, [](std::uint64_t) { return Cloud(0); });
  mapper.Update(
      {Group(0, {0}, true), Group(1, {1}, true), Group(2, {2}, false)},
      Poses(3));
  const auto old = mapper.GetSubmap(0);
  mapper.GetSubmap(1);
  mapper.GetSubmap(2);
  EXPECT_LE(mapper.GetStats().resident_submaps, 2u);
  const auto rebuilt = mapper.GetSubmap(0);
  EXPECT_GT(rebuilt.representation_revision, old.representation_revision);
  EXPECT_EQ(old.data->input_point_count, 8u);
  auto altered = std::make_shared<PoseSnapshot>(*Poses(3));
  altered->poses[0].map_from_body.translation.x() = 7;
  EXPECT_THROW(mapper.Update({}, altered), std::invalid_argument);
  EXPECT_EQ(mapper.GetSubmap(0).parent_from_submap.translation.x(), 0);
  auto invalid = Group(0, {0, 1}, true);
  EXPECT_THROW(mapper.Update({invalid}, Poses(3)), std::invalid_argument);
}

TEST(GeometryMapperTest, SmallCorrectionsAreComparedWithBuiltSources) {
  GeometryMapper mapper({}, [](std::uint64_t) { return Cloud(0); });
  mapper.Update({Group(0, {0, 1}, true)}, Poses(2));
  const auto original = mapper.GetSubmap(0);
  auto first = std::make_shared<PoseSnapshot>(*Poses(2, 1));
  first->poses[1].map_from_body.translation.x() += 0.75e-9;
  mapper.Update({}, first);
  EXPECT_EQ(mapper.GetSubmap(0).data.get(), original.data.get());
  auto second = std::make_shared<PoseSnapshot>(*first);
  second->version = 2;
  second->base_version = 1;
  second->poses[1].map_from_body.translation.x() += 0.75e-9;
  mapper.Update({}, second);
  const auto rebuilt = mapper.GetSubmap(0);
  EXPECT_GT(rebuilt.representation_revision, original.representation_revision);
  EXPECT_NE(rebuilt.data.get(), original.data.get());
}

TEST(GeometryMapperTest, RebuildFailureDoesNotPublishAndCanBeRetried) {
  bool fail = true;
  GeometryMapper mapper({}, [&](std::uint64_t id) -> PointCloud::ConstPtr {
    if (fail && id == 1) throw std::runtime_error("read failed");
    return Cloud(0);
  });
  mapper.Update({Group(0, {0, 1}, true)}, Poses(2));
  EXPECT_THROW(mapper.GetSubmap(0), std::runtime_error);
  EXPECT_EQ(mapper.GetStats().full_builds, 0u);
  EXPECT_EQ(mapper.GetStats().resident_submaps, 0u);
  fail = false;
  const auto recovered = mapper.GetSubmap(0);
  EXPECT_EQ(recovered.representation_revision, 1u);
  EXPECT_EQ(recovered.data->input_point_count, 16u);
}

TEST(GeometryMapperTest, RejectsExcessMembersAndChangingSupport) {
  GeometryMapper mapper({}, [](std::uint64_t) { return Cloud(0); });
  std::vector<std::uint64_t> ids;
  for (std::uint64_t i = 0; i < 65; ++i) ids.push_back(i);
  EXPECT_THROW(mapper.Update({Group(0, ids, true)}, Poses(65)),
               std::invalid_argument);
  mapper.Update({Group(0, {0, 1}, true), Group(1, {2}, false, {0, 1})},
                Poses(3));
  auto invalid = Group(1, {2, 3}, false, {1});
  EXPECT_THROW(mapper.Update({invalid}, Poses(4)), std::invalid_argument);
  EXPECT_EQ(mapper.GetSubmap(1).membership.support_keyframe_ids,
            (std::vector<std::uint64_t>{0, 1}));
}

TEST(GeometryMapperTest, GrowingPoseFrontierDoesNotRebuildOldBlock) {
  GeometryMapper mapper({}, [](std::uint64_t) { return Cloud(0); });
  mapper.Update({Group(0, {0, 1}, true)}, Poses(2));
  mapper.Update({}, Poses(2, 1));
  const auto before = mapper.GetSubmap(0);
  auto grown = std::make_shared<PoseSnapshot>(*Poses(3, 1));
  grown->optimized_frame_count = 2;
  mapper.Update({Group(1, {2}, false, {1})}, grown);
  const auto after = mapper.GetSubmap(0);
  EXPECT_EQ(after.data.get(), before.data.get());
  EXPECT_EQ(after.representation_revision, before.representation_revision);
  EXPECT_EQ(after.frame_count, 3u);
  auto next = std::make_shared<PoseSnapshot>(*Poses(3, 3));
  mapper.Update({}, next);
  EXPECT_EQ(mapper.GetSubmap(0).pose_version, 3u);
}

TEST(GeometryMapperTest, RejectsInvalidIntermediateRevisionWithoutMutation) {
  GeometryMapper mapper({}, [](std::uint64_t) { return Cloud(0); });
  OnlineSubmap invalid;
  invalid.primary.id = 0;
  invalid.primary.anchor_keyframe_id = 0;
  invalid.membership_revision = 0;
  EXPECT_THROW(mapper.Update({invalid, Group(0, {0}, false)}, Poses(1)),
               std::invalid_argument);
  EXPECT_THROW(mapper.GetSubmap(0), std::out_of_range);
  EXPECT_NO_THROW(mapper.Update({Group(0, {0}, false)}, Poses(1)));
  EXPECT_EQ(mapper.GetSubmap(0).representation_revision, 1u);
}

TEST(GeometryMapperTest, ConsecutiveNoncommutingRigidCorrectionsReuseData) {
  GeometryMapper mapper({}, [](std::uint64_t) { return Cloud(0); });
  mapper.Update({Group(0, {0, 1}, true)}, Poses(2));
  const auto initial = mapper.GetSubmap(0);
  auto corrected = std::make_shared<PoseSnapshot>(*Poses(2));
  for (int step = 0; step < 2; ++step) {
    const Eigen::Vector3d axis =
        step == 0 ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(1.5707963267948966, axis).toRotationMatrix();
    for (auto& pose : corrected->poses) {
      pose.map_from_body.translation =
          rotation * pose.map_from_body.translation + Eigen::Vector3d(3, -2, 1);
      pose.map_from_body.rotation = rotation * pose.map_from_body.rotation;
    }
    corrected->version = step + 1;
    corrected->base_version = step;
    corrected->optimized_frame_count = 2;
    corrected->parent_frame = PoseFrame::kMap;
    mapper.Update({}, std::make_shared<const PoseSnapshot>(*corrected));
    const auto result = mapper.GetSubmap(0);
    EXPECT_EQ(result.data.get(), initial.data.get());
    EXPECT_EQ(result.representation_revision, initial.representation_revision);
    EXPECT_NEAR((result.parent_from_submap.translation -
                 corrected->poses[0].map_from_body.translation)
                    .norm(),
                0, 1e-12);
  }
}

TEST(GeometryMapperTest, AnchorOnlyCorrectionRebuildsAgainstNewLocalFrame) {
  const auto reader = [](std::uint64_t) { return Cloud(0); };
  GeometryMapper mapper({}, reader);
  const auto groups = std::vector<OnlineSubmap>{Group(0, {0, 1}, true)};
  mapper.Update(groups, Poses(2));
  const auto original = mapper.GetSubmap(0);
  auto corrected = std::make_shared<PoseSnapshot>(*Poses(2, 1));
  corrected->poses[0].map_from_body.translation = Eigen::Vector3d(2, -1, 0);
  corrected->poses[0].map_from_body.rotation =
      Eigen::AngleAxisd(1.5707963267948966, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  mapper.Update({}, corrected);
  const auto rebuilt = mapper.GetSubmap(0);
  EXPECT_GT(rebuilt.representation_revision, original.representation_revision);
  EXPECT_NE(rebuilt.data.get(), original.data.get());
  GeometryMapper fresh({}, reader);
  fresh.Update(groups, corrected);
  const auto expected = fresh.GetSubmap(0);
  ASSERT_EQ(rebuilt.data->voxels.size(), expected.data->voxels.size());
  for (std::size_t i = 0; i < rebuilt.data->voxels.size(); ++i) {
    EXPECT_TRUE(rebuilt.data->voxels[i].key == expected.data->voxels[i].key);
    EXPECT_NEAR(
        (rebuilt.data->voxels[i].centroid - expected.data->voxels[i].centroid)
            .norm(),
        0, 1e-12);
  }
}

TEST(GeometryMapperTest, EvictedBlockUsesNewestOfMultiplePoseVersions) {
  GeometryOptions options;
  options.max_resident_submaps = 2;
  const auto reader = [](std::uint64_t) { return Cloud(0); };
  GeometryMapper mapper(options, reader);
  const auto groups = std::vector<OnlineSubmap>{
      Group(0, {0, 1}, true), Group(1, {2}, true), Group(2, {3}, false)};
  mapper.Update(groups, Poses(4));
  const auto old = mapper.GetSubmap(0);
  mapper.GetSubmap(1);
  mapper.GetSubmap(2);
  auto version1 = std::make_shared<PoseSnapshot>(*Poses(4, 1));
  for (auto& pose : version1->poses) pose.map_from_body.translation.x() += 4;
  mapper.Update({}, version1);
  auto version2 = std::make_shared<PoseSnapshot>(*version1);
  version2->version = 2;
  version2->base_version = 1;
  version2->poses[1].map_from_body.translation.y() += 0.3;
  mapper.Update({}, version2);
  const auto reloaded = mapper.GetSubmap(0);
  EXPECT_EQ(reloaded.pose_version, 2u);
  EXPECT_GT(reloaded.representation_revision, old.representation_revision);
  GeometryMapper fresh(options, reader);
  fresh.Update(groups, version2);
  const auto expected = fresh.GetSubmap(0);
  ASSERT_EQ(reloaded.data->voxels.size(), expected.data->voxels.size());
  for (std::size_t i = 0; i < reloaded.data->voxels.size(); ++i) {
    EXPECT_TRUE(reloaded.data->voxels[i].key == expected.data->voxels[i].key);
    EXPECT_NEAR(
        (reloaded.data->voxels[i].centroid - expected.data->voxels[i].centroid)
            .norm(),
        0, 1e-12);
  }
}

TEST(GeometryMapperTest, RejectsChangingTailPolicyWithinVersion) {
  GeometryMapper mapper({}, [](std::uint64_t) { return Cloud(0); });
  const auto poses = Poses(2, 1);
  mapper.Update({Group(0, {0, 1}, true)}, poses);
  const auto original = mapper.GetSubmap(0);
  auto changed = std::make_shared<PoseSnapshot>(*poses);
  changed->tail_policy = PoseTailPolicy::kProjectedLastOptimizedAnchor;
  EXPECT_THROW(mapper.Update({}, changed), std::invalid_argument);
  const auto after = mapper.GetSubmap(0);
  EXPECT_EQ(after.pose_version, original.pose_version);
  EXPECT_EQ(after.data, original.data);
}

}  // namespace
}  // namespace livo_atlas
