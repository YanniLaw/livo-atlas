#include "livo_atlas/mapping/geometry_mapper.h"

#include <atomic>
#include <cstdlib>
#include <limits>
#include <memory>
#include <new>
#include <numeric>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

namespace {
std::atomic<long> allocations_before_failure{-1};
thread_local bool count_allocations = false;
thread_local std::size_t allocation_count = 0;

void* Allocate(std::size_t size) {
  if (count_allocations) ++allocation_count;
  if (allocations_before_failure.load(std::memory_order_relaxed) >= 0 &&
      allocations_before_failure.fetch_sub(1, std::memory_order_relaxed) == 0) {
    allocations_before_failure.store(-1, std::memory_order_relaxed);
    throw std::bad_alloc();
  }
  if (void* result = std::malloc(size ? size : 1)) return result;
  throw std::bad_alloc();
}
}  // namespace

void* operator new(std::size_t size) { return Allocate(size); }

void* operator new[](std::size_t size) { return Allocate(size); }

void operator delete(void* ptr) noexcept { std::free(ptr); }

void operator delete[](void* ptr) noexcept { std::free(ptr); }

void operator delete(void* ptr, std::size_t) noexcept { std::free(ptr); }

void operator delete[](void* ptr, std::size_t) noexcept { std::free(ptr); }

namespace livo_atlas {
namespace {

class AllocationCount {
 public:
  AllocationCount() {
    allocation_count = 0;
    count_allocations = true;
  }

  ~AllocationCount() { count_allocations = false; }

  std::size_t Stop() {
    count_allocations = false;
    return allocation_count;
  }
};

void ExpectSameGeometry(const GeometryData& a, const GeometryData& b) {
  EXPECT_EQ(a.options.enabled, b.options.enabled);
  EXPECT_EQ(a.options.voxel_size, b.options.voxel_size);
  EXPECT_EQ(a.options.min_points_per_voxel, b.options.min_points_per_voxel);
  EXPECT_EQ(a.options.max_plane_rms, b.options.max_plane_rms);
  EXPECT_EQ(a.options.max_plane_ratio, b.options.max_plane_ratio);
  EXPECT_EQ(a.options.max_voxels_per_submap, b.options.max_voxels_per_submap);
  EXPECT_EQ(a.options.max_points_per_keyframe,
            b.options.max_points_per_keyframe);
  EXPECT_EQ(a.options.max_resident_submaps, b.options.max_resident_submaps);
  ASSERT_EQ(a.input_point_count, b.input_point_count);
  ASSERT_EQ(a.sources.size(), b.sources.size());
  for (std::size_t i = 0; i < a.sources.size(); ++i) {
    EXPECT_EQ(a.sources[i].keyframe_id, b.sources[i].keyframe_id);
    EXPECT_TRUE(a.sources[i].submap_from_body.rotation ==
                b.sources[i].submap_from_body.rotation);
    EXPECT_TRUE(a.sources[i].submap_from_body.translation ==
                b.sources[i].submap_from_body.translation);
  }
  ASSERT_EQ(a.voxels.size(), b.voxels.size());
  for (std::size_t i = 0; i < a.voxels.size(); ++i) {
    const auto& x = a.voxels[i];
    const auto& y = b.voxels[i];
    EXPECT_TRUE(x.key == y.key);
    EXPECT_EQ(x.point_count, y.point_count);
    EXPECT_TRUE(x.centroid == y.centroid);
    EXPECT_TRUE(x.spatial_covariance == y.spatial_covariance);
    EXPECT_EQ(x.mean_intensity, y.mean_intensity);
    EXPECT_EQ(x.keyframe_ids, y.keyframe_ids);
    ASSERT_EQ(x.normal.has_value(), y.normal.has_value());
    if (x.normal) EXPECT_TRUE(*x.normal == *y.normal);
  }
}

TEST(GeometryFailureTest, FullBuildAvoidsRepeatedTransactionalCopies) {
  constexpr int kFrames = 16;
  PointCloud::Ptr cloud(new PointCloud);
  for (int i = 0; i < 128; ++i) {
    Point point;
    point.x = 0.25f * (i % 8);
    point.y = 0.25f * (i / 8);
    point.z = 0.1f;
    point.intensity = i;
    cloud->push_back(point);
  }
  auto poses = std::make_shared<PoseSnapshot>();
  poses->frame_count = kFrames;
  OnlineSubmap group;
  group.primary.keyframe_ids.resize(kFrames);
  std::iota(group.primary.keyframe_ids.begin(),
            group.primary.keyframe_ids.end(), 0);
  group.membership_revision = kFrames + 1;
  group.state = OnlineSubmapState::kSealed;
  for (int id = 0; id < kFrames; ++id) {
    RigidTransform pose;
    pose.translation.x() = 2 * id;
    poses->poses.push_back({static_cast<std::uint64_t>(id), pose});
  }
  GeometrySubmapBuilder reference({});
  AllocationCount reference_count;
  for (int id = 0; id < kFrames; ++id)
    reference.AddKeyframe(id, *cloud, poses->poses[id].parent_from_body());
  const auto expected = reference.Snapshot();
  const auto transactional_allocations = reference_count.Stop();

  GeometryMapper mapper({}, [&](std::uint64_t) { return cloud; });
  mapper.Update({group}, poses);
  AllocationCount build_count;
  const auto actual = mapper.GetSubmap(0);
  const auto build_allocations = build_count.Stop();
  RecordProperty("full_build_allocations", std::to_string(build_allocations));
  RecordProperty("transactional_allocations",
                 std::to_string(transactional_allocations));
  ExpectSameGeometry(*actual.data, *expected);
  // A broad relative bound avoids depending on STL-specific node sizes or
  // growth policies. No wall-clock threshold is used for this regression.
  EXPECT_LT(build_allocations, transactional_allocations / 2)
      << "complete build=" << build_allocations
      << " per-frame transactions=" << transactional_allocations;
}

TEST(GeometryFailureTest, PublicAppendRollsBackEveryAllocationFailure) {
  PointCloud cloud;
  for (int i = 0; i < 12; ++i) {
    Point point;
    point.x = 0.25f * i;
    point.y = point.z = 0.1f;
    point.intensity = i;
    cloud.push_back(point);
  }
  GeometrySubmapBuilder initial({});
  initial.AddKeyframe(0, cloud, {});
  const auto before = initial.Snapshot();
  auto extended = cloud;
  for (auto& point : extended) point.y += 1;
  auto reference = initial;
  reference.AddKeyframe(1, extended, {});
  const auto expected = reference.Snapshot();
  bool reached_success = false;
  for (long fail_after = 0; fail_after < 512; ++fail_after) {
    auto builder = initial;
    allocations_before_failure.store(fail_after);
    bool failed = false;
    try {
      builder.AddKeyframe(1, extended, {});
    } catch (const std::bad_alloc&) {
      failed = true;
    }
    allocations_before_failure.store(-1);
    if (!failed) {
      ExpectSameGeometry(*builder.Snapshot(), *expected);
      reached_success = true;
      break;
    }
    ExpectSameGeometry(*builder.Snapshot(), *before);
    builder.AddKeyframe(1, extended, {});
    ExpectSameGeometry(*builder.Snapshot(), *expected);
  }
  EXPECT_TRUE(reached_success);
}

TEST(GeometryFailureTest, FailedMultiFrameBuildCanRetryWithoutChangingCache) {
  PointCloud::Ptr cloud(new PointCloud);
  Point point;
  point.x = point.y = point.z = 0.1f;
  point.intensity = 3;
  cloud->push_back(point);
  auto bad = *cloud;
  bad.push_back(point);
  bad.back().x = std::numeric_limits<float>::quiet_NaN();
  auto poses = std::make_shared<PoseSnapshot>();
  poses->frame_count = 4;
  for (std::uint64_t id = 0; id < 4; ++id) poses->poses.push_back({id, {}});
  OnlineSubmap a, b;
  a.primary = {0, 0, {0, 1}};
  b.primary = {1, 2, {2, 3}};
  a.membership_revision = b.membership_revision = 3;
  a.state = b.state = OnlineSubmapState::kSealed;
  PointCloud::Ptr bad_cloud(new PointCloud(bad));
  bool fail = true;
  GeometryMapper mapper({}, [&](std::uint64_t id) {
    return id == 3 && fail ? bad_cloud : cloud;
  });
  mapper.Update({a, b}, poses);
  const auto old = mapper.GetSubmap(0);
  EXPECT_THROW(mapper.GetSubmap(1), std::invalid_argument);
  EXPECT_EQ(mapper.GetStats().full_builds, 1u);
  EXPECT_EQ(mapper.GetSubmap(0).data, old.data);
  EXPECT_EQ(old.data->input_point_count, 2u);
  fail = false;
  const auto retried = mapper.GetSubmap(1);
  EXPECT_EQ(retried.representation_revision, 1u);
  EXPECT_EQ(retried.data->input_point_count, 2u);
  GeometrySubmapBuilder reference({});
  reference.AddKeyframe(2, *cloud, {});
  reference.AddKeyframe(3, *cloud, {});
  ExpectSameGeometry(*retried.data, *reference.Snapshot());
}

TEST(GeometryFailureTest, AllocationFailureCannotPublishUnversionedGeometry) {
  PointCloud::Ptr cloud(new PointCloud);
  Point point;
  point.x = point.y = point.z = 0.1f;
  cloud->push_back(point);
  point.x = 0.35f;
  cloud->push_back(point);
  auto poses = std::make_shared<PoseSnapshot>();
  poses->frame_count = 3;
  for (std::uint64_t id = 0; id < 3; ++id) poses->poses.push_back({id, {}});
  OnlineSubmap group;
  group.primary = {0, 0, {0, 1, 2}};
  group.membership_revision = 3;
  bool reached_success = false;
  int failures = 0;
  for (long fail_after = 0; fail_after < 512; ++fail_after) {
    GeometryMapper mapper({}, [&](std::uint64_t) { return cloud; });
    mapper.Update({group}, poses);
    bool failed = false;
    allocations_before_failure.store(fail_after);
    try {
      (void)mapper.GetSubmap(0);
    } catch (const std::bad_alloc&) {
      failed = true;
    }
    allocations_before_failure.store(-1);
    if (!failed) {
      reached_success = true;
      break;
    }
    ++failures;
    const auto retried = mapper.GetSubmap(0);
    EXPECT_EQ(retried.representation_revision, 1u)
        << "allocation " << fail_after;
    EXPECT_EQ(retried.data->input_point_count, 6u);
    EXPECT_EQ(retried.data->sources.size(), 3u);
  }
  EXPECT_GT(failures, 0);
  EXPECT_TRUE(reached_success);
}

}  // namespace
}  // namespace livo_atlas
