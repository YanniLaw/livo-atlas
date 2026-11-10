#include <gtest/gtest.h>

#include <atomic>
#include <cstdlib>
#include <new>

#include "livo_atlas/mapping/geometry_mapper.h"

namespace {
std::atomic<long> allocations_before_failure{-1};

void* Allocate(std::size_t size) {
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

TEST(GeometryFailureTest, AllocationFailureCannotPublishUnversionedGeometry) {
  PointCloud::Ptr cloud(new PointCloud);
  Point point;
  point.x = point.y = point.z = 0.1f;
  cloud->push_back(point);
  auto poses = std::make_shared<PoseSnapshot>();
  poses->frame_count = 1;
  poses->poses.push_back({0, {}});
  OnlineSubmap group;
  group.primary = {0, 0, {0}};
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
    EXPECT_EQ(retried.data->input_point_count, 1u);
  }
  EXPECT_GT(failures, 0);
  EXPECT_TRUE(reached_success);
}

}  // namespace
}  // namespace livo_atlas
