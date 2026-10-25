#include <gtest/gtest.h>
#include <pcl/filters/voxel_grid.h>

#include "livo_atlas/common/math_types.h"
#include "livo_atlas/sensor/point_types.h"

namespace livo_atlas {
namespace {

// PCL allocates the filtered point vector in its precompiled library. The
// application must use a compatible Eigen allocator when releasing that vector.
TEST(PclMemoryCompatibilityTest, ReleasesCloudAllocatedBySystemVoxelFilter) {
  PointCloud::Ptr input(new PointCloud);
  PointCloud::Ptr output(new PointCloud);
  for (int i = 0; i < 10000; ++i) {
    Point point{};
    point.x = (i % 100) * 0.05f;
    point.y = (i / 100) * 0.05f;
    point.z = 1.0f;
    point.normal_z = 1.0f;
    input->push_back(point);
  }

  pcl::VoxelGrid<Point> filter;
  filter.setInputCloud(input);
  filter.setLeafSize(0.1f, 0.1f, 0.1f);
  filter.filter(*output);
  ASSERT_EQ(output->size(), 2500u);

  // This release used to segfault with the package's forced -march=native.
  output.reset();
}

}  // namespace
}  // namespace livo_atlas
