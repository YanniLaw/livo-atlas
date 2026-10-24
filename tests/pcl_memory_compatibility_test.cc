#include <gtest/gtest.h>
#include <pcl/filters/voxel_grid.h>

#include "livo_atlas/internal/legacy/types.h"

namespace livo_atlas {
namespace {

// PCL allocates the filtered point vector in its precompiled library. The
// application must use a compatible Eigen allocator when releasing that vector.
TEST(PclMemoryCompatibilityTest, ReleasesCloudAllocatedBySystemVoxelFilter) {
  PointCloudXYZI::Ptr input(new PointCloudXYZI);
  PointCloudXYZI::Ptr output(new PointCloudXYZI);
  for (int i = 0; i < 10000; ++i) {
    PointType point{};
    point.x = (i % 100) * 0.05f;
    point.y = (i / 100) * 0.05f;
    point.z = 1.0f;
    point.normal_z = 1.0f;
    input->push_back(point);
  }

  pcl::VoxelGrid<PointType> filter;
  filter.setInputCloud(input);
  filter.setLeafSize(0.1f, 0.1f, 0.1f);
  filter.filter(*output);
  ASSERT_EQ(output->size(), 2500u);

  // This release used to segfault with the package's forced -march=native.
  output.reset();
}

}  // namespace
}  // namespace livo_atlas
