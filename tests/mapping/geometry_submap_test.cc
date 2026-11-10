#include "livo_atlas/mapping/geometry_submap.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

namespace livo_atlas {
namespace {

PointCloud Plane() {
  PointCloud cloud;
  for (float x : {0.125f, 0.375f}) {
    for (float y : {0.125f, 0.375f}) {
      Point point;
      point.x = x;
      point.y = y;
      point.z = 0.25f;
      point.intensity = 8;
      cloud.push_back(point);
    }
  }
  return cloud;
}

TEST(GeometrySubmapTest, AccumulatesKnownPlaneAndSources) {
  GeometryOptions options;
  options.voxel_size = 0.5;
  GeometrySubmapBuilder builder(options);
  builder.AddKeyframe(0, Plane(), {});
  builder.AddKeyframe(1, Plane(), {});
  const auto data = builder.Snapshot();
  ASSERT_EQ(data->voxels.size(), 1u);
  const auto& voxel = data->voxels.front();
  EXPECT_EQ(voxel.key.x, 0);
  EXPECT_EQ(voxel.point_count, 8u);
  EXPECT_NEAR((voxel.centroid - Eigen::Vector3d(0.25, 0.25, 0.25)).norm(), 0,
              1e-12);
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  covariance(0, 0) = covariance(1, 1) = 0.015625;
  EXPECT_NEAR((voxel.spatial_covariance - covariance).norm(), 0, 1e-12);
  EXPECT_DOUBLE_EQ(voxel.mean_intensity, 8);
  ASSERT_TRUE(voxel.normal);
  EXPECT_NEAR((*voxel.normal - Eigen::Vector3d::UnitZ()).norm(), 0, 1e-12);
  EXPECT_EQ(voxel.keyframe_ids, (std::vector<std::uint64_t>{0, 1}));
}

TEST(GeometrySubmapTest, UsesFloorAndChecksIndexRange) {
  GeometryOptions options;
  options.voxel_size = 0.5;
  GeometrySubmapBuilder builder(options);
  PointCloud cloud;
  for (float value :
       {-0.5f, std::nextafter(-0.5f, -1.0f), std::nextafter(-0.5f, 0.0f), -0.0f,
        0.0f, std::nextafter(0.0f, -1.0f), 0.5f, std::nextafter(0.5f, 0.0f),
        std::nextafter(0.5f, 1.0f)}) {
    Point point;
    point.x = value;
    cloud.push_back(point);
  }
  builder.AddKeyframe(0, cloud, {});
  const auto data = builder.Snapshot();
  ASSERT_EQ(data->voxels.size(), 4u);
  EXPECT_EQ(data->voxels[0].key.x, -2);
  EXPECT_EQ(data->voxels[1].key.x, -1);
  EXPECT_EQ(data->voxels[2].key.x, 0);
  EXPECT_EQ(data->voxels[3].key.x, 1);
  options.voxel_size = 1e-20;
  GeometrySubmapBuilder tiny(options);
  cloud[0].x = std::numeric_limits<float>::max();
  EXPECT_THROW(tiny.AddKeyframe(0, cloud, {}), std::invalid_argument);
}

TEST(GeometrySubmapTest, RejectsPartialInsertionAndLimitsSources) {
  GeometryOptions options;
  options.voxel_size = 0.5;
  GeometrySubmapBuilder builder(options);
  builder.AddKeyframe(0, Plane(), {});
  auto bad = Plane();
  bad[2].x = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(builder.AddKeyframe(1, bad, {}), std::invalid_argument);
  EXPECT_THROW(builder.AddKeyframe(0, Plane(), {}), std::invalid_argument);
  EXPECT_EQ(builder.Snapshot()->input_point_count, 4u);
  for (std::uint64_t id = 1; id < 64; ++id)
    builder.AddKeyframe(id, Plane(), {});
  EXPECT_THROW(builder.AddKeyframe(64, Plane(), {}), std::invalid_argument);
  EXPECT_EQ(builder.Snapshot()->input_point_count, 256u);
}

TEST(GeometrySubmapTest, DegenerateVoxelsAndSnapshotsRemainOwned) {
  GeometryOptions options;
  options.voxel_size = 0.5;
  GeometrySubmapBuilder builder(options);
  PointCloud cloud(6, 1);
  for (auto& point : cloud) point.x = point.y = point.z = 0.25f;
  builder.AddKeyframe(0, cloud, {});
  const auto old = builder.Snapshot();
  ASSERT_EQ(old->voxels.size(), 1u);
  EXPECT_FALSE(old->voxels[0].normal);
  builder.AddKeyframe(1, Plane(), {});
  EXPECT_EQ(old->input_point_count, 6u);
  EXPECT_EQ(old->voxels[0].point_count, 6u);
  EXPECT_EQ(builder.Snapshot()->input_point_count, 10u);
}

TEST(GeometrySubmapTest, RejectsLimitsWithoutPartiallyChangingStatistics) {
  GeometryOptions options;
  options.voxel_size = 0.5;
  options.max_voxels_per_submap = 1;
  GeometrySubmapBuilder builder(options);
  builder.AddKeyframe(0, Plane(), {});
  auto cloud = Plane();
  cloud.back().x = 1;
  EXPECT_THROW(builder.AddKeyframe(1, cloud, {}), std::invalid_argument);
  EXPECT_THROW(builder.AddKeyframe(1, PointCloud{}, {}), std::invalid_argument);
  EXPECT_EQ(builder.Snapshot()->input_point_count, 4u);
  EXPECT_NO_THROW(builder.AddKeyframe(1, Plane(), {}));
  options.max_points_per_keyframe = 3;
  GeometrySubmapBuilder limited(options);
  EXPECT_THROW(limited.AddKeyframe(0, Plane(), {}), std::invalid_argument);
  EXPECT_TRUE(limited.Snapshot()->voxels.empty());
}

TEST(GeometrySubmapTest, RejectsLinesAndSeparatedPlanesAsSurfaces) {
  GeometryOptions options;
  options.voxel_size = 0.5;
  PointCloud line;
  for (int i = 0; i < 8; ++i) {
    Point point;
    point.x = 0.05f * i;
    point.y = point.z = 0.25f;
    line.push_back(point);
  }
  GeometrySubmapBuilder line_builder(options);
  line_builder.AddKeyframe(0, line, {});
  EXPECT_FALSE(line_builder.Snapshot()->voxels[0].normal);
  PointCloud two_planes;
  for (float z : {0.125f, 0.375f}) {
    auto plane = Plane();
    for (auto& point : plane) point.z = z;
    two_planes += plane;
  }
  GeometrySubmapBuilder plane_builder(options);
  plane_builder.AddKeyframe(0, two_planes, {});
  EXPECT_FALSE(plane_builder.Snapshot()->voxels[0].normal);
}

TEST(GeometrySubmapTest, AppliesPlaneRmsAndRatioThresholdsIndependently) {
  const auto surface = [](float half_thickness) {
    PointCloud points;
    for (float z : {0.25f - half_thickness, 0.25f + half_thickness}) {
      auto plane = Plane();
      for (auto& point : plane) point.z = z;
      points += plane;
    }
    return points;
  };
  GeometryOptions options;
  options.voxel_size = 0.5;
  GeometrySubmapBuilder below_rms(options), above_rms(options);
  below_rms.AddKeyframe(0, surface(0.015625f), {});
  above_rms.AddKeyframe(0, surface(0.03125f), {});
  EXPECT_TRUE(below_rms.Snapshot()->voxels[0].normal);
  EXPECT_FALSE(above_rms.Snapshot()->voxels[0].normal);
  options.max_plane_rms = 0.1;
  options.max_plane_ratio = 0.01;
  GeometrySubmapBuilder below_ratio(options), above_ratio(options);
  below_ratio.AddKeyframe(0, surface(0.0078125f), {});
  above_ratio.AddKeyframe(0, surface(0.03125f), {});
  EXPECT_TRUE(below_ratio.Snapshot()->voxels[0].normal);
  EXPECT_FALSE(above_ratio.Snapshot()->voxels[0].normal);
}

TEST(GeometrySubmapTest, RejectsInvalidOptionsEvenWhenDisabled) {
  GeometryOptions options;
  options.enabled = false;
  options.voxel_size = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(GeometrySubmapBuilder{options}, std::invalid_argument);
  options.voxel_size = std::numeric_limits<double>::infinity();
  EXPECT_THROW(GeometrySubmapBuilder{options}, std::invalid_argument);
  options.voxel_size = 0.2;
  options.min_points_per_voxel = 2;
  EXPECT_THROW(GeometrySubmapBuilder{options}, std::invalid_argument);
  options.min_points_per_voxel = 6;
  options.max_plane_rms = 0;
  EXPECT_THROW(GeometrySubmapBuilder{options}, std::invalid_argument);
  options.max_plane_rms = 0.02;
  options.max_plane_ratio = 1;
  EXPECT_THROW(GeometrySubmapBuilder{options}, std::invalid_argument);
  options.max_plane_ratio = 0.1;
  options.max_resident_submaps = 1;
  EXPECT_THROW(GeometrySubmapBuilder{options}, std::invalid_argument);
}

}  // namespace
}  // namespace livo_atlas
