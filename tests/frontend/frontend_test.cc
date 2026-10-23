#include "livo_atlas/frontend/frontend.h"

#include <gtest/gtest.h>

namespace livo_atlas {
namespace {

FrontendOptions MakeOptions() {
  FrontendOptions options;
  options.calibration.camera.model = "Pinhole";
  options.calibration.camera.width = 512;
  options.calibration.camera.height = 512;
  options.calibration.camera.fx = 200;
  options.calibration.camera.fy = 200;
  options.calibration.camera.cx = 256;
  options.calibration.camera.cy = 256;
  return options;
}

SynchronizedMeasurements MakeScan(
    double time = 100.0, UpdateKind kind = UpdateKind::kLidarOnly) {
  SynchronizedMeasurements input;
  input.lio_vio_flg = kind;
  input.lidar_frame_beg_time = time;
  input.lidar_frame_end_time = time + 0.125;
  for (int i = 0; i < 2; ++i) {
    Point point{};
    point.x = 2 + i;
    point.intensity = 10;
    point.curvature = 125 * i;
    input.lidar->push_back(point);
  }
  input.pcl_proc_cur = input.lidar;
  input.measures.emplace_back();
  input.measures.back().lio_time = time + 0.125;
  if (kind == UpdateKind::kLidarInertial) {
    input.last_lio_update_time = time;
    for (double timestamp : {time + 0.0625, time + 0.125}) {
      input.measures.back().imu.push_back(
          {timestamp, Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, -9.81)});
    }
  }
  return input;
}

TEST(FrontendTest, WaitingAndEmptyGroupsDoNotStartFirstFrame) {
  Frontend frontend(MakeOptions());
  SynchronizedMeasurements input;
  EXPECT_FALSE(frontend.Process(&input).updated);
  input.lio_vio_flg = UpdateKind::kLidarOnly;
  EXPECT_FALSE(frontend.Process(&input).updated);
  input = MakeScan();
  const auto result = frontend.Process(&input);
  ASSERT_TRUE(result.updated);
  EXPECT_TRUE(result.first_frame);
}

TEST(FrontendTest, LidarOnlyUpdatesStateAndWorldCloud) {
  Frontend frontend(MakeOptions());
  auto input = MakeScan();
  const auto result = frontend.Process(&input);
  ASSERT_TRUE(result.updated);
  EXPECT_EQ(result.kind, UpdateKind::kLidarOnly);
  EXPECT_DOUBLE_EQ(result.timestamp, 100.125);
  EXPECT_DOUBLE_EQ(result.relative_time, 101.125);
  EXPECT_NEAR(result.prediction.cov(3, 7), 0.001, 1e-15);
  EXPECT_TRUE(result.estimate.cov.isApprox(frontend.state().cov, 0));
  ASSERT_TRUE(result.world_cloud);
  ASSERT_EQ(result.world_cloud->size(), 2);
  EXPECT_FLOAT_EQ(result.world_cloud->front().x, 2);
  EXPECT_FLOAT_EQ(result.world_cloud->back().x, 3);
}

TEST(FrontendTest, ContinuousScansKeepSharedStateAndMap) {
  Frontend frontend(MakeOptions());
  auto input = MakeScan();
  ASSERT_TRUE(frontend.Process(&input).updated);
  input = MakeScan(100.25);
  const auto result = frontend.Process(&input);
  ASSERT_TRUE(result.updated);
  EXPECT_FALSE(result.first_frame);
  EXPECT_DOUBLE_EQ(result.timestamp, 100.375);
  EXPECT_DOUBLE_EQ(result.relative_time, 101.375);
  EXPECT_TRUE(result.estimate.cov.isApprox(frontend.state().cov, 0));
  ASSERT_EQ(result.world_cloud->size(), 2);
  EXPECT_FLOAT_EQ(result.world_cloud->front().x, 2);
}

TEST(FrontendTest, ResultStateSnapshotSurvivesNextUpdate) {
  Frontend frontend(MakeOptions());
  auto input = MakeScan();
  const auto first = frontend.Process(&input);
  ASSERT_TRUE(first.updated);
  input = MakeScan(100.25);
  ASSERT_TRUE(frontend.Process(&input).updated);
  EXPECT_NEAR(first.prediction.cov(3, 7), 0.001, 1e-15);
  EXPECT_NEAR(first.estimate.cov(3, 7), 0.001, 1e-15);
  EXPECT_GT(frontend.state().cov(3, 7), first.estimate.cov(3, 7));
}

TEST(FrontendTest, InstancesDoNotShareState) {
  Frontend first(MakeOptions());
  Frontend second(MakeOptions());
  auto input = MakeScan();
  ASSERT_TRUE(first.Process(&input).updated);
  EXPECT_DOUBLE_EQ(second.state().cov(3, 7), 0);
  EXPECT_NE(&first.state(), &second.state());
  input = MakeScan(200.0);
  const auto result = second.Process(&input);
  ASSERT_TRUE(result.updated);
  EXPECT_TRUE(result.first_frame);
  EXPECT_DOUBLE_EQ(result.timestamp, 200.125);
}

TEST(FrontendTest, InitialImuReturnsPredictionWithoutMapping) {
  auto options = MakeOptions();
  options.imu.enabled = true;
  Frontend frontend(options);
  auto input = MakeScan(100, UpdateKind::kLidarInertial);
  const auto result = frontend.Process(&input);
  EXPECT_FALSE(result.updated);
  EXPECT_TRUE(result.first_frame);
  EXPECT_TRUE(result.prediction.gravity.isApprox(Eigen::Vector3d(0, 0, 9.81)));
  EXPECT_FALSE(result.updated);
  EXPECT_FALSE(result.world_cloud);
  EXPECT_TRUE(frontend.imu_needs_initialization());
  EXPECT_DOUBLE_EQ(frontend.imu_mean_acceleration_norm(), 0);
}

TEST(FrontendTest, EmptyVisualInputDoesNotUpdate) {
  auto options = MakeOptions();
  options.imu.enabled = true;
  Frontend frontend(options);
  auto input = MakeScan(100, UpdateKind::kLidarInertial);
  frontend.Process(&input);
  input.measures.clear();
  input.measures.emplace_back();
  input.measures.back().vio_time = 100.125;
  input.lio_vio_flg = UpdateKind::kVisual;
  const auto result = frontend.Process(&input);
  EXPECT_FALSE(result.updated);
  EXPECT_FALSE(result.first_frame);
  EXPECT_EQ(result.kind, UpdateKind::kVisual);
}

TEST(FrontendTest, ImuDiagnosticsExposeOriginalUncorrectedMeans) {
  auto options = MakeOptions();
  options.imu.enabled = true;
  options.imu.initialization_frames = 1;
  Frontend frontend(options);
  auto input = MakeScan(100, UpdateKind::kLidarInertial);
  const auto initialized = frontend.Process(&input);
  EXPECT_TRUE(initialized.imu_initialized);
  EXPECT_TRUE(initialized.imu_diagnostics.empty());
  input = MakeScan(100.25, UpdateKind::kLidarInertial);
  const auto result = frontend.Process(&input);
  ASSERT_EQ(result.imu_diagnostics.size(), 2);
  EXPECT_DOUBLE_EQ(result.imu_diagnostics[0].relative_time, 0.125);
  EXPECT_DOUBLE_EQ(result.imu_diagnostics[1].relative_time, 0.3125);
  for (const auto& diagnostic : result.imu_diagnostics) {
    EXPECT_TRUE(diagnostic.angular_velocity.isZero(0));
    EXPECT_TRUE(diagnostic.acceleration.isApprox(
        Eigen::Vector3d(0, 0, -9.81), 0));
  }
}

TEST(FrontendTest, OptionalMapSnapshotsExposePlanesAndEffectivePoints) {
  auto options = MakeOptions();
  options.output_effective_points = true;
  options.voxel_map.is_pub_plane_map_ = true;
  options.voxel_map.max_voxel_size_ = 10;
  Frontend frontend(options);
  auto input = MakeScan();
  input.lidar->clear();
  for (float x : {1, 2, 3}) {
    for (float y : {1, 2, 3}) {
      Point point;
      point.x = x;
      point.y = y;
      point.z = 0;
      point.curvature = 125;
      input.lidar->push_back(point);
    }
  }
  const auto result = frontend.Process(&input);
  ASSERT_TRUE(result.updated);
  ASSERT_TRUE(result.effective_cloud);
  EXPECT_TRUE(result.has_plane_visualization);
  ASSERT_FALSE(result.planes.empty());
  EXPECT_TRUE(result.planes.front().valid);
  EXPECT_NEAR(result.planes.front().center.x(), 2, 1e-6);
  // Nine initial points plus the first six insertions trigger the old map's
  // refresh: (9 * 2 + 3 * 1 + 3 * 2) / 15 = 1.8. The last three wait.
  EXPECT_NEAR(result.planes.front().center.y(), 1.8, 1e-6);
  EXPECT_NEAR(result.planes.front().normal.norm(), 1, 1e-12);
}

TEST(FrontendTest, ExtrinsicsTransformOutputCloud) {
  auto options = MakeOptions();
  options.calibration.imu_from_lidar.rotation << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  options.calibration.imu_from_lidar.translation << 1, 2, 3;
  Frontend frontend(options);
  auto input = MakeScan();
  const auto result = frontend.Process(&input);
  ASSERT_TRUE(result.updated);
  ASSERT_EQ(result.world_cloud->size(), 2);
  EXPECT_FLOAT_EQ(result.world_cloud->front().x, 1);
  EXPECT_FLOAT_EQ(result.world_cloud->front().y, 4);
  EXPECT_FLOAT_EQ(result.world_cloud->front().z, 3);
  EXPECT_FLOAT_EQ(result.world_cloud->back().y, 5);
}

}  // namespace
}  // namespace livo_atlas
