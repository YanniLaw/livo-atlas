#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <memory>
#include <new>
#include <type_traits>

#include "livo_atlas/frontend/imu/imu_processor.h"

namespace livo_atlas {
namespace {

Imu MakeImu(double timestamp, double yaw_rate = 0) {
  return {timestamp, Eigen::Vector3d(0, 0, yaw_rate),
          Eigen::Vector3d(0, 0, -2)};
}

TEST(ImuMeasurementTest, InitializationNormalizesRawAccelerationInAlgorithm) {
  ImuProcessor process;
  process.first_lidar_time = 0;
  process.SetInitializationFrames(
      1000);  // Keep the legacy file writer inactive.
  FilterState state;
  SynchronizedMeasurements input;
  input.update_kind = UpdateKind::kLidarInertial;
  UpdateMeasurements group;
  group.imu = {MakeImu(0), MakeImu(0.01), MakeImu(0.02)};
  group.lidar_update_time = 0.02;
  input.updates.push_back(group);
  PointCloud::Ptr output(new PointCloud());
  process.Process(input, state, output);
  EXPECT_DOUBLE_EQ(process.IMU_mean_acc_norm, 2);
  EXPECT_TRUE(state.gravity.isApprox(Eigen::Vector3d(0, 0, 9.81), 1e-15));
  EXPECT_TRUE(state.cov.allFinite());
  EXPECT_TRUE(process.imu_need_init);
}

TEST(ImuMeasurementTest, PropagationCarriesTheLastSampleAcrossFrames) {
  ImuProcessor process;
  process.first_lidar_time = 0;
  process.SetInitializationFrames(1000);
  FilterState state;
  SynchronizedMeasurements input;
  input.update_kind = UpdateKind::kLidarInertial;
  UpdateMeasurements group;
  group.imu = {MakeImu(0)};
  input.updates.push_back(group);
  PointCloud::Ptr output(new PointCloud());
  process.Process(input, state, output);

  // Exercise propagation without crossing the legacy file-opening branch.
  process.imu_need_init = false;
  input.updates.back().imu.clear();
  input.updates.back().lidar_update_time = 0.1;
  process.Process(input, state, output);  // Establish a known propagation end.
  input.updates.back().imu = {MakeImu(0.2, 2)};
  input.updates.back().lidar_update_time = 0.2;
  process.Process(input, state, output);
  EXPECT_NEAR(std::atan2(state.rot_end(1, 0), state.rot_end(0, 0)), 0.1, 1e-12);
  input.updates.back().imu = {MakeImu(0.3, 4)};
  input.updates.back().lidar_update_time = 0.3;
  process.Process(input, state, output);
  EXPECT_NEAR(std::atan2(state.rot_end(1, 0), state.rot_end(0, 0)), 0.4, 1e-12);
  EXPECT_NEAR(state.pos_end.norm(), 0, 1e-12);
  EXPECT_NEAR(state.vel_end.norm(), 0, 1e-12);
  EXPECT_TRUE(state.cov.allFinite());
  EXPECT_DOUBLE_EQ(input.last_lidar_update_time, 0.3);
}

TEST(ImuMeasurementTest, FirstPropagationDoesNotDependOnPreviousStorageBytes) {
  std::aligned_storage_t<sizeof(ImuProcessor), alignof(ImuProcessor)> storage;
  std::memset(&storage, 0x7f, sizeof(storage));
  const auto destroy = [](ImuProcessor* process) { process->~ImuProcessor(); };
  std::unique_ptr<ImuProcessor, decltype(destroy)> process(
      new (&storage) ImuProcessor(), destroy);
  process->first_lidar_time = 0;
  process->SetInitializationFrames(1000);
  FilterState state;
  SynchronizedMeasurements input;
  input.update_kind = UpdateKind::kLidarInertial;
  UpdateMeasurements group;
  group.imu = {MakeImu(0)};
  input.updates.push_back(group);
  PointCloud::Ptr output(new PointCloud());
  process->Process(input, state, output);
  process->imu_need_init = false;
  input.updates.back().imu = {MakeImu(0.2, 2)};
  input.updates.back().lidar_update_time = 0.2;
  process->Process(input, state, output);
  // Average yaw rate 1 rad/s for 0.2 s, even in reused nonzero storage.
  EXPECT_NEAR(std::atan2(state.rot_end(1, 0), state.rot_end(0, 0)), 0.2, 1e-12);
}

TEST(ImuMeasurementTest, LidarOnlyDeskewDoesNotInheritPreviousSensorModel) {
  std::aligned_storage_t<sizeof(ImuProcessor), alignof(ImuProcessor)> storage;
  auto* previous = new (&storage) ImuProcessor();
  previous->lidar_type = kDepthCamera;
  previous->~ImuProcessor();
  const auto destroy = [](ImuProcessor* process) { process->~ImuProcessor(); };
  std::unique_ptr<ImuProcessor, decltype(destroy)> process(
      new (&storage) ImuProcessor(), destroy);
  process->DisableImu();
  FilterState state;
  state.vel_end = Eigen::Vector3d(1, 0, 0);
  SynchronizedMeasurements input;
  input.update_kind = UpdateKind::kLidarOnly;
  input.scan_start_time = 100;
  for (int i = 0; i < 3; ++i) {
    Point point{};
    point.x = i + 1;
    point.curvature = i * 62.5;
    input.lidar_cloud->push_back(point);
  }
  PointCloud::Ptr output(new PointCloud);
  process->Process(input, state, output);
  ASSERT_EQ(output->size(), 3);
  // The middle return precedes the scan end by 62.5 ms at 1 m/s.
  EXPECT_FLOAT_EQ(output->points[1].x, 1.9375F);
}

}  // namespace
}  // namespace livo_atlas
