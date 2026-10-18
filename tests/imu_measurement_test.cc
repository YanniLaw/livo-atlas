#include "IMU_Processing.h"

#include <cmath>

#include <gtest/gtest.h>

namespace livo_atlas {
namespace {

ImuMeasurement MakeImu(double timestamp, double yaw_rate = 0) {
  return {timestamp, Eigen::Vector3d(0, 0, yaw_rate),
          Eigen::Vector3d(0, 0, -2)};
}

TEST(ImuMeasurementTest, InitializationNormalizesRawAccelerationInAlgorithm) {
  ImuProcess process;
  process.set_imu_init_frame_num(1000);  // Keep the legacy file writer inactive.
  StatesGroup state;
  LidarMeasureGroup input;
  input.lio_vio_flg = LIO;
  MeasureGroup group;
  group.imu = {MakeImu(0), MakeImu(0.01), MakeImu(0.02)};
  group.lio_time = 0.02;
  input.measures.push_back(group);
  PointCloudXYZI::Ptr output(new PointCloudXYZI());
  process.Process2(input, state, output);
  EXPECT_DOUBLE_EQ(process.IMU_mean_acc_norm, 2);
  EXPECT_TRUE(state.gravity.isApprox(Eigen::Vector3d(0, 0, 9.81), 1e-15));
  EXPECT_TRUE(state.cov.allFinite());
  EXPECT_TRUE(process.imu_need_init);
}

TEST(ImuMeasurementTest, PropagationCarriesTheLastSampleAcrossFrames) {
  ImuProcess process;
  process.set_imu_init_frame_num(1000);
  StatesGroup state;
  LidarMeasureGroup input;
  input.lio_vio_flg = LIO;
  MeasureGroup group;
  group.imu = {MakeImu(0)};
  input.measures.push_back(group);
  PointCloudXYZI::Ptr output(new PointCloudXYZI());
  process.Process2(input, state, output);

  // Exercise propagation without crossing the legacy file-opening branch.
  process.imu_need_init = false;
  input.measures.back().imu.clear();
  input.measures.back().lio_time = 0.1;
  process.Process2(input, state, output);  // Establish a known propagation end.
  input.measures.back().imu = {MakeImu(0.2, 2)};
  input.measures.back().lio_time = 0.2;
  process.Process2(input, state, output);
  EXPECT_NEAR(std::atan2(state.rot_end(1, 0), state.rot_end(0, 0)), 0.1, 1e-12);
  input.measures.back().imu = {MakeImu(0.3, 4)};
  input.measures.back().lio_time = 0.3;
  process.Process2(input, state, output);
  EXPECT_NEAR(std::atan2(state.rot_end(1, 0), state.rot_end(0, 0)), 0.4, 1e-12);
  EXPECT_NEAR(state.pos_end.norm(), 0, 1e-12);
  EXPECT_NEAR(state.vel_end.norm(), 0, 1e-12);
  EXPECT_TRUE(state.cov.allFinite());
  EXPECT_DOUBLE_EQ(input.last_lio_update_time, 0.3);
}

}  // namespace
}  // namespace livo_atlas
