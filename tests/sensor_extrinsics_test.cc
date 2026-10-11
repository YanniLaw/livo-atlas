#include <array>
#include <cmath>
#include <limits>
#include <string>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "livo_atlas/core/sensor_extrinsics.h"

namespace livo_atlas {
namespace {

TEST(SensorExtrinsicsTest, DefaultsToIdentityTransforms) {
  const SensorExtrinsics extrinsics;
  EXPECT_TRUE(extrinsics.lidar_to_imu_transform.matrix().isIdentity());
  EXPECT_TRUE(extrinsics.lidar_to_camera_transform.matrix().isIdentity());

  std::string error = "stale error";
  EXPECT_TRUE(ValidateSensorExtrinsics(extrinsics, &error)) << error;
  EXPECT_TRUE(error.empty());
}

TEST(SensorExtrinsicsTest, AcceptsRoundedAviaRotationWithoutChangingIt) {
  SensorExtrinsics extrinsics;
  const std::array<double, 9> rotation_values = {
      0.00610193, -0.999863, -0.0154172,
      -0.00615449, 0.0153796, -0.999863,
      0.999962, 0.00619598, -0.0060598};
  for (Eigen::Index row = 0; row < 3; ++row) {
    for (Eigen::Index column = 0; column < 3; ++column) {
      extrinsics.lidar_to_camera_transform.linear()(row, column) =
          rotation_values[row * 3 + column];
    }
  }

  const Eigen::Matrix3d original_rotation =
      extrinsics.lidar_to_camera_transform.linear();
  std::string error;
  ASSERT_TRUE(ValidateSensorExtrinsics(extrinsics, &error)) << error;
  EXPECT_EQ(0.0,
            (extrinsics.lidar_to_camera_transform.linear() - original_rotation).norm());
}

TEST(SensorExtrinsicsTest, RowMajorCameraTransformMapsLidarPointsToCamera) {
  SensorExtrinsics extrinsics;
  const std::array<double, 9> rotation_values = {
      0.00610193, -0.999863, -0.0154172,
      -0.00615449, 0.0153796, -0.999863,
      0.999962, 0.00619598, -0.0060598};
  for (Eigen::Index row = 0; row < 3; ++row) {
    for (Eigen::Index column = 0; column < 3; ++column) {
      extrinsics.lidar_to_camera_transform.linear()(row, column) =
          rotation_values[row * 3 + column];
    }
  }
  extrinsics.lidar_to_camera_transform.translation() =
      Eigen::Vector3d(0.0194384, 0.104689, -0.0251952);

  const Eigen::Vector3d point_lidar(1.4, -0.2, 0.5);
  const Eigen::Vector3d expected_point_camera(
      rotation_values[0] * point_lidar.x() +
          rotation_values[1] * point_lidar.y() +
          rotation_values[2] * point_lidar.z() + 0.0194384,
      rotation_values[3] * point_lidar.x() +
          rotation_values[4] * point_lidar.y() +
          rotation_values[5] * point_lidar.z() + 0.104689,
      rotation_values[6] * point_lidar.x() +
          rotation_values[7] * point_lidar.y() +
          rotation_values[8] * point_lidar.z() - 0.0251952);

  const Eigen::Vector3d point_camera =
      extrinsics.lidar_to_camera_transform * point_lidar;
  EXPECT_LT((point_camera - expected_point_camera).norm(), 1e-12);
  EXPECT_LT((extrinsics.lidar_to_camera_transform.inverse() * point_camera - point_lidar)
                .norm(),
            1e-4);
}

TEST(SensorExtrinsicsTest, RejectsNonFiniteValuesAndNamesTheParameter) {
  SensorExtrinsics extrinsics;
  extrinsics.lidar_to_camera_transform.translation().x() =
      std::numeric_limits<double>::quiet_NaN();

  std::string error;
  EXPECT_FALSE(ValidateSensorExtrinsics(extrinsics, &error));
  EXPECT_NE(std::string::npos, error.find("extrin_calib/Pcl"));

  extrinsics = SensorExtrinsics();
  extrinsics.lidar_to_camera_transform.linear()(0, 0) =
      std::numeric_limits<double>::infinity();
  EXPECT_FALSE(ValidateSensorExtrinsics(extrinsics, &error));
  EXPECT_NE(std::string::npos, error.find("extrin_calib/Rcl"));

  extrinsics = SensorExtrinsics();
  extrinsics.lidar_to_imu_transform.translation().y() =
      std::numeric_limits<double>::infinity();
  EXPECT_FALSE(ValidateSensorExtrinsics(extrinsics, &error));
  EXPECT_NE(std::string::npos, error.find("extrin_calib/extrinsic_T"));

  extrinsics = SensorExtrinsics();
  extrinsics.lidar_to_imu_transform.matrix()(3, 0) =
      std::numeric_limits<double>::infinity();
  EXPECT_FALSE(ValidateSensorExtrinsics(extrinsics, &error));
  EXPECT_NE(std::string::npos, error.find("extrin_calib/extrinsic_R"));
}

TEST(SensorExtrinsicsTest, RejectsZeroReflectionAndDistortedRotations) {
  SensorExtrinsics extrinsics;
  std::string error;

  extrinsics.lidar_to_imu_transform.linear().setZero();
  EXPECT_FALSE(ValidateSensorExtrinsics(extrinsics, &error));
  EXPECT_NE(std::string::npos, error.find("extrin_calib/extrinsic_R"));

  extrinsics = SensorExtrinsics();
  extrinsics.lidar_to_imu_transform.linear()(0, 0) = -1.0;
  EXPECT_FALSE(ValidateSensorExtrinsics(extrinsics, &error));
  EXPECT_NE(std::string::npos, error.find("extrin_calib/extrinsic_R"));

  extrinsics = SensorExtrinsics();
  extrinsics.lidar_to_camera_transform.linear()(0, 0) = 1.001;
  EXPECT_FALSE(ValidateSensorExtrinsics(extrinsics, &error));
  EXPECT_NE(std::string::npos, error.find("extrin_calib/Rcl"));
}

}  // namespace
}  // namespace livo_atlas

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
