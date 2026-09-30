#ifndef LIVO_ATLAS_INCLUDE_LIVO_ATLAS_CORE_SENSOR_EXTRINSICS_H_
#define LIVO_ATLAS_INCLUDE_LIVO_ATLAS_CORE_SENSOR_EXTRINSICS_H_

#include <cmath>
#include <string>

#include <Eigen/Geometry>

#include "livo_atlas/core/pose3d.h"

namespace livo_atlas {

// Sensor transforms map points from the LiDAR frame into the named target
// frame. The camera transform preserves the direction of the legacy Rcl/Pcl
// parameters; it is not an IMU-to-camera transform.
struct SensorExtrinsics {
  // Rigid transform from the LiDAR frame to the IMU frame.
  Pose3d lidar_to_imu_transform = Pose3d::Identity();
  // Rigid transform from the LiDAR frame to the camera frame.
  Pose3d lidar_to_camera_transform = Pose3d::Identity();
};

namespace sensor_extrinsics_internal {

inline void SetError(const std::string& message, std::string* error) {
  if (error != nullptr) {
    *error = message;
  }
}

inline bool ValidatePose(const Pose3d& pose,
                         const char* rotation_parameter,
                         const char* translation_parameter,
                         std::string* error) {
  if (!pose.linear().allFinite()) {
    SetError(std::string(rotation_parameter) + " contains non-finite values",
             error);
    return false;
  }
  if (!pose.translation().allFinite()) {
    SetError(std::string(translation_parameter) +
                 " contains non-finite values",
             error);
    return false;
  }
  if (!pose.matrix().allFinite()) {
    SetError(std::string(rotation_parameter) + " or " + translation_parameter +
                 " produces a non-finite homogeneous transform",
             error);
    return false;
  }

  Eigen::Matrix<double, 1, 4> expected_bottom_row;
  expected_bottom_row << 0.0, 0.0, 0.0, 1.0;
  if (!(pose.matrix().row(3).array() == expected_bottom_row.array()).all()) {
    SetError(std::string(rotation_parameter) + " or " + translation_parameter +
                 " does not form a valid homogeneous transform",
             error);
    return false;
  }

  const Eigen::Matrix3d orthogonality_error =
      pose.linear().transpose() * pose.linear() - Eigen::Matrix3d::Identity();
  if (!std::isfinite(orthogonality_error.norm()) ||
      orthogonality_error.norm() > 1e-4) {
    SetError(std::string(rotation_parameter) +
                 " is not orthogonal within tolerance 1e-4",
             error);
    return false;
  }

  const double determinant_error = std::abs(pose.linear().determinant() - 1.0);
  if (!std::isfinite(determinant_error) || determinant_error > 1e-4) {
    SetError(std::string(rotation_parameter) +
                 " determinant differs from +1 by more than 1e-4",
             error);
    return false;
  }
  return true;
}

}  // namespace sensor_extrinsics_internal

// Validate both rigid transforms without normalizing or changing their values.
inline bool ValidateSensorExtrinsics(const SensorExtrinsics& extrinsics,
                                     std::string* error) {
  if (!sensor_extrinsics_internal::ValidatePose(
          extrinsics.lidar_to_imu_transform, "extrin_calib/extrinsic_R",
          "extrin_calib/extrinsic_T", error)) {
    return false;
  }
  if (!sensor_extrinsics_internal::ValidatePose(
          extrinsics.lidar_to_camera_transform, "extrin_calib/Rcl", "extrin_calib/Pcl",
          error)) {
    return false;
  }
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_INCLUDE_LIVO_ATLAS_CORE_SENSOR_EXTRINSICS_H_
