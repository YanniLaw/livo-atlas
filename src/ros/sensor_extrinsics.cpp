#include "livo_atlas/ros/sensor_extrinsics.h"

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace livo_atlas {
namespace {

bool ReadRequiredArray(const ros::NodeHandle& nh, const std::string& name,
                       std::size_t expected_size, std::vector<double>* values,
                       std::string* error) {
  std::vector<double> parameter_values;
  if (!nh.getParam(name, parameter_values)) {
    if (error != nullptr) {
      *error = name + " is required and must be a numeric array";
    }
    return false;
  }
  if (parameter_values.size() != expected_size) {
    if (error != nullptr) {
      *error = name + " must contain exactly " +
               std::to_string(expected_size) + " values (got " +
               std::to_string(parameter_values.size()) + ")";
    }
    return false;
  }
  *values = std::move(parameter_values);
  return true;
}

bool ReadRotation(const ros::NodeHandle& nh, const std::string& name,
                  Eigen::Matrix3d* rotation, std::string* error) {
  std::vector<double> values;
  if (!ReadRequiredArray(nh, name, 9, &values, error)) {
    return false;
  }
  for (Eigen::Index row = 0; row < 3; ++row) {
    for (Eigen::Index column = 0; column < 3; ++column) {
      (*rotation)(row, column) = values[row * 3 + column];
    }
  }
  return true;
}

bool ReadTranslation(const ros::NodeHandle& nh, const std::string& name,
                     Eigen::Vector3d* translation, std::string* error) {
  std::vector<double> values;
  if (!ReadRequiredArray(nh, name, 3, &values, error)) {
    return false;
  }
  for (Eigen::Index index = 0; index < 3; ++index) {
    (*translation)(index) = values[index];
  }
  return true;
}

}  // namespace

bool LoadSensorExtrinsics(const ros::NodeHandle& nh, SensorExtrinsics* output,
                          std::string* error) {
  if (output == nullptr) {
    if (error != nullptr) {
      *error = "sensor extrinsics output must not be null";
    }
    return false;
  }

  SensorExtrinsics candidate;
  Eigen::Matrix3d lidar_to_imu_rotation;
  Eigen::Vector3d lidar_to_imu_translation;
  Eigen::Matrix3d lidar_to_camera_rotation;
  Eigen::Vector3d lidar_to_camera_translation;
  if (!ReadRotation(nh, "extrin_calib/extrinsic_R",
                    &lidar_to_imu_rotation, error) ||
      !ReadTranslation(nh, "extrin_calib/extrinsic_T",
                       &lidar_to_imu_translation, error) ||
      !ReadRotation(nh, "extrin_calib/Rcl",
                    &lidar_to_camera_rotation, error) ||
      !ReadTranslation(nh, "extrin_calib/Pcl",
                       &lidar_to_camera_translation, error)) {
    return false;
  }

  candidate.lidar_to_imu_transform.linear() = lidar_to_imu_rotation;
  candidate.lidar_to_imu_transform.translation() = lidar_to_imu_translation;
  candidate.lidar_to_camera_transform.linear() = lidar_to_camera_rotation;
  candidate.lidar_to_camera_transform.translation() = lidar_to_camera_translation;
  if (!ValidateSensorExtrinsics(candidate, error)) {
    return false;
  }

  *output = candidate;
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

}  // namespace livo_atlas
