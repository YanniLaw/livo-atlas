#include "livo_atlas/ros1/calibration_reader.h"

#include <array>
#include <utility>
#include <vector>

#include <XmlRpcValue.h>

namespace livo_atlas::ros1 {
namespace {

bool ReadNumber(const XmlRpc::XmlRpcValue& value, double* result) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    *result = static_cast<int>(value);
    return true;
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    *result = static_cast<double>(value);
    return true;
  }
  return false;
}

std::string ReadArray(const ros::NodeHandle& node, const std::string& name,
                     std::vector<double>* result) {
  XmlRpc::XmlRpcValue value;
  if (!node.getParam(name, value) ||
      value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return node.resolveName(name) + ": expected a numeric array";
  }
  std::vector<double> numbers(value.size());
  for (int i = 0; i < value.size(); ++i) {
    if (!ReadNumber(value[i], &numbers[i])) {
      return node.resolveName(name) + ": array elements must be numeric";
    }
  }
  *result = std::move(numbers);
  return {};
}

std::string ReadTransform(const ros::NodeHandle& node,
                         const std::string& rotation_name,
                         const std::string& translation_name,
                         RigidTransform* result) {
  std::vector<double> rotation;
  std::vector<double> translation;
  std::string error = ReadArray(node, rotation_name, &rotation);
  if (!error.empty()) {
    return error;
  }
  error = ReadArray(node, translation_name, &translation);
  if (!error.empty()) {
    return error;
  }
  error = ParseExtrinsic(rotation, translation, result);
  if (!error.empty()) {
    return node.resolveName(rotation_name) + " / " +
           node.resolveName(translation_name) + ": " + error;
  }
  return {};
}

std::string ReadCameraNumber(const ros::NodeHandle& node,
                            const std::string& name, double* result,
                            bool optional = false, double fallback = 0.0) {
  XmlRpc::XmlRpcValue value;
  if (!node.getParam(name, value)) {
    if (optional) {
      *result = fallback;
      return {};
    }
    return node.resolveName(name) + ": required numeric parameter is missing";
  }
  if (!ReadNumber(value, result)) {
    return node.resolveName(name) + ": expected a numeric value";
  }
  return {};
}

std::string ReadCamera(const ros::NodeHandle& node, CameraConfig* config) {
  XmlRpc::XmlRpcValue value;
  if (!node.getParam("cam_model", value) ||
      value.getType() != XmlRpc::XmlRpcValue::TypeString) {
    return node.resolveName("cam_model") + ": expected a model name";
  }
  config->model = static_cast<std::string>(value);
  const std::array<const char*, 2> dimensions = {"cam_width", "cam_height"};
  const std::array<int*, 2> outputs = {&config->width, &config->height};
  for (std::size_t i = 0; i < dimensions.size(); ++i) {
    if (!node.getParam(dimensions[i], value) ||
        value.getType() != XmlRpc::XmlRpcValue::TypeInt) {
      return node.resolveName(dimensions[i]) +
             ": expected an integer dimension";
    }
    *outputs[i] = static_cast<int>(value);
  }
  const std::array<const char*, 4> names = {
      "cam_fx", "cam_fy", "cam_cx", "cam_cy"};
  const std::array<double*, 4> intrinsics = {
      &config->fx, &config->fy, &config->cx, &config->cy};
  for (std::size_t i = 0; i < names.size(); ++i) {
    const std::string error = ReadCameraNumber(node, names[i], intrinsics[i]);
    if (!error.empty()) {
      return error;
    }
  }
  std::string error =
      ReadCameraNumber(node, "scale", &config->scale, true, 1.0);
  if (!error.empty()) {
    return error;
  }
  const std::array<const char*, 4> pinhole = {
      "cam_d0", "cam_d1", "cam_d2", "cam_d3"};
  const std::array<const char*, 4> equidistant = {"k1", "k2", "k3", "k4"};
  const auto& distortion = config->model == "Pinhole" ? pinhole : equidistant;
  for (std::size_t i = 0; i < distortion.size(); ++i) {
    error = ReadCameraNumber(node, distortion[i], &config->distortion[i], true);
    if (!error.empty()) {
      return error;
    }
  }
  error = ValidateCameraConfig(*config);
  return error.empty() ? error : node.getNamespace() + ": " + error;
}

}  // namespace

std::string ReadCalibration(const ros::NodeHandle& node,
                           const ros::NodeHandle& camera_node,
                           Calibration* result) {
  if (result == nullptr) {
    return "calibration output must not be null";
  }
  Calibration calibration;
  std::string error = ReadTransform(node, "extrin_calib/extrinsic_R",
                                    "extrin_calib/extrinsic_T",
                                    &calibration.imu_from_lidar);
  if (!error.empty()) {
    return error;
  }
  error = ReadTransform(node, "extrin_calib/Rcl", "extrin_calib/Pcl",
                       &calibration.camera_from_lidar);
  if (!error.empty()) {
    return error;
  }
  error = ReadCamera(camera_node, &calibration.camera);
  if (!error.empty()) {
    return error;
  }
  *result = calibration;
  return {};
}

}  // namespace livo_atlas::ros1
