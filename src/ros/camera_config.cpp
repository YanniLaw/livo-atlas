#include "livo_atlas/ros/camera_config.h"

#include <fstream>
#include <string>
#include <type_traits>
#include <utility>

#include <vikit/atan_camera.h>
#include <vikit/equidistant_camera.h>
#include <vikit/omni_camera.h>
#include <vikit/pinhole_camera.h>
#include <vikit/polynomial_camera.h>

namespace livo_atlas {
namespace ros {
namespace {

std::string ParameterName(const std::string& namespace_prefix,
                          const std::string& parameter) {
  if (namespace_prefix.empty()) {
    return parameter;
  }
  if (namespace_prefix.back() == '/') {
    return namespace_prefix + parameter;
  }
  return namespace_prefix + "/" + parameter;
}

bool SetError(const std::string& message, std::string* error) {
  if (error != nullptr) {
    *error = message;
  }
  return false;
}

template <typename T>
bool ReadRequired(const ::ros::NodeHandle& nh, const std::string& name,
                  T* value, std::string* error) {
  if (value == nullptr) {
    return SetError("camera config output must not be null", error);
  }
  if (!nh.getParam(name, *value)) {
    return SetError(name + " is required and has the wrong or missing type",
                    error);
  }
  return true;
}

template <typename T>
bool ReadOptional(const ::ros::NodeHandle& nh, const std::string& name,
                  const T& default_value, T* value, std::string* error) {
  if (value == nullptr) {
    return SetError("camera config output must not be null", error);
  }
  if (!nh.hasParam(name)) {
    *value = default_value;
    return true;
  }
  if (!nh.getParam(name, *value)) {
    return SetError(name + " has the wrong type", error);
  }
  return true;
}

bool ReadDimensionsAndIntrinsics(const ::ros::NodeHandle& nh,
                                 const std::string& prefix, int* width,
                                 int* height, double* fx, double* fy,
                                 double* cx, double* cy,
                                 std::string* error) {
  return ReadRequired(nh, ParameterName(prefix, "cam_width"), width, error) &&
         ReadRequired(nh, ParameterName(prefix, "cam_height"), height,
                      error) &&
         ReadRequired(nh, ParameterName(prefix, "cam_fx"), fx, error) &&
         ReadRequired(nh, ParameterName(prefix, "cam_fy"), fy, error) &&
         ReadRequired(nh, ParameterName(prefix, "cam_cx"), cx, error) &&
         ReadRequired(nh, ParameterName(prefix, "cam_cy"), cy, error);
}

bool ReadPinhole(const ::ros::NodeHandle& nh, const std::string& prefix,
                 PinholeCameraConfig* config, std::string* error) {
  return ReadDimensionsAndIntrinsics(nh, prefix, &config->width,
                                     &config->height, &config->fx,
                                     &config->fy, &config->cx, &config->cy,
                                     error) &&
         ReadOptional(nh, ParameterName(prefix, "scale"), 1.0,
                      &config->scale, error) &&
         ReadOptional(nh, ParameterName(prefix, "cam_d0"), 0.0,
                      &config->d0, error) &&
         ReadOptional(nh, ParameterName(prefix, "cam_d1"), 0.0,
                      &config->d1, error) &&
         ReadOptional(nh, ParameterName(prefix, "cam_d2"), 0.0,
                      &config->d2, error) &&
         ReadOptional(nh, ParameterName(prefix, "cam_d3"), 0.0,
                      &config->d3, error);
}

bool ReadEquidistant(const ::ros::NodeHandle& nh, const std::string& prefix,
                     EquidistantCameraConfig* config, std::string* error) {
  return ReadDimensionsAndIntrinsics(nh, prefix, &config->width,
                                     &config->height, &config->fx,
                                     &config->fy, &config->cx, &config->cy,
                                     error) &&
         ReadOptional(nh, ParameterName(prefix, "scale"), 1.0,
                      &config->scale, error) &&
         ReadOptional(nh, ParameterName(prefix, "k1"), 0.0, &config->k1,
                      error) &&
         ReadOptional(nh, ParameterName(prefix, "k2"), 0.0, &config->k2,
                      error) &&
         ReadOptional(nh, ParameterName(prefix, "k3"), 0.0, &config->k3,
                      error) &&
         ReadOptional(nh, ParameterName(prefix, "k4"), 0.0, &config->k4,
                      error);
}

bool ReadPolynomial(const ::ros::NodeHandle& nh, const std::string& prefix,
                    PolynomialCameraConfig* config, std::string* error) {
  return ReadDimensionsAndIntrinsics(nh, prefix, &config->width,
                                     &config->height, &config->fx,
                                     &config->fy, &config->cx, &config->cy,
                                     error) &&
         ReadOptional(nh, ParameterName(prefix, "cam_skew"), 0.0,
                      &config->skew, error) &&
         ReadOptional(nh, ParameterName(prefix, "k2"), 0.0, &config->k2,
                      error) &&
         ReadOptional(nh, ParameterName(prefix, "k3"), 0.0, &config->k3,
                      error) &&
         ReadOptional(nh, ParameterName(prefix, "k4"), 0.0, &config->k4,
                      error) &&
         ReadOptional(nh, ParameterName(prefix, "k5"), 0.0, &config->k5,
                      error) &&
         ReadOptional(nh, ParameterName(prefix, "k6"), 0.0, &config->k6,
                      error) &&
         ReadOptional(nh, ParameterName(prefix, "k7"), 0.0, &config->k7,
                      error);
}

bool ReadAtan(const ::ros::NodeHandle& nh, const std::string& prefix,
              AtanCameraConfig* config, std::string* error) {
  return ReadDimensionsAndIntrinsics(nh, prefix, &config->width,
                                     &config->height, &config->fx,
                                     &config->fy, &config->cx, &config->cy,
                                     error) &&
         ReadOptional(nh, ParameterName(prefix, "cam_d0"), 0.0,
                      &config->distortion, error);
}

}  // namespace

bool LoadCameraConfig(const ::ros::NodeHandle& nh,
                      const std::string& namespace_prefix,
                      CameraConfig* output, std::string* error) {
  if (output == nullptr) {
    return SetError("camera config output must not be null", error);
  }

  const std::string model_name =
      ParameterName(namespace_prefix, "cam_model");
  std::string model_value;
  if (!ReadRequired(nh, model_name, &model_value, error)) {
    return false;
  }

  CameraConfig candidate;
  if (model_value == "Ocam") {
    OcamCameraConfig model;
    const std::string calibration_parameter =
        ParameterName(namespace_prefix, "cam_calib_file");
    if (!ReadOptional(nh, calibration_parameter, std::string(),
                      &model.calibration_file, error)) {
      return false;
    }
    candidate.model = std::move(model);
  } else if (model_value == "Pinhole") {
    PinholeCameraConfig model;
    if (!ReadPinhole(nh, namespace_prefix, &model, error)) {
      return false;
    }
    candidate.model = model;
  } else if (model_value == "EquidistantCamera") {
    EquidistantCameraConfig model;
    if (!ReadEquidistant(nh, namespace_prefix, &model, error)) {
      return false;
    }
    candidate.model = model;
  } else if (model_value == "PolynomialCamera") {
    PolynomialCameraConfig model;
    if (!ReadPolynomial(nh, namespace_prefix, &model, error)) {
      return false;
    }
    candidate.model = model;
  } else if (model_value == "ATAN") {
    AtanCameraConfig model;
    if (!ReadAtan(nh, namespace_prefix, &model, error)) {
      return false;
    }
    candidate.model = model;
  } else {
    return SetError(model_name + " has unsupported model '" + model_value +
                        "'",
                    error);
  }

  std::string validation_error;
  if (!ValidateCameraConfig(candidate, &validation_error)) {
    return SetError(ParameterName(namespace_prefix, validation_error), error);
  }
  if (const auto* model = std::get_if<OcamCameraConfig>(&candidate.model)) {
    std::ifstream calibration(model->calibration_file);
    if (!calibration.good()) {
      return SetError(ParameterName(namespace_prefix,
                                    "cam_calib_file cannot be opened"),
                      error);
    }
  }

  *output = std::move(candidate);
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

std::unique_ptr<vk::AbstractCamera> MakeVikitCameraModel(
    const CameraConfig& config) {
  if (!ValidateCameraConfig(config, nullptr)) {
    return nullptr;
  }
  return std::visit(
      [](const auto& model) -> std::unique_ptr<vk::AbstractCamera> {
        using Model = std::decay_t<decltype(model)>;
        if constexpr (std::is_same_v<Model, OcamCameraConfig>) {
          return std::make_unique<vk::OmniCamera>(model.calibration_file);
        } else if constexpr (std::is_same_v<Model, PinholeCameraConfig>) {
          return std::make_unique<vk::PinholeCamera>(
              model.width, model.height, model.scale, model.fx, model.fy,
              model.cx, model.cy, model.d0, model.d1, model.d2, model.d3);
        } else if constexpr (std::is_same_v<Model,
                                            EquidistantCameraConfig>) {
          return std::make_unique<vk::EquidistantCamera>(
              model.width, model.height, model.scale, model.fx, model.fy,
              model.cx, model.cy, model.k1, model.k2, model.k3, model.k4);
        } else if constexpr (std::is_same_v<Model,
                                            PolynomialCameraConfig>) {
          return std::make_unique<vk::PolynomialCamera>(
              model.width, model.height, model.fx, model.fy, model.cx,
              model.cy, model.skew, model.k2, model.k3, model.k4, model.k5,
              model.k6, model.k7);
        } else {
          return std::make_unique<vk::ATANCamera>(
              model.width, model.height, model.fx, model.fy, model.cx,
              model.cy, model.distortion);
        }
      },
      config.model);
}

}  // namespace ros
}  // namespace livo_atlas
