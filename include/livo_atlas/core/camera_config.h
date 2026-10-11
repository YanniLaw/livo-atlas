#ifndef LIVO_ATLAS_INCLUDE_LIVO_ATLAS_CORE_CAMERA_CONFIG_H_
#define LIVO_ATLAS_INCLUDE_LIVO_ATLAS_CORE_CAMERA_CONFIG_H_

#include <cmath>
#include <limits>
#include <string>
#include <type_traits>
#include <variant>

namespace livo_atlas {

struct OcamCameraConfig {
  std::string calibration_file;
};

struct PinholeCameraConfig {
  int width = 0;
  int height = 0;
  double scale = 1.0;
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  double d0 = 0.0;
  double d1 = 0.0;
  double d2 = 0.0;
  double d3 = 0.0;
};

struct EquidistantCameraConfig {
  int width = 0;
  int height = 0;
  double scale = 1.0;
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  double k1 = 0.0;
  double k2 = 0.0;
  double k3 = 0.0;
  double k4 = 0.0;
};

struct PolynomialCameraConfig {
  int width = 0;
  int height = 0;
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  double skew = 0.0;
  double k2 = 0.0;
  double k3 = 0.0;
  double k4 = 0.0;
  double k5 = 0.0;
  double k6 = 0.0;
  double k7 = 0.0;
};

struct AtanCameraConfig {
  int width = 0;
  int height = 0;
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  double distortion = 0.0;
};

using CameraModelConfig =
    std::variant<OcamCameraConfig, PinholeCameraConfig,
                 EquidistantCameraConfig, PolynomialCameraConfig,
                 AtanCameraConfig>;

struct CameraConfig {
  CameraModelConfig model;
};

namespace camera_config_internal {

inline bool SetError(const std::string& message, std::string* error) {
  if (error != nullptr) {
    *error = message;
  }
  return false;
}

inline bool ValidateDimensions(int width, int height, std::string* error) {
  if (width <= 0) {
    return SetError("cam_width must be greater than zero", error);
  }
  if (height <= 0) {
    return SetError("cam_height must be greater than zero", error);
  }
  return true;
}

inline bool ValidateIntrinsics(double fx, double fy, double cx, double cy,
                               std::string* error) {
  if (!std::isfinite(fx) || fx <= 0.0) {
    return SetError("cam_fx must be finite and greater than zero", error);
  }
  if (!std::isfinite(fy) || fy <= 0.0) {
    return SetError("cam_fy must be finite and greater than zero", error);
  }
  if (!std::isfinite(cx)) {
    return SetError("cam_cx must be finite", error);
  }
  if (!std::isfinite(cy)) {
    return SetError("cam_cy must be finite", error);
  }
  return true;
}

inline bool ValidateScale(double scale, int width, int height,
                          std::string* error) {
  if (!std::isfinite(scale) || scale <= 0.0) {
    return SetError("scale must be finite and greater than zero", error);
  }
  const double scaled_width = width * scale;
  const double scaled_height = height * scale;
  if (!std::isfinite(scaled_width) || !std::isfinite(scaled_height) ||
      scaled_width < 1.0 || scaled_height < 1.0 ||
      scaled_width > std::numeric_limits<int>::max() ||
      scaled_height > std::numeric_limits<int>::max()) {
    return SetError("scale produces invalid camera dimensions", error);
  }
  return true;
}

inline bool ValidateDistortion(double value, const char* name,
                               std::string* error) {
  if (!std::isfinite(value)) {
    return SetError(std::string(name) + " must be finite", error);
  }
  return true;
}

inline bool ValidatePinhole(const PinholeCameraConfig& config,
                            std::string* error) {
  if (!ValidateDimensions(config.width, config.height, error) ||
      !ValidateIntrinsics(config.fx, config.fy, config.cx, config.cy, error) ||
      !ValidateScale(config.scale, config.width, config.height, error)) {
    return false;
  }
  return ValidateDistortion(config.d0, "cam_d0", error) &&
         ValidateDistortion(config.d1, "cam_d1", error) &&
         ValidateDistortion(config.d2, "cam_d2", error) &&
         ValidateDistortion(config.d3, "cam_d3", error);
}

inline bool ValidateEquidistant(const EquidistantCameraConfig& config,
                                std::string* error) {
  if (!ValidateDimensions(config.width, config.height, error) ||
      !ValidateIntrinsics(config.fx, config.fy, config.cx, config.cy, error) ||
      !ValidateScale(config.scale, config.width, config.height, error)) {
    return false;
  }
  return ValidateDistortion(config.k1, "k1", error) &&
         ValidateDistortion(config.k2, "k2", error) &&
         ValidateDistortion(config.k3, "k3", error) &&
         ValidateDistortion(config.k4, "k4", error);
}

inline bool ValidatePolynomial(const PolynomialCameraConfig& config,
                               std::string* error) {
  if (!ValidateDimensions(config.width, config.height, error) ||
      !ValidateIntrinsics(config.fx, config.fy, config.cx, config.cy, error) ||
      !ValidateDistortion(config.skew, "cam_skew", error)) {
    return false;
  }
  return ValidateDistortion(config.k2, "k2", error) &&
         ValidateDistortion(config.k3, "k3", error) &&
         ValidateDistortion(config.k4, "k4", error) &&
         ValidateDistortion(config.k5, "k5", error) &&
         ValidateDistortion(config.k6, "k6", error) &&
         ValidateDistortion(config.k7, "k7", error);
}

inline bool ValidateAtan(const AtanCameraConfig& config,
                         std::string* error) {
  return ValidateDimensions(config.width, config.height, error) &&
         ValidateIntrinsics(config.fx, config.fy, config.cx, config.cy,
                            error) &&
         ValidateDistortion(config.distortion, "cam_d0", error);
}

}  // namespace camera_config_internal

// Validate values before a Vikit camera constructor can use dimensions or
// intrinsics in image allocation and projection math.
inline bool ValidateCameraConfig(const CameraConfig& config,
                                 std::string* error) {
  bool valid = std::visit(
      [error](const auto& model) {
        using Model = std::decay_t<decltype(model)>;
        if constexpr (std::is_same_v<Model, OcamCameraConfig>) {
          if (model.calibration_file.empty()) {
            return camera_config_internal::SetError(
                "cam_calib_file must not be empty", error);
          }
          return true;
        } else if constexpr (std::is_same_v<Model, PinholeCameraConfig>) {
          return camera_config_internal::ValidatePinhole(model, error);
        } else if constexpr (std::is_same_v<Model,
                                            EquidistantCameraConfig>) {
          return camera_config_internal::ValidateEquidistant(model, error);
        } else if constexpr (std::is_same_v<Model,
                                            PolynomialCameraConfig>) {
          return camera_config_internal::ValidatePolynomial(model, error);
        } else {
          return camera_config_internal::ValidateAtan(model, error);
        }
      },
      config.model);
  if (valid && error != nullptr) {
    error->clear();
  }
  return valid;
}

// Vikit scales dimensions and intrinsics only for these two camera models.
inline double CameraImageResizeScale(const CameraConfig& config) {
  if (const auto* model = std::get_if<PinholeCameraConfig>(&config.model)) {
    return model->scale;
  }
  if (const auto* model =
          std::get_if<EquidistantCameraConfig>(&config.model)) {
    return model->scale;
  }
  return 1.0;
}

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_INCLUDE_LIVO_ATLAS_CORE_CAMERA_CONFIG_H_
