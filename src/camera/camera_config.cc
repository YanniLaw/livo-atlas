#include "livo_atlas/camera/camera_config.h"

#include <cmath>
#include <limits>

namespace livo_atlas {

std::string ValidateCameraConfig(const CameraConfig& config) {
  if (config.model != "Pinhole" && config.model != "EquidistantCamera") {
    return "unsupported cam_model: " + config.model;
  }
  if (config.width <= 0 || config.height <= 0) {
    return "cam_width and cam_height must be positive";
  }
  if (!std::isfinite(config.scale) || config.scale <= 0) {
    return "camera scale must be finite and positive";
  }
  const double width = config.width * config.scale;
  const double height = config.height * config.scale;
  constexpr int kMaxIndex = std::numeric_limits<int>::max();
  if (!std::isfinite(width) || !std::isfinite(height) || width < 1 ||
      height < 1 || width > kMaxIndex || height > kMaxIndex) {
    return "scaled camera dimensions must fit positive integers";
  }
  // VIO uses int arithmetic for row offsets into an image.
  if (static_cast<int>(width) > kMaxIndex / static_cast<int>(height)) {
    return "scaled camera pixel count exceeds the image index range";
  }
  const std::array<double, 4> intrinsics = {
      config.fx, config.fy, config.cx, config.cy};
  const std::array<const char*, 4> names = {
      "cam_fx", "cam_fy", "cam_cx", "cam_cy"};
  constexpr double kMaxFloat = std::numeric_limits<float>::max();
  for (std::size_t i = 0; i < intrinsics.size(); ++i) {
    const double scaled = intrinsics[i] * config.scale;
    if (!std::isfinite(intrinsics[i]) || !std::isfinite(scaled)) {
      return std::string(names[i]) + " must remain finite after scaling";
    }
    if (i < 2 && scaled <= 0) {
      return std::string(names[i]) + " must remain positive after scaling";
    }
    // PinholeCamera also stores calibration in float OpenCV matrices.
    if (config.model == "Pinhole" &&
        (std::abs(scaled) > kMaxFloat ||
         (i < 2 && static_cast<float>(scaled) == 0))) {
      return std::string(names[i]) + " is outside the pinhole matrix range";
    }
  }
  for (double coefficient : config.distortion) {
    if (!std::isfinite(coefficient) ||
        (config.model == "Pinhole" && std::abs(coefficient) > kMaxFloat)) {
      return "camera distortion must fit finite model coefficients";
    }
  }
  return {};
}

}  // namespace livo_atlas
