#include "livo_atlas/camera/camera.h"

#include <stdexcept>

namespace livo_atlas {

Camera::Camera(const CameraConfig& config) {
  const std::string error = ValidateCameraConfig(config);
  if (!error.empty()) throw std::invalid_argument(error);
  width_ = static_cast<int>(config.width * config.scale);
  height_ = static_cast<int>(config.height * config.scale);
  scale_ = config.scale;
  fx_ = config.fx * config.scale;
  fy_ = config.fy * config.scale;
  cx_ = config.cx * config.scale;
  cy_ = config.cy * config.scale;
}

bool Camera::IsInFrame(const Eigen::Vector2i& pixel, int border,
                       int level) const {
  if (border < 0 || level < 0 || level >= 31) return false;
  const int divisor = 1 << level;
  return pixel.x() >= border && pixel.y() >= border &&
         pixel.x() < width_ / divisor - border &&
         pixel.y() < height_ / divisor - border;
}

}  // namespace livo_atlas
