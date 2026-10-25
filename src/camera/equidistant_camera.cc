#include "livo_atlas/camera/equidistant_camera.h"

#include <cmath>
#include <stdexcept>

namespace livo_atlas {

// Numerical conventions follow Christian Forster's vikit EquidistantCamera.
EquidistantCamera::EquidistantCamera(const CameraConfig& config)
    : Camera(config),
      distorted_(std::abs(config.distortion[0]) > 1e-7),
      distortion_(config.distortion) {
  if (config.model != "EquidistantCamera") {
    throw std::invalid_argument(
        "EquidistantCamera requires equidistant calibration");
  }
}

Eigen::Vector2d EquidistantCamera::Project(const Eigen::Vector2d& point) const {
  if (!distorted_) return {fx() * point.x() + cx(), fy() * point.y() + cy()};
  const double radius = point.norm();
  // Preserve the legacy near-axis branch; changing the model is separate work.
  if (radius < 1e-8) return point;
  const double theta = std::atan(radius);
  const double theta2 = theta * theta;
  const double theta4 = theta2 * theta2;
  const double theta6 = theta4 * theta2;
  const double theta8 = theta4 * theta4;
  const double distorted_theta = theta *
      (1 + distortion_[0] * theta2 + distortion_[1] * theta4 +
       distortion_[2] * theta6 + distortion_[3] * theta8);
  const double factor = distorted_theta / radius;
  return {point.x() * factor * fx() + cx(), point.y() * factor * fy() + cy()};
}

Eigen::Vector3d EquidistantCamera::Unproject(
    const Eigen::Vector2d& pixel) const {
  double x = (pixel.x() - cx()) / fx();
  double y = (pixel.y() - cy()) / fy();
  if (distorted_) {
    const double distorted_theta = std::sqrt(x * x + y * y);
    double theta = distorted_theta;
    for (int i = 0; i < 5; ++i) {
      const double theta2 = theta * theta;
      const double theta4 = theta2 * theta2;
      const double theta6 = theta4 * theta2;
      const double theta8 = theta4 * theta4;
      theta = distorted_theta / (1 + distortion_[0] * theta2 +
          distortion_[1] * theta4 + distortion_[2] * theta6 +
          distortion_[3] * theta8);
    }
    // At the optical axis the old model returns NaN. Keep that contract here.
    const double factor = std::tan(theta) / distorted_theta;
    x *= factor;
    y *= factor;
  }
  return Eigen::Vector3d(x, y, 1).normalized();
}

}  // namespace livo_atlas
