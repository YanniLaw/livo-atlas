#ifndef LIVO_ATLAS_CAMERA_EQUIDISTANT_CAMERA_H_
#define LIVO_ATLAS_CAMERA_EQUIDISTANT_CAMERA_H_

#include <array>

#include "livo_atlas/camera/camera.h"

namespace livo_atlas {

class EquidistantCamera final : public Camera {
 public:
  explicit EquidistantCamera(const CameraConfig& config);
  using Camera::Project;
  using Camera::Unproject;
  Eigen::Vector2d Project(const Eigen::Vector2d& point) const override;
  Eigen::Vector3d Unproject(const Eigen::Vector2d& pixel) const override;

 private:
  bool distorted_;
  std::array<double, 4> distortion_;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_CAMERA_EQUIDISTANT_CAMERA_H_
