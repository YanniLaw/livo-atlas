#ifndef LIVO_ATLAS_CAMERA_PINHOLE_CAMERA_H_
#define LIVO_ATLAS_CAMERA_PINHOLE_CAMERA_H_

#include <array>

#include <opencv2/core.hpp>

#include "livo_atlas/camera/camera.h"

namespace livo_atlas {

class PinholeCamera final : public Camera {
 public:
  explicit PinholeCamera(const CameraConfig& config);
  using Camera::Project;
  using Camera::Unproject;
  Eigen::Vector2d Project(const Eigen::Vector2d& point) const override;
  Eigen::Vector3d Unproject(const Eigen::Vector2d& pixel) const override;
  void UndistortImage(const cv::Mat& raw, cv::Mat* output) const;

 private:
  bool distorted_;
  std::array<double, 5> distortion_;
  cv::Mat intrinsics_;
  cv::Mat coefficients_;
  cv::Mat map_x_;
  cv::Mat map_y_;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_CAMERA_PINHOLE_CAMERA_H_
