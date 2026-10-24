#include "livo_atlas/camera/pinhole_camera.h"

#include <cmath>
#include <stdexcept>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace livo_atlas {

// Numerical conventions follow Christian Forster's vikit PinholeCamera.
PinholeCamera::PinholeCamera(const CameraConfig& config)
    : Camera(config),
      distorted_(std::abs(config.distortion[0]) > 1e-7),
      distortion_{config.distortion[0], config.distortion[1],
                  config.distortion[2], config.distortion[3], 0} {
  if (config.model != "Pinhole") {
    throw std::invalid_argument("PinholeCamera requires Pinhole calibration");
  }
  intrinsics_ =
      (cv::Mat_<float>(3, 3) << fx(), 0, cx(), 0, fy(), cy(), 0, 0, 1);
  coefficients_ = (cv::Mat_<float>(1, 5) << distortion_[0], distortion_[1],
                    distortion_[2], distortion_[3], distortion_[4]);
  cv::initUndistortRectifyMap(intrinsics_, coefficients_,
      cv::Mat_<double>::eye(3, 3), intrinsics_, cv::Size(width(), height()),
      CV_16SC2, map_x_, map_y_);
}

Eigen::Vector2d PinholeCamera::Project(const Eigen::Vector2d& point) const {
  if (!distorted_) return {fx() * point.x() + cx(), fy() * point.y() + cy()};
  const double x = point.x();
  const double y = point.y();
  const double r2 = x * x + y * y;
  const double r4 = r2 * r2;
  const double r6 = r4 * r2;
  const double a1 = 2 * x * y;
  const double a2 = r2 + 2 * x * x;
  const double a3 = r2 + 2 * y * y;
  const double radial = 1 + distortion_[0] * r2 + distortion_[1] * r4 +
                        distortion_[4] * r6;
  const double xd = x * radial + distortion_[2] * a1 + distortion_[3] * a2;
  const double yd = y * radial + distortion_[2] * a3 + distortion_[3] * a1;
  return {xd * fx() + cx(), yd * fy() + cy()};
}

Eigen::Vector3d PinholeCamera::Unproject(const Eigen::Vector2d& pixel) const {
  Eigen::Vector3d bearing;
  if (!distorted_) {
    bearing = {(pixel.x() - cx()) / fx(), (pixel.y() - cy()) / fy(), 1};
  } else {
    cv::Point2f source(pixel.x(), pixel.y());
    cv::Point2f destination;
    const cv::Mat input(1, 1, CV_32FC2, &source.x);
    cv::Mat output(1, 1, CV_32FC2, &destination.x);
    cv::undistortPoints(input, output, intrinsics_, coefficients_);
    bearing = {destination.x, destination.y, 1};
  }
  return bearing.normalized();
}

void PinholeCamera::UndistortImage(const cv::Mat& raw, cv::Mat* output) const {
  if (output == nullptr) throw std::invalid_argument("null undistorted image");
  if (distorted_) {
    cv::remap(raw, *output, map_x_, map_y_, cv::INTER_LINEAR);
  } else {
    *output = raw.clone();
  }
}

}  // namespace livo_atlas
