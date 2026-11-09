#include "colmap_camera.h"

#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>

namespace livo_atlas {
namespace {
void CheckPose(const RigidTransform& pose) {
  const auto error = ValidateExtrinsic(pose);
  if (!error.empty())
    throw std::invalid_argument("Export camera pose: " + error);
}

RigidTransform Compose(const RigidTransform& a, const RigidTransform& b) {
  RigidTransform result{a.rotation * b.rotation,
                        a.rotation * b.translation + a.translation};
  CheckPose(result);
  return result;
}

RigidTransform Inverse(const RigidTransform& pose) {
  const Eigen::Matrix3d rotation = pose.rotation.transpose();
  return {rotation, -rotation * pose.translation};
}

RigidTransform RigidPose(const RigidTransform& pose) {
  CheckPose(pose);
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      pose.rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
  return {svd.matrixU() * svd.matrixV().transpose(), pose.translation};
}

Eigen::Vector3d Position(const Point& point, const RigidTransform& pose) {
  const Eigen::Vector3d result =
      pose.rotation * Eigen::Vector3d(point.x, point.y, point.z) +
      pose.translation;
  if (!result.allFinite())
    throw std::invalid_argument("Nonfinite exported point position");
  return result;
}

Eigen::Vector2d Project(const Eigen::Vector3d& camera_point,
                        const CameraConfig& camera) {
  return {camera.fx * (camera_point.x() / camera_point.z()) + camera.cx,
          camera.fy * (camera_point.y() / camera_point.z()) + camera.cy};
}
}  // namespace

ColmapCamera::ColmapCamera(const CameraConfig& config) : pinhole_(config) {
  const auto error = ValidateCameraConfig(config);
  if (!error.empty()) throw std::invalid_argument(error);
  pinhole_.width = static_cast<int>(config.width * config.scale);
  pinhole_.height = static_cast<int>(config.height * config.scale);
  pinhole_.fx *= config.scale;
  pinhole_.fy *= config.scale;
  pinhole_.cx = pinhole_.width / 2.0;
  pinhole_.cy = pinhole_.height / 2.0;
  pinhole_.model = "Pinhole";
  pinhole_.scale = 1;
  pinhole_.distortion = {};
  const cv::Mat intrinsics =
      (cv::Mat_<double>(3, 3) << pinhole_.fx, 0, config.cx * config.scale, 0,
       pinhole_.fy, config.cy * config.scale, 0, 0, 1);
  // OpenCV pixel centers are integers; COLMAP centers are offset by (0.5,0.5).
  // Centering the virtual camera also supports 3DGS's symmetric FOV projection.
  const cv::Mat target =
      (cv::Mat_<double>(3, 3) << pinhole_.fx, 0, pinhole_.cx - .5, 0,
       pinhole_.fy, pinhole_.cy - .5, 0, 0, 1);
  cv::Mat distortion(4, 1, CV_64F);
  for (int i = 0; i < 4; ++i) distortion.at<double>(i) = config.distortion[i];
  const cv::Size size(pinhole_.width, pinhole_.height);
  if (config.model == "Pinhole") {
    cv::initUndistortRectifyMap(intrinsics, distortion,
                                cv::Mat::eye(3, 3, CV_64F), target, size,
                                CV_32FC1, map_x_, map_y_);
  } else {
    cv::fisheye::initUndistortRectifyMap(intrinsics, distortion,
                                         cv::Mat::eye(3, 3, CV_64F), target,
                                         size, CV_32FC1, map_x_, map_y_);
  }
  valid_ = cv::Mat::zeros(size, CV_8UC1);
  for (int y = 0; y < size.height; ++y) {
    for (int x = 0; x < size.width; ++x) {
      const float sx = map_x_.at<float>(y, x), sy = map_y_.at<float>(y, x);
      if (std::isfinite(sx) && std::isfinite(sy) && sx >= 0 && sy >= 0 &&
          sx < size.width - 1 && sy < size.height - 1) {
        valid_.at<unsigned char>(y, x) = 1;
      }
    }
  }
}

ColmapView ColmapCamera::MakeView(const Keyframe& frame,
                                  const RigidTransform& parent_from_body,
                                  std::uint32_t max_points) const {
  if (max_points == 0 || max_points == UINT32_MAX)
    throw std::invalid_argument(
        "Export point limit must be in [1, UINT32_MAX)");
  ValidateKeyframe(frame);
  if (!frame.image_timestamp || frame.image.cols != pinhole_.width ||
      frame.image.rows != pinhole_.height)
    throw std::invalid_argument("Export requires a calibrated visual keyframe");
  // Historical snapshots allow small calibration/rotation roundoff. Project
  // each accepted rotation once before composing either points or cameras.
  const auto parent_pose = RigidPose(parent_from_body);
  const auto camera_from_body = Compose(RigidPose(*frame.camera_from_odom),
                                        RigidPose(frame.odom_from_body));
  ColmapView view;
  view.camera_from_parent = Compose(camera_from_body, Inverse(parent_pose));
  // Use the same normalized rotation for projection and COLMAP serialization.
  view.camera_from_parent.rotation =
      Eigen::Quaterniond(view.camera_from_parent.rotation)
          .normalized()
          .toRotationMatrix();
  cv::remap(frame.image, view.image, map_x_, map_y_, cv::INTER_LINEAR,
            cv::BORDER_CONSTANT);
  const auto no_point = std::numeric_limits<std::size_t>::max();
  std::vector<std::size_t> nearest(valid_.total(), no_point);
  std::vector<double> depths(valid_.total(),
                             std::numeric_limits<double>::infinity());
  for (std::size_t i = 0; i < frame.cloud->size(); ++i) {
    const auto position = Position((*frame.cloud)[i], parent_pose);
    const auto camera_point = view.camera_from_parent.rotation * position +
                              view.camera_from_parent.translation;
    if (!camera_point.allFinite() || camera_point.z() <= 0) continue;
    const auto pixel = Project(camera_point, pinhole_);
    if (!pixel.allFinite() || pixel.x() < 0 || pixel.y() < 0 ||
        pixel.x() >= pinhole_.width || pixel.y() >= pinhole_.height)
      continue;
    const int x = static_cast<int>(pixel.x()), y = static_cast<int>(pixel.y());
    if (!valid_.at<unsigned char>(y, x)) continue;
    const auto offset = static_cast<std::size_t>(y) * pinhole_.width + x;
    if (camera_point.z() < depths[offset]) {
      depths[offset] = camera_point.z();
      nearest[offset] = i;
    }
  }
  const auto visible = static_cast<std::size_t>(
      std::count_if(nearest.begin(), nearest.end(),
                    [&](auto index) { return index != no_point; }));
  const auto stride = std::max<std::size_t>(
      1, visible / max_points + (visible % max_points != 0));
  view.samples.reserve(std::min<std::size_t>(visible, max_points));
  std::size_t rank = 0;
  for (std::size_t offset = 0; offset < nearest.size(); ++offset) {
    if (nearest[offset] == no_point) continue;
    if (rank++ % stride != 0) continue;
    const auto position =
        Position((*frame.cloud)[nearest[offset]], parent_pose);
    const auto camera_point = view.camera_from_parent.rotation * position +
                              view.camera_from_parent.translation;
    const auto pixel = Project(camera_point, pinhole_);
    const auto bgr = view.image.at<cv::Vec3b>(offset / pinhole_.width,
                                              offset % pinhole_.width);
    view.samples.push_back({position, pixel, {bgr[2], bgr[1], bgr[0]}});
  }
  return view;
}
}  // namespace livo_atlas
