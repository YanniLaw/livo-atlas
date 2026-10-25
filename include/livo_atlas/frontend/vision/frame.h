/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIVO_ATLAS_FRONTEND_VISION_FRAME_H_
#define LIVO_ATLAS_FRONTEND_VISION_FRAME_H_

#include <sophus/se3.h>

#include <Eigen/Core>
#include <atomic>
#include <cstddef>
#include <list>
#include <memory>
#include <opencv2/core.hpp>

#include "livo_atlas/camera/camera.h"

namespace livo_atlas {

class VisualPoint;
struct Feature;

using Features = std::list<Feature*>;

/// A frame saves the image, the associated features and the estimated pose.
class Frame {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static std::atomic<int> frame_counter_;  //!< Assigns unique frame ids.
  int id_;                                 //!< Unique id of the frame.
  livo_atlas::Camera* cam_;                //!< Camera model.
  Sophus::SE3 T_f_w_;                      //!< Transform (f)rame from (w)orld.
  Sophus::SE3 T_f_w_prior_;  //!< Transform (f)rame from (w)orld provided by the
                             //!< IMU prior.
  cv::Mat img_;              //!< Image of the frame.
  Features fts_;             //!< List of features in the image.

  Frame(livo_atlas::Camera* cam, const cv::Mat& img);
  ~Frame();
  Frame(const Frame&) = delete;
  Frame& operator=(const Frame&) = delete;

  /// Initialize the grayscale frame.
  void Initialize(const cv::Mat& img);

  /// Return number of point observations.
  std::size_t ObservationCount() const { return fts_.size(); }

  /// Transforms point coordinates in world-frame (w) to camera pixel
  /// coordinates (c).
  Eigen::Vector2d WorldToPixel(const Eigen::Vector3d& xyz_w) const {
    return cam_->Project(Eigen::Vector3d(T_f_w_ * xyz_w));
  }

  /// Project a world point using the IMU prior pose.
  Eigen::Vector2d WorldToPixelWithPrior(const Eigen::Vector3d& xyz_w) const {
    return cam_->Project(Eigen::Vector3d(T_f_w_prior_ * xyz_w));
  }

  /// Transforms pixel coordinates (c) to frame unit sphere coordinates (f).
  Eigen::Vector3d PixelToBearing(const Eigen::Vector2d& px) const {
    return cam_->Unproject(px);
  }

  /// Transforms pixel coordinates (c) to frame unit sphere coordinates (f).
  Eigen::Vector3d PixelToBearing(double x, double y) const {
    return cam_->Unproject(x, y);
  }

  /// Transforms point coordinates in world-frame (w) to camera-frams (f).
  Eigen::Vector3d WorldToCamera(const Eigen::Vector3d& xyz_w) const {
    return T_f_w_ * xyz_w;
  }

  /// Transform a frame point into the world frame.
  Eigen::Vector3d CameraToWorld(const Eigen::Vector3d& f) const {
    return T_f_w_.inverse() * f;
  }

  /// Projects Point from unit sphere (f) in camera pixels (c).
  Eigen::Vector2d BearingToPixel(const Eigen::Vector3d& f) const {
    return cam_->Project(f);
  }

  /// Return the pose of the frame in the (w)orld coordinate frame.
  Eigen::Vector3d CameraPositionInWorld() const {
    return T_f_w_.inverse().translation();
  }
};

typedef std::unique_ptr<Frame> FramePtr;

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_FRONTEND_VISION_FRAME_H_
