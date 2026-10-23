#ifndef LIVO_ATLAS_CAMERA_CAMERA_H_
#define LIVO_ATLAS_CAMERA_CAMERA_H_

#include <Eigen/Core>

#include "livo_atlas/camera/camera_config.h"

namespace livo_atlas {

class Camera {
 public:
  explicit Camera(const CameraConfig& config);
  virtual ~Camera() = default;

  int width() const { return width_; }
  int height() const { return height_; }
  double scale() const { return scale_; }
  double fx() const { return fx_; }
  double fy() const { return fy_; }
  double cx() const { return cx_; }
  double cy() const { return cy_; }

  virtual Eigen::Vector2d Project(const Eigen::Vector2d& point) const = 0;
  Eigen::Vector2d Project(const Eigen::Vector3d& point) const {
    return Project(Eigen::Vector2d(point.head<2>() / point.z()));
  }
  virtual Eigen::Vector3d Unproject(const Eigen::Vector2d& pixel) const = 0;
  Eigen::Vector3d Unproject(double u, double v) const {
    return Unproject(Eigen::Vector2d(u, v));
  }
  bool IsInFrame(const Eigen::Vector2i& pixel, int border = 0,
                 int level = 0) const;

 private:
  int width_;
  int height_;
  double scale_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_CAMERA_CAMERA_H_
