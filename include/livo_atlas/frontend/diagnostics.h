#ifndef LIVO_ATLAS_FRONTEND_DIAGNOSTICS_H_
#define LIVO_ATLAS_FRONTEND_DIAGNOSTICS_H_

#include <string>

#include <Eigen/Core>
#include <opencv2/core/mat.hpp>

namespace livo_atlas {

struct ImuDiagnostic {
  double relative_time = 0.0;
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
};

struct DebugImage {
  std::string name;
  cv::Mat image;
};

struct PlaneVisualization {
  int id = 0;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d x_axis = Eigen::Vector3d::UnitX();
  Eigen::Vector3d y_axis = Eigen::Vector3d::UnitY();
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d variance_diagonal = Eigen::Vector3d::Zero();
  // Minimum, middle and maximum eigenvalues, in this order.
  Eigen::Vector3d eigenvalues = Eigen::Vector3d::Zero();
  bool valid = false;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_FRONTEND_DIAGNOSTICS_H_
