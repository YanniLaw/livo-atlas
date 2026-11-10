#ifndef LIVO_ATLAS_SRC_COMMON_ROTATION_PROJECTION_H_
#define LIVO_ATLAS_SRC_COMMON_ROTATION_PROJECTION_H_

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

namespace livo_atlas {

// The caller validates the rotation before projecting a temporary copy.
inline Eigen::Matrix3d ProjectRotation(const Eigen::Matrix3d& rotation) {
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d direction = Eigen::Matrix3d::Identity();
  if ((svd.matrixU() * svd.matrixV().transpose()).determinant() < 0) {
    direction(2, 2) = -1;
  }
  return svd.matrixU() * direction * svd.matrixV().transpose();
}

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_SRC_COMMON_ROTATION_PROJECTION_H_
