#include "livo_atlas/mapping/map_correction.h"

#include <cmath>
#include <stdexcept>

#include "mapping/pose_math.h"

namespace livo_atlas {
namespace {

constexpr double kRigidTolerance = 1e-9;

void CheckRigid(const RigidTransform& pose) {
  if (!pose.rotation.allFinite() || !pose.translation.allFinite())
    throw std::invalid_argument("Global pose transform must be finite");
  const Eigen::Matrix3d error =
      pose.rotation.transpose() * pose.rotation - Eigen::Matrix3d::Identity();
  const double determinant = pose.rotation.determinant();
  if (!error.allFinite() || !std::isfinite(determinant) ||
      error.cwiseAbs().maxCoeff() > kRigidTolerance ||
      std::abs(determinant - 1) > kRigidTolerance)
    throw std::invalid_argument("Global pose rotation must be rigid");
}

}  // namespace

RigidTransform ApplyMapCorrection(
    const MapCorrection& correction,
    const RigidTransform& published_odom_from_body) {
  CheckRigid(correction.map_from_odom);
  CheckRigid(published_odom_from_body);
  if (correction.pose_version == 0) {
    if (correction.optimized_frame_count != 0 ||
        (correction.map_from_odom.rotation - Eigen::Matrix3d::Identity())
                .cwiseAbs()
                .maxCoeff() > kRigidTolerance ||
        correction.map_from_odom.translation.cwiseAbs().maxCoeff() >
            kRigidTolerance)
      throw std::invalid_argument(
          "Version zero requires an identity map correction");
  } else if (correction.optimized_frame_count == 0 ||
             correction.optimized_frame_count > correction.frame_count) {
    throw std::invalid_argument("Invalid map correction optimized prefix");
  }
  const auto result =
      pose_math::Compose(correction.map_from_odom, published_odom_from_body);
  if (!result.rotation.allFinite() || !result.translation.allFinite())
    throw std::invalid_argument("Global pose composition must be finite");
  return result;
}

}  // namespace livo_atlas
