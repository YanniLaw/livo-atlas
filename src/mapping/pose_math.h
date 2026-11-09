#ifndef LIVO_ATLAS_SRC_MAPPING_POSE_MATH_H_
#define LIVO_ATLAS_SRC_MAPPING_POSE_MATH_H_
#include <Eigen/Geometry>
#include <stdexcept>

#include "livo_atlas/sensor/extrinsics.h"
namespace livo_atlas::pose_math {
inline void Validate(const RigidTransform& pose) {
  const auto error = ValidateExtrinsic(pose);
  if (!error.empty()) throw std::invalid_argument(error);
}
inline RigidTransform Inverse(const RigidTransform& pose) {
  const Eigen::Matrix3d rotation = pose.rotation.transpose();
  return {rotation, -rotation * pose.translation};
}
inline RigidTransform Compose(const RigidTransform& a,
                              const RigidTransform& b) {
  return {a.rotation * b.rotation, a.rotation * b.translation + a.translation};
}
inline bool Equal(const RigidTransform& a, const RigidTransform& b) {
  return (a.translation - b.translation).norm() <= 1e-9 &&
         Eigen::AngleAxisd(a.rotation.transpose() * b.rotation).angle() <= 1e-9;
}
}  // namespace livo_atlas::pose_math
#endif  // LIVO_ATLAS_SRC_MAPPING_POSE_MATH_H_
