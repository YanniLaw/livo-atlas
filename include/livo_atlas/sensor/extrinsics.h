#ifndef LIVO_ATLAS_SENSOR_EXTRINSICS_H_
#define LIVO_ATLAS_SENSOR_EXTRINSICS_H_

#include <Eigen/Core>
#include <string>
#include <vector>

namespace livo_atlas {

// T_A_B maps coordinates from B into A: p_A = rotation * p_B + translation.
// Translation is in meters. Calibration rotations are never orthogonalized.
struct RigidTransform {
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
};

// An empty error string denotes success.
std::string ValidateExtrinsic(const RigidTransform& transform);

// Rotation has nine row-major coefficients; translation has three coefficients.
// On failure, result is unchanged.
std::string ParseExtrinsic(const std::vector<double>& rotation,
                           const std::vector<double>& translation,
                           RigidTransform* result);

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_SENSOR_EXTRINSICS_H_
