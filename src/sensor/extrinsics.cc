#include "livo_atlas/sensor/extrinsics.h"

#include <cmath>

#include <Eigen/LU>

namespace livo_atlas {
namespace {

// Existing calibration files round rotation coefficients to about six digits.
constexpr double kRotationTolerance = 1e-4;

}  // namespace

std::string ValidateExtrinsic(const RigidTransform& transform) {
  if (!transform.rotation.allFinite()) {
    return "rotation must contain finite coefficients";
  }
  if (!transform.translation.allFinite()) {
    return "translation must contain finite coefficients";
  }
  const Eigen::Matrix3d residual =
      transform.rotation.transpose() * transform.rotation -
      Eigen::Matrix3d::Identity();
  if (residual.cwiseAbs().maxCoeff() > kRotationTolerance ||
      std::abs(transform.rotation.determinant() - 1.0) > kRotationTolerance) {
    return "rotation must be right-handed and orthonormal (tolerance 1e-4)";
  }
  return {};
}

std::string ParseExtrinsic(const std::vector<double>& rotation,
                          const std::vector<double>& translation,
                          RigidTransform* result) {
  if (rotation.size() != 9) {
    return "rotation must contain 9 row-major coefficients";
  }
  if (translation.size() != 3) {
    return "translation must contain 3 coefficients";
  }
  if (result == nullptr) {
    return "extrinsic output must not be null";
  }
  RigidTransform transform;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      transform.rotation(row, col) = rotation[3 * row + col];
    }
    transform.translation(row) = translation[row];
  }
  const std::string error = ValidateExtrinsic(transform);
  if (!error.empty()) {
    return error;
  }
  *result = transform;
  return {};
}

}  // namespace livo_atlas
