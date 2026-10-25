#ifndef LIVO_ATLAS_COMMON_SO3_MATH_H_
#define LIVO_ATLAS_COMMON_SO3_MATH_H_

#include <cmath>

#include <Eigen/Core>

namespace livo_atlas {

// Preserve FAST-LIVO2's distinct thresholds for each Exp overload.
template <typename T>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1>&& angle) {
  const T angle_norm = angle.norm();
  const Eigen::Matrix<T, 3, 3> identity = Eigen::Matrix<T, 3, 3>::Identity();
  if (angle_norm > 0.0000001) {
    const Eigen::Matrix<T, 3, 1> axis = angle / angle_norm;
    Eigen::Matrix<T, 3, 3> skew;
    skew << 0.0, -axis[2], axis[1], axis[2], 0.0, -axis[0], -axis[1],
        axis[0], 0.0;
    return identity + std::sin(angle_norm) * skew +
           (1.0 - std::cos(angle_norm)) * skew * skew;
  }
  return identity;
}

template <typename T, typename Ts>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1>& angular_velocity,
                         const Ts& dt) {
  const T velocity_norm = angular_velocity.norm();
  const Eigen::Matrix<T, 3, 3> identity = Eigen::Matrix<T, 3, 3>::Identity();
  if (velocity_norm > 0.0000001) {
    const Eigen::Matrix<T, 3, 1> axis = angular_velocity / velocity_norm;
    Eigen::Matrix<T, 3, 3> skew;
    skew << 0.0, -axis[2], axis[1], axis[2], 0.0, -axis[0], -axis[1],
        axis[0], 0.0;
    const T angle = velocity_norm * dt;
    return identity + std::sin(angle) * skew +
           (1.0 - std::cos(angle)) * skew * skew;
  }
  return identity;
}

template <typename T>
Eigen::Matrix<T, 3, 3> Exp(const T& x, const T& y, const T& z) {
  const T norm = std::sqrt(x * x + y * y + z * z);
  const Eigen::Matrix<T, 3, 3> identity = Eigen::Matrix<T, 3, 3>::Identity();
  if (norm > 0.00001) {
    const T axis[3] = {x / norm, y / norm, z / norm};
    Eigen::Matrix<T, 3, 3> skew;
    skew << 0.0, -axis[2], axis[1], axis[2], 0.0, -axis[0], -axis[1],
        axis[0], 0.0;
    return identity + std::sin(norm) * skew +
           (1.0 - std::cos(norm)) * skew * skew;
  }
  return identity;
}

template <typename T>
Eigen::Matrix<T, 3, 1> Log(const Eigen::Matrix<T, 3, 3>& rotation) {
  const T theta = (rotation.trace() > 3.0 - 1e-6)
                      ? 0.0
                      : std::acos(0.5 * (rotation.trace() - 1));
  const Eigen::Matrix<T, 3, 1> skew_vector(
      rotation(2, 1) - rotation(1, 2), rotation(0, 2) - rotation(2, 0),
      rotation(1, 0) - rotation(0, 1));
  return (std::abs(theta) < 0.001)
             ? (0.5 * skew_vector)
             : (0.5 * theta / std::sin(theta) * skew_vector);
}

template <typename T>
Eigen::Matrix<T, 3, 1> RotationMatrixToEuler(
    const Eigen::Matrix<T, 3, 3>& rotation) {
  const T sy = std::sqrt(rotation(0, 0) * rotation(0, 0) +
                         rotation(1, 0) * rotation(1, 0));
  const bool singular = sy < 1e-6;
  T x = 0;
  T y = 0;
  T z = 0;
  if (!singular) {
    x = std::atan2(rotation(2, 1), rotation(2, 2));
    y = std::atan2(-rotation(2, 0), sy);
    z = std::atan2(rotation(1, 0), rotation(0, 0));
  } else {
    x = std::atan2(-rotation(1, 2), rotation(1, 1));
    y = std::atan2(-rotation(2, 0), sy);
    z = 0;
  }
  return Eigen::Matrix<T, 3, 1>(x, y, z);
}

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_COMMON_SO3_MATH_H_
