#ifndef SO3_MATH_H
#define SO3_MATH_H

#include <math.h>

#include "livo_atlas/core/so3_math.h"

// Compatibility entry points for the existing algorithm consumers.
#define SKEW_SYM_MATRX(v) \
  0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0

using livo_atlas::Exp;
using livo_atlas::Log;

template <typename T>
Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3>& rotation) {
  return livo_atlas::RotationMatrixToEuler(rotation);
}

#endif  // SO3_MATH_H
