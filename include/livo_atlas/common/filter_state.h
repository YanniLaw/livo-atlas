/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIVO_ATLAS_COMMON_FILTER_STATE_H_
#define LIVO_ATLAS_COMMON_FILTER_STATE_H_

#include <Eigen/Core>

#include "livo_atlas/common/math_types.h"
#include "livo_atlas/common/so3_math.h"

namespace livo_atlas {

inline constexpr int kStateDimension = 19;
inline constexpr double kInitialCovariance = 0.01;
inline constexpr double kGravityMetersPerSecondSquared = 9.81;

using StateIncrement = Eigen::Matrix<double, kStateDimension, 1>;
using StateCovariance =
    Eigen::Matrix<double, kStateDimension, kStateDimension>;

// Shared FAST-LIVO2 state. Field names and order retain legacy compatibility.
struct FilterState {
  FilterState()
      : rot_end(Matrix3d::Identity()),
        pos_end(Vector3d::Zero()),
        vel_end(Vector3d::Zero()),
        inv_expo_time(1.0),
        bias_g(Vector3d::Zero()),
        bias_a(Vector3d::Zero()),
        gravity(Vector3d::Zero()),
        cov(StateCovariance::Identity() * kInitialCovariance) {
    cov(6, 6) = 0.00001;
    cov.block<9, 9>(10, 10) = Eigen::Matrix<double, 9, 9>::Identity() * 0.00001;
  }

  FilterState operator+(const StateIncrement& increment) const {
    FilterState result;
    result.rot_end = rot_end * Exp(increment(0, 0), increment(1, 0),
                                  increment(2, 0));
    result.pos_end = pos_end + increment.block<3, 1>(3, 0);
    result.inv_expo_time = inv_expo_time + increment(6, 0);
    result.vel_end = vel_end + increment.block<3, 1>(7, 0);
    result.bias_g = bias_g + increment.block<3, 1>(10, 0);
    result.bias_a = bias_a + increment.block<3, 1>(13, 0);
    result.gravity = gravity + increment.block<3, 1>(16, 0);
    result.cov = cov;
    return result;
  }

  FilterState& operator+=(const StateIncrement& increment) {
    rot_end = rot_end * Exp(increment(0, 0), increment(1, 0), increment(2, 0));
    pos_end += increment.block<3, 1>(3, 0);
    inv_expo_time += increment(6, 0);
    vel_end += increment.block<3, 1>(7, 0);
    bias_g += increment.block<3, 1>(10, 0);
    bias_a += increment.block<3, 1>(13, 0);
    gravity += increment.block<3, 1>(16, 0);
    return *this;
  }

  StateIncrement operator-(const FilterState& reference) const {
    StateIncrement result;
    const Matrix3d rotation_difference(reference.rot_end.transpose() * rot_end);
    result.block<3, 1>(0, 0) = Log(rotation_difference);
    result.block<3, 1>(3, 0) = pos_end - reference.pos_end;
    result(6, 0) = inv_expo_time - reference.inv_expo_time;
    result.block<3, 1>(7, 0) = vel_end - reference.vel_end;
    result.block<3, 1>(10, 0) = bias_g - reference.bias_g;
    result.block<3, 1>(13, 0) = bias_a - reference.bias_a;
    result.block<3, 1>(16, 0) = gravity - reference.gravity;
    return result;
  }

  // Reset only attitude, position, and velocity; retain calibration and cov.
  void ResetPose() {
    rot_end = Matrix3d::Identity();
    pos_end = Vector3d::Zero();
    vel_end = Vector3d::Zero();
  }

  // Compatibility entry point for existing FAST-LIVO2 callers.
  void resetpose() { ResetPose(); }

  Matrix3d rot_end;       // Attitude at the end of the LiDAR scan.
  Vector3d pos_end;       // Position in the world frame.
  Vector3d vel_end;       // Velocity in the world frame.
  double inv_expo_time;   // Inverse exposure time, without scale.
  Vector3d bias_g;        // Gyroscope bias.
  Vector3d bias_a;        // Accelerometer bias.
  Vector3d gravity;       // Gravity acceleration in the world frame.
  StateCovariance cov;   // State covariance.
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_COMMON_FILTER_STATE_H_
