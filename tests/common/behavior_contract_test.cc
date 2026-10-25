#include <gtest/gtest.h>

#include <cmath>

#include "livo_atlas/common/filter_state.h"
#include "livo_atlas/common/math_types.h"
#include "livo_atlas/common/so3_math.h"

namespace livo_atlas {
namespace {

TEST(SharedMathContractTest, PreservesEulerConvention) {
  Matrix3d rotation;
  rotation << std::cos(0.25), -std::sin(0.25), 0, std::sin(0.25),
      std::cos(0.25), 0, 0, 0, 1;
  EXPECT_TRUE(livo_atlas::RotationMatrixToEuler(rotation).isApprox(
      Vector3d(0, 0, 0.25), 1e-12));
}

TEST(SharedMathContractTest, ResetPreservesCalibrationAndCovariance) {
  FilterState state;
  state.pos_end = Vector3d(1, 2, 3);
  state.vel_end = Vector3d(4, 5, 6);
  state.gravity = Vector3d(0, 0, -9.81);
  state.inv_expo_time = 2;
  state.cov(1, 5) = 0.002;
  state.ResetPose();
  EXPECT_TRUE(state.rot_end.isIdentity(0));
  EXPECT_TRUE(state.pos_end.isZero(0));
  EXPECT_TRUE(state.vel_end.isZero(0));
  EXPECT_TRUE(state.gravity.isApprox(Vector3d(0, 0, -9.81), 0));
  EXPECT_DOUBLE_EQ(state.inv_expo_time, 2);
  EXPECT_DOUBLE_EQ(state.cov(1, 5), 0.002);
}

}  // namespace
}  // namespace livo_atlas
