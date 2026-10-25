#include "livo_atlas/common/filter_state.h"

#include <array>
#include <cmath>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

namespace livo_atlas {
namespace {

FilterState MakePopulatedState() {
  FilterState state;
  state.rot_end =
      Eigen::AngleAxisd(0.7, Vector3d(1, 2, 3).normalized()).toRotationMatrix();
  state.pos_end = Vector3d(1, 2, 3);
  state.vel_end = Vector3d(4, 5, 6);
  state.inv_expo_time = 1.5;
  state.bias_g = Vector3d(0.01, 0.02, 0.03);
  state.bias_a = Vector3d(0.1, 0.2, 0.3);
  state.gravity = Vector3d(0, 0, -9.81);
  state.cov = StateCovariance::Identity() * 0.2;
  state.cov(1, 5) = 0.002;
  state.cov(5, 1) = 0.002;
  return state;
}

void ExpectSameState(const FilterState& actual, const FilterState& expected) {
  EXPECT_TRUE(actual.rot_end.isApprox(expected.rot_end, 0));
  EXPECT_TRUE(actual.pos_end.isApprox(expected.pos_end, 0));
  EXPECT_TRUE(actual.vel_end.isApprox(expected.vel_end, 0));
  EXPECT_DOUBLE_EQ(actual.inv_expo_time, expected.inv_expo_time);
  EXPECT_TRUE(actual.bias_g.isApprox(expected.bias_g, 0));
  EXPECT_TRUE(actual.bias_a.isApprox(expected.bias_a, 0));
  EXPECT_TRUE(actual.gravity.isApprox(expected.gravity, 0));
  EXPECT_TRUE(actual.cov.isApprox(expected.cov, 0));
}

TEST(StateTest, InitializesStateAndCovariance) {
  const FilterState state;
  EXPECT_TRUE(state.rot_end.isIdentity(0));
  EXPECT_TRUE(state.pos_end.isZero(0));
  EXPECT_TRUE(state.vel_end.isZero(0));
  EXPECT_TRUE(state.bias_g.isZero(0));
  EXPECT_TRUE(state.bias_a.isZero(0));
  EXPECT_TRUE(state.gravity.isZero(0));
  EXPECT_DOUBLE_EQ(state.inv_expo_time, 1);
  ASSERT_EQ(state.cov.rows(), 19);
  for (int row = 0; row < 19; ++row) {
    for (int column = 0; column < 19; ++column) {
      const double expected =
          row != column ? 0 : ((row == 6 || row >= 10) ? 0.00001 : 0.01);
      EXPECT_DOUBLE_EQ(state.cov(row, column), expected)
          << "at " << row << ", " << column;
    }
  }
}

TEST(StateTest, PreservesNineteenStateIndices) {
  for (int index = 0; index < 19; ++index) {
    SCOPED_TRACE(index);
    StateIncrement increment = StateIncrement::Zero();
    increment[index] = 0.01;
    const FilterState original;
    const FilterState result = original + increment;
    Matrix3d expected_rotation = Matrix3d::Identity();
    if (index < 3) {
      expected_rotation =
          Eigen::AngleAxisd(0.01, Vector3d::Unit(index)).toRotationMatrix();
    }
    EXPECT_TRUE(result.rot_end.isApprox(expected_rotation, 1e-12));
    const std::array<double, 16> values = {
        result.pos_end.x(), result.pos_end.y(), result.pos_end.z(),
        result.inv_expo_time - 1, result.vel_end.x(), result.vel_end.y(),
        result.vel_end.z(), result.bias_g.x(), result.bias_g.y(),
        result.bias_g.z(), result.bias_a.x(), result.bias_a.y(),
        result.bias_a.z(), result.gravity.x(), result.gravity.y(),
        result.gravity.z()};
    for (int component = 0; component < 16; ++component) {
      EXPECT_NEAR(values[component], index == component + 3 ? 0.01 : 0,
                  1e-12);
    }
    FilterState in_place = original;
    in_place += increment;
    ExpectSameState(in_place, result);
  }
}

TEST(StateTest, RightMultipliesRotation) {
  FilterState original;
  original.rot_end << 1, 0, 0, 0, 0, -1, 0, 1, 0;
  StateIncrement increment = StateIncrement::Zero();
  increment[1] = 0.25;
  Matrix3d expected;
  expected << std::cos(0.25), 0, std::sin(0.25), std::sin(0.25), 0,
      -std::cos(0.25), 0, 1, 0;
  EXPECT_TRUE((original + increment).rot_end.isApprox(expected, 1e-12));
  original += increment;
  EXPECT_TRUE(original.rot_end.isApprox(expected, 1e-12));
}

TEST(StateTest, DifferenceRecoversIncrement) {
  const FilterState original = MakePopulatedState();
  StateIncrement increment;
  for (int index = 0; index < 19; ++index) {
    increment[index] = 0.01 * (index + 1);
  }
  const FilterState updated = original + increment;
  EXPECT_TRUE((updated - original).isApprox(increment, 1e-12));
  EXPECT_TRUE((original - original).isZero(0));
}

TEST(StateTest, PreservesCovarianceAndCopiesValues) {
  const FilterState original = MakePopulatedState();
  FilterState copy(original);
  FilterState assigned;
  assigned = original;
  ExpectSameState(copy, original);
  ExpectSameState(assigned, original);
  const StateIncrement increment = StateIncrement::Constant(0.1);
  EXPECT_TRUE((original + increment).cov.isApprox(original.cov, 0));
  assigned += increment;
  EXPECT_TRUE(assigned.cov.isApprox(original.cov, 0));
  copy.pos_end.x() = 100;
  copy.cov(1, 5) = 0.5;
  EXPECT_DOUBLE_EQ(original.pos_end.x(), 1);
  EXPECT_DOUBLE_EQ(original.cov(1, 5), 0.002);
}

TEST(StateTest, ResetPosePreservesCalibrationAndCovariance) {
  FilterState state = MakePopulatedState();
  const FilterState original = state;
  state.ResetPose();
  EXPECT_TRUE(state.rot_end.isIdentity(0));
  EXPECT_TRUE(state.pos_end.isZero(0));
  EXPECT_TRUE(state.vel_end.isZero(0));
  EXPECT_DOUBLE_EQ(state.inv_expo_time, 1.5);
  EXPECT_TRUE(state.bias_g.isApprox(Vector3d(0.01, 0.02, 0.03), 0));
  EXPECT_TRUE(state.bias_a.isApprox(Vector3d(0.1, 0.2, 0.3), 0));
  EXPECT_TRUE(state.gravity.isApprox(Vector3d(0, 0, -9.81), 0));
  EXPECT_TRUE(state.cov.isApprox(original.cov, 0));
}

}  // namespace
}  // namespace livo_atlas
