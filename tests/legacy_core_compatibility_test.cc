#include "livo_atlas/internal/legacy/common.h"

#include <cmath>
#include <type_traits>

#include <gtest/gtest.h>

#include "livo_atlas/common/filter_state.h"

namespace livo_atlas {
namespace {

static_assert(std::is_same<StatesGroup, FilterState>::value);
static_assert(std::is_same<V2F, Vector2f>::value);
static_assert(std::is_same<V2D, Vector2d>::value);
static_assert(std::is_same<V3D, Vector3d>::value);
static_assert(std::is_same<M3D, Matrix3d>::value);
static_assert(std::is_same<V3F, Vector3f>::value);
static_assert(std::is_same<M3F, Matrix3f>::value);
static_assert(DIM_STATE == kStateDimension);

TEST(LegacyCoreCompatibilityTest, PreservesLegacyEulerConversion) {
  M3D rotation;
  rotation << std::cos(0.25), -std::sin(0.25), 0, std::sin(0.25),
      std::cos(0.25), 0, 0, 0, 1;
  EXPECT_TRUE(::RotMtoEuler(rotation).isApprox(V3D(0, 0, 0.25), 1e-12));
}

TEST(LegacyCoreCompatibilityTest, PreservesLegacyResetScope) {
  StatesGroup state;
  state.pos_end = V3D(1, 2, 3);
  state.vel_end = V3D(4, 5, 6);
  state.gravity = V3D(0, 0, -9.81);
  state.inv_expo_time = 2;
  state.cov(1, 5) = 0.002;
  state.resetpose();
  EXPECT_TRUE(state.rot_end.isIdentity(0));
  EXPECT_TRUE(state.pos_end.isZero(0));
  EXPECT_TRUE(state.vel_end.isZero(0));
  EXPECT_TRUE(state.gravity.isApprox(V3D(0, 0, -9.81), 0));
  EXPECT_DOUBLE_EQ(state.inv_expo_time, 2);
  EXPECT_DOUBLE_EQ(state.cov(1, 5), 0.002);
}

}  // namespace
}  // namespace livo_atlas
