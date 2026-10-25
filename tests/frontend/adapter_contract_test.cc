#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "livo_atlas/frontend/imu/imu_propagation.h"
#include "livo_atlas/frontend/vision/image_options.h"

namespace livo_atlas {
namespace {
CameraConfig Camera() {
  CameraConfig camera;
  camera.model = "Pinhole";
  camera.width = 1280;
  camera.height = 1024;
  camera.scale = 0.5;
  camera.fx = camera.fy = 640;
  camera.cx = 320;
  camera.cy = 256;
  return camera;
}
TEST(ImageOptionsTest, AcceptsOriginalConfiguration) {
  EXPECT_TRUE(ValidateImageOptions(Camera(), VisualOptions{}).empty());
}
TEST(ImageOptionsTest, RejectsCameraBeforeIntegerConversion) {
  auto camera = Camera();
  camera.scale = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(ValidateImageOptions(camera, VisualOptions{}).empty());
}
TEST(ImageOptionsTest, RejectsInvalidGrids) {
  for (int grid : {0, -1, 100000}) {
    VisualOptions visual;
    visual.grid_size = grid;
    EXPECT_FALSE(ValidateImageOptions(Camera(), visual).empty());
  }
  for (int rows : {0, -1, 10000}) {
    VisualOptions visual;
    visual.grid_rows = rows;
    EXPECT_FALSE(ValidateImageOptions(Camera(), visual).empty());
  }
}
TEST(ImageOptionsTest, RejectsPatchAndPyramidBounds) {
  for (int patch : {0, -2, 7, 100000}) {
    VisualOptions visual;
    visual.patch_size = patch;
    EXPECT_FALSE(ValidateImageOptions(Camera(), visual).empty());
  }
  for (int levels : {0, -1, 29, 28}) {
    VisualOptions visual;
    visual.patch_pyramid_levels = levels;
    EXPECT_FALSE(ValidateImageOptions(Camera(), visual).empty());
  }
}
TEST(ImageOptionsTest, RejectsUncoveredTail) {
  VisualOptions visual;
  visual.grid_size = 100;
  visual.patch_size = 2;
  visual.patch_pyramid_levels = 1;
  EXPECT_FALSE(ValidateImageOptions(Camera(), visual).empty());
}
TEST(ImuPropagationTest, StationaryWithGravityAndBiases) {
  FilterState state;
  state.gravity.z() = -9.81;
  state.bias_a << 1, 2, 3;
  state.bias_g << 0.1, 0.2, 0.3;
  PropagateImu(&state, 0.5, Eigen::Vector3d(1, 2, 12.81), state.bias_g, 9.81);
  EXPECT_LT(state.pos_end.norm(), 1e-14);
  EXPECT_LT(state.vel_end.norm(), 1e-14);
  EXPECT_TRUE(state.rot_end.isIdentity());
}
TEST(ImuPropagationTest, UsesUpdatedRotationAndRetainsOtherState) {
  FilterState state;
  state.pos_end << 1, 2, 3;
  state.vel_end << 2, 0, 0;
  const auto covariance = state.cov;
  PropagateImu(&state, 0.5, Eigen::Vector3d(2, 0, 0),
               Eigen::Vector3d(0, 0, std::acos(-1.0)), 9.81);
  EXPECT_NEAR(state.pos_end.x(), 2, 1e-14);
  EXPECT_NEAR(state.pos_end.y(), 2.25, 1e-14);
  EXPECT_NEAR(state.vel_end.y(), 1, 1e-14);
  EXPECT_TRUE(state.cov == covariance);
  EXPECT_DOUBLE_EQ(state.inv_expo_time, 1);
}
TEST(ImuPropagationTest, IndependentStatesAndRepeatedSteps) {
  FilterState first, second;
  PropagateImu(&first, 0.5, Eigen::Vector3d(2, 0, 0), Eigen::Vector3d::Zero(),
               9.81);
  PropagateImu(&first, 0.5, Eigen::Vector3d(2, 0, 0), Eigen::Vector3d::Zero(),
               9.81);
  PropagateImu(&second, 1, Eigen::Vector3d(0, 4, 0), Eigen::Vector3d::Zero(),
               9.81);
  EXPECT_DOUBLE_EQ(first.pos_end.x(), 1);
  EXPECT_DOUBLE_EQ(first.vel_end.x(), 2);
  EXPECT_DOUBLE_EQ(second.pos_end.y(), 2);
  EXPECT_DOUBLE_EQ(second.pos_end.x(), 0);
}
}  // namespace
}  // namespace livo_atlas
