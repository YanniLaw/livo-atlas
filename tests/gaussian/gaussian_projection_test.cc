#include "livo_atlas/gaussian/gaussian_projection.h"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

namespace livo_atlas {
namespace {

GaussianCamera Camera() {
  GaussianCamera camera;
  camera.width = 100;
  camera.height = 80;
  camera.K << 100, 0, 49.5, 0, 100, 39.5, 0, 0, 1;
  return camera;
}

GaussianExtent Point(double x = 0, double y = 0, double z = 4) {
  return {0, {x, y, z}, {x, y, z}, .03};
}

TEST(GaussianProjection, OriginalPixelCentersAreConvertedExactlyOnce) {
  const auto result = ProjectGaussianExtents({Point()}, Camera(), {}, 0);
  // Radius6 plus outward FP32 padding gives [43,57) x [33,47).
  EXPECT_EQ(result.candidate_pixels, 196u);
  EXPECT_EQ(result.distance, 4);
  EXPECT_FALSE(result.uncertain);
  auto shifted = Camera();
  shifted.K(0, 2) -= .5;
  shifted.K(1, 2) -= .5;
  EXPECT_EQ(ProjectGaussianExtents({Point()}, shifted, {}, 0)
                .candidate_pixels, 169u);
}

TEST(GaussianProjection, OverlappingOriginsUseRectangleUnion) {
  auto second = Point(.4);
  second.origin_keyframe_id = 1;
  const auto result =
      ProjectGaussianExtents({Point(), second}, Camera(), {}, 0);
  EXPECT_EQ(result.candidate_pixels, 336u);
  EXPECT_EQ(result.distance, 4);
}

TEST(GaussianProjection, CenterDepthClippingKeepsCrossingNearPlane) {
  EXPECT_EQ(ProjectGaussianExtents({Point(0, 0, -4)}, Camera(), {}, 0)
                .candidate_pixels, 0u);
  EXPECT_EQ(ProjectGaussianExtents({Point(0, 0, .005)}, Camera(), {}, 0)
                .candidate_pixels, 0u);
  EXPECT_EQ(ProjectGaussianExtents({Point(0, 0, 1e11)}, Camera(), {}, 0)
                .candidate_pixels, 0u);
  auto crossing = Point();
  crossing.min = {-1, -1, -1};
  crossing.max = {1, 1, 2};
  const auto result = ProjectGaussianExtents({crossing}, Camera(), {}, 0);
  EXPECT_EQ(result.candidate_pixels, 8000u);
  EXPECT_FALSE(result.uncertain);
  crossing.max = {1, 1, 1};
  EXPECT_EQ(ProjectGaussianExtents({crossing}, Camera(), {}, 0)
                .candidate_pixels, 8000u);
}

TEST(GaussianProjection, PixelEdgesIncludeFootprintsBeyondTheCenter) {
  EXPECT_GT(ProjectGaussianExtents({Point(2)}, Camera(), {}, 0)
                .candidate_pixels, 0u);
  EXPECT_EQ(ProjectGaussianExtents({Point(2.4)}, Camera(), {}, 0)
                .candidate_pixels, 0u);
  EXPECT_EQ(ProjectGaussianExtents({Point(-2.4)}, Camera(), {}, 0)
                .candidate_pixels, 0u);
}

TEST(GaussianProjection, RotationAndMaximumAxisScaleRemainConservative) {
  RigidTransform rotated;
  rotated.rotation << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  const auto result =
      ProjectGaussianExtents({Point(1)}, Camera(), rotated, 0);
  EXPECT_EQ(result.candidate_pixels, 196u);
  EXPECT_DOUBLE_EQ(result.distance, std::sqrt(17));
  auto large = Point();
  large.max_scale = .2;
  EXPECT_GT(ProjectGaussianExtents({large}, Camera(), rotated, 0)
                .candidate_pixels, result.candidate_pixels);
}

TEST(GaussianProjection, NumericUncertaintyKeepsTheWholeCanvas) {
  auto camera = Camera();
  camera.K(0, 0) = 1e308;
  auto result = ProjectGaussianExtents({Point()}, camera, {}, 0);
  EXPECT_TRUE(result.uncertain);
  EXPECT_EQ(result.candidate_pixels, 8000u);
  result = ProjectGaussianExtents({Point(1e300)}, Camera(), {}, 0);
  EXPECT_TRUE(result.uncertain);
  EXPECT_EQ(result.candidate_pixels, 8000u);
  result = ProjectGaussianExtents({Point()}, Camera(), {},
                                   std::numeric_limits<double>::infinity());
  EXPECT_TRUE(result.uncertain);
  EXPECT_EQ(result.candidate_pixels, 8000u);
}

TEST(GaussianProjection, LargeParentCoordinatesCannotCullByCancellation) {
  const auto result = ProjectGaussianExtents({Point()}, Camera(), {}, 1e8);
  EXPECT_EQ(result.candidate_pixels, 8000u);
}

TEST(GaussianProjection, MalformedInputsFailBeforeProjection) {
  auto extent = Point();
  extent.max_scale = 0;
  EXPECT_THROW(ProjectGaussianExtents({extent}, Camera(), {}, 0),
               std::invalid_argument);
  extent = Point();
  extent.min.x() = 1;
  EXPECT_THROW(ProjectGaussianExtents({extent}, Camera(), {}, 0),
               std::invalid_argument);
  auto camera = Camera();
  camera.K(0, 0) = -1;
  EXPECT_THROW(ProjectGaussianExtents({Point()}, camera, {}, 0),
               std::invalid_argument);
  EXPECT_THROW(ProjectGaussianExtents({}, Camera(), {}, 0),
               std::invalid_argument);
  EXPECT_THROW(ProjectGaussianExtents(std::vector<GaussianExtent>(11, Point()),
                                      Camera(), {}, 0),
               std::invalid_argument);
  EXPECT_THROW(ProjectGaussianExtents({Point()}, Camera(), {}, -1),
               std::invalid_argument);
}

}  // namespace
}  // namespace livo_atlas
