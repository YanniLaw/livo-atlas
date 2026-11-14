#include "livo_atlas/mapping/map_correction.h"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace livo_atlas {
namespace {

MapCorrection OptimizedCorrection() {
  MapCorrection correction;
  correction.pose_version = 2;
  correction.frame_count = 7;
  correction.optimized_frame_count = 5;
  correction.map_from_odom.rotation << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  correction.map_from_odom.translation << 1, 2, 3;
  return correction;
}

TEST(MapCorrectionTest, IdentityBeforeAnyKeyframe) {
  RigidTransform published;
  published.rotation << 1, 0, 0, 0, 0, -1, 0, 1, 0;
  published.translation << 4, 5, 6;
  MapCorrection correction;
  for (auto count : {0u, 4u}) {
    correction.frame_count = count;
    const auto global = ApplyMapCorrection(correction, published);
    EXPECT_TRUE((global.rotation.array() == published.rotation.array()).all());
    EXPECT_TRUE(
        (global.translation.array() == published.translation.array()).all());
  }
}

TEST(MapCorrectionTest, ComposesNoncommutingRigidTransforms) {
  auto correction = OptimizedCorrection();
  RigidTransform published;
  published.rotation << 1, 0, 0, 0, 0, -1, 0, 1, 0;
  published.translation << 4, 5, 6;
  const auto global = ApplyMapCorrection(correction, published);
  Eigen::Matrix3d expected_rotation;
  expected_rotation << 0, 0, 1, 1, 0, 0, 0, 1, 0;
  const Eigen::Vector3d expected_translation(-4, 6, 9);
  EXPECT_TRUE((global.rotation.array() == expected_rotation.array()).all());
  EXPECT_TRUE(
      (global.translation.array() == expected_translation.array()).all());
  EXPECT_DOUBLE_EQ(correction.map_from_odom.translation.x(), 1);
  EXPECT_DOUBLE_EQ(published.translation.x(), 4);
  EXPECT_DOUBLE_EQ(correction.map_from_odom.rotation(0, 1), -1);
  EXPECT_DOUBLE_EQ(published.rotation(1, 2), -1);
}

TEST(MapCorrectionTest, RejectsInvalidHeadersAndTransforms) {
  RigidTransform published;
  auto correction = OptimizedCorrection();
  correction.optimized_frame_count = 0;
  EXPECT_THROW(ApplyMapCorrection(correction, published),
               std::invalid_argument);
  correction.optimized_frame_count = 8;
  EXPECT_THROW(ApplyMapCorrection(correction, published),
               std::invalid_argument);
  correction = {};
  correction.frame_count = correction.optimized_frame_count = 1;
  EXPECT_THROW(ApplyMapCorrection(correction, published),
               std::invalid_argument);
  correction = {};
  correction.map_from_odom.translation.x() = 1;
  EXPECT_THROW(ApplyMapCorrection(correction, published),
               std::invalid_argument);
  correction.map_from_odom = OptimizedCorrection().map_from_odom;
  correction.map_from_odom.translation.setZero();
  EXPECT_THROW(ApplyMapCorrection(correction, published),
               std::invalid_argument);

  correction = OptimizedCorrection();
  correction.map_from_odom.rotation(0, 0) =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(ApplyMapCorrection(correction, published),
               std::invalid_argument);
  correction = OptimizedCorrection();
  correction.map_from_odom.translation.z() =
      std::numeric_limits<double>::infinity();
  EXPECT_THROW(ApplyMapCorrection(correction, published),
               std::invalid_argument);
  correction = OptimizedCorrection();
  published.rotation(0, 0) = 1.000045;
  EXPECT_THROW(ApplyMapCorrection(correction, published),
               std::invalid_argument);
  published.rotation(0, 0) = -1;
  EXPECT_THROW(ApplyMapCorrection(correction, published),
               std::invalid_argument);
  published = {};
  published.translation.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(ApplyMapCorrection(correction, published),
               std::invalid_argument);
}

TEST(MapCorrectionTest, AcceptsRepeatedOptimizationOfSamePrefix) {
  MapCorrection correction;
  correction.pose_version = 100;
  correction.frame_count = correction.optimized_frame_count = 1;
  const auto global = ApplyMapCorrection(correction, RigidTransform{});
  EXPECT_DOUBLE_EQ(global.translation.squaredNorm(), 0);
  EXPECT_TRUE(global.rotation.isIdentity());
}

TEST(MapCorrectionTest, RejectsCompositionOverflow) {
  MapCorrection correction;
  correction.pose_version = correction.frame_count =
      correction.optimized_frame_count = 1;
  correction.map_from_odom.translation.x() = std::numeric_limits<double>::max();
  RigidTransform published;
  published.translation.x() = std::numeric_limits<double>::max();
  EXPECT_THROW(ApplyMapCorrection(correction, published),
               std::invalid_argument);
}

}  // namespace
}  // namespace livo_atlas
