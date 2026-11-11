#include "backend/loop_internal.h"

#include <cmath>

#include <gtest/gtest.h>

#include "backend/loop_fixtures.h"

namespace livo_atlas {
namespace {

TEST(LoopQualityTest, FullSceneHasIndependentCorrespondencesAndRank) {
  const auto scene = loop_test::Scene();
  const auto metrics =
      loop_internal::EvaluateMetrics(*scene.data, *scene.data, {}, {});
  EXPECT_EQ(metrics.inliers, 300u);
  EXPECT_DOUBLE_EQ(metrics.source_coverage, 1);
  EXPECT_DOUBLE_EQ(metrics.target_coverage, 1);
  ASSERT_TRUE(metrics.rmse_m);
  EXPECT_NEAR(*metrics.rmse_m, 0, 1e-6);
  ASSERT_TRUE(metrics.eigenvalue_ratio);
  EXPECT_GT(*metrics.eigenvalue_ratio, 1e-4);
}

TEST(LoopQualityTest, DuplicateSourcePointsCannotInflateCorrespondences) {
  auto target = loop_test::Scene();
  auto source = *target.data;
  source.voxels = {target.data->voxels[0], target.data->voxels[0],
                   target.data->voxels[0]};
  const auto metrics =
      loop_internal::EvaluateMetrics(*target.data, source, {}, {});
  EXPECT_EQ(metrics.inliers, 1u);
  EXPECT_NEAR(metrics.source_coverage, 1.0 / 3, 1e-12);
}

TEST(LoopQualityTest, UsesMetersAndNearestRankPercentile) {
  auto scene = *loop_test::Scene().data;
  auto moved = scene;
  for (auto& voxel : moved.voxels) voxel.centroid.x() += 0.6;
  LoopRegistrationOptions options;
  options.match_distance_m = 0.5;
  // A single correspondence isolates the radius from grid neighbors.
  scene.voxels.resize(1);
  moved.voxels.resize(1);
  EXPECT_EQ(loop_internal::EvaluateMetrics(scene, moved, {}, options).inliers,
            0u);
  moved.voxels[0].centroid.x() = scene.voxels[0].centroid.x() + 0.4;
  const auto metrics =
      loop_internal::EvaluateMetrics(scene, moved, {}, options);
  ASSERT_TRUE(metrics.p95_m);
  EXPECT_NEAR(*metrics.p95_m, 0.4, 1e-6);
}

TEST(LoopQualityTest, LowResidualPlaneAndMissingNormalsDoNotImplyRank) {
  const auto plane = loop_test::Scene(0, 0, {}, true);
  const auto metrics =
      loop_internal::EvaluateMetrics(*plane.data, *plane.data, {}, {});
  ASSERT_TRUE(metrics.eigenvalue_ratio);
  EXPECT_LT(*metrics.eigenvalue_ratio, 1e-10);
  auto no_normals = *plane.data;
  for (auto& voxel : no_normals.voxels) voxel.normal.reset();
  EXPECT_FALSE(loop_internal::EvaluateMetrics(no_normals, no_normals, {}, {})
                   .eigenvalue_ratio);
}

TEST(LoopQualityTest, KeepsBothOriginsAndInitialCorrectionDirections) {
  constexpr double kHalfPi = 1.5707963267948966;
  RigidTransform initial;
  initial.rotation =
      Eigen::AngleAxisd(kHalfPi, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  initial.translation = {10, 0, 0};
  RigidTransform forward;
  forward.rotation =
      initial.rotation *
      Eigen::AngleAxisd(kHalfPi, Eigen::Vector3d::UnitX()).toRotationMatrix();
  forward.translation = {8, 1, 3};
  RigidTransform reverse{initial.rotation.transpose(),
                         Eigen::Vector3d(0, 10, 0)};
  const auto errors = loop_internal::TransformErrors(initial, forward, reverse);
  EXPECT_NEAR(errors[0].translation_m, std::sqrt(234.0), 1e-12);
  EXPECT_NEAR(errors[1].translation_m, std::sqrt(14.0), 1e-12);
  EXPECT_NEAR(errors[2].translation_m, std::sqrt(14.0), 1e-12);
  EXPECT_NEAR(errors[3].translation_m, 0, 1e-12);
  EXPECT_NEAR(errors[0].rotation_deg, 90, 1e-10);
  EXPECT_NEAR(errors[2].rotation_deg, 90, 1e-10);
  EXPECT_NEAR(errors[3].rotation_deg, 0, 1e-10);
}

TEST(LoopQualityTest, UsesNearestRankForSeveralResidualsAndRejectsLineRank) {
  auto target = *loop_test::Scene().data;
  target.voxels.resize(21);
  for (std::size_t i = 0; i < target.voxels.size(); ++i) {
    target.voxels[i].centroid = {2.0 * i, 0, 0};
    target.voxels[i].normal = Eigen::Vector3d::UnitZ();
  }
  auto source = target;
  for (std::size_t i = 0; i < source.voxels.size(); ++i) {
    source.voxels[i].centroid.y() = 0.01 * i;
  }
  LoopRegistrationOptions options;
  options.min_normal_pairs = 10;
  const auto metrics =
      loop_internal::EvaluateMetrics(target, source, {}, options);
  ASSERT_EQ(metrics.inliers, 21u);
  ASSERT_TRUE(metrics.p95_m);
  EXPECT_NEAR(*metrics.p95_m, 0.19, 1e-12);
  ASSERT_TRUE(metrics.eigenvalue_ratio);
  EXPECT_LT(*metrics.eigenvalue_ratio, 1e-10);
}

TEST(LoopQualityTest, ClassifiesBothDirectionsBeforeAcceptingQuality) {
  LoopMetrics good;
  good.inliers = good.normal_pairs = 300;
  good.source_coverage = good.target_coverage = 1;
  good.rmse_m = good.p95_m = 0.01;
  good.eigenvalue_ratio = 0.1;
  auto forward = good, reverse = good;
  std::array<LoopTransformError, 4> errors{};
  EXPECT_EQ(loop_internal::ClassifyQuality(forward, reverse, errors, {}),
            LoopStatus::kVerified);
  forward.rmse_m = 1;
  reverse.source_coverage = 0;
  EXPECT_EQ(loop_internal::ClassifyQuality(forward, reverse, errors, {}),
            LoopStatus::kLowOverlap);
  reverse = good;
  EXPECT_EQ(loop_internal::ClassifyQuality(forward, reverse, errors, {}),
            LoopStatus::kHighResidual);
  forward = good;
  errors[1].translation_m = 0.11;
  EXPECT_EQ(loop_internal::ClassifyQuality(forward, reverse, errors, {}),
            LoopStatus::kInconsistentTransform);
  errors = {};
  errors[3].rotation_deg = 21;
  EXPECT_EQ(loop_internal::ClassifyQuality(forward, reverse, errors, {}),
            LoopStatus::kLargeCorrection);
  errors = {};
  reverse.eigenvalue_ratio = 0;
  EXPECT_EQ(loop_internal::ClassifyQuality(forward, reverse, errors, {}),
            LoopStatus::kDegenerate);
}

}  // namespace
}  // namespace livo_atlas
