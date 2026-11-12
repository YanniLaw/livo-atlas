#include "livo_atlas/backend/loop_closure.h"

#include <limits>
#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>

#include "backend/loop_fixtures.h"

namespace livo_atlas {
namespace {

TEST(LoopGeometryCheckTest, EvaluatesKnownFixedTransformInBothDirections) {
  const auto target = loop_test::Scene();
  const auto transform = loop_test::Transform();
  const auto source = loop_test::Scene(5, 100, transform);
  const auto result = EvaluateLoopGeometry(target, source, transform);
  ASSERT_EQ(result.status, LoopStatus::kVerified);
  for (const auto* metrics : {&result.forward, &result.reverse}) {
    EXPECT_EQ(metrics->inliers, 300u);
    EXPECT_EQ(metrics->normal_pairs, 300u);
    EXPECT_DOUBLE_EQ(metrics->source_coverage, 1);
    EXPECT_DOUBLE_EQ(metrics->target_coverage, 1);
    ASSERT_TRUE(metrics->rmse_m);
    EXPECT_LT(*metrics->rmse_m, 1e-12);
  }
}

TEST(LoopGeometryCheckTest, DoesNotRefitAnIncorrectMeasurement) {
  const auto target = loop_test::Scene();
  auto source = loop_test::Scene(5, 100);
  RigidTransform wrong;
  wrong.translation.x() = 0.15;
  source.parent_from_submap = wrong;
  LoopRegistrationOptions options;
  options.voxel_size_m = 0.2;
  options.max_rmse_m = 0.05;
  options.max_p95_m = 0.05;
  EXPECT_EQ(VerifyLoop(target, source, options).status, LoopStatus::kVerified);
  const auto result = EvaluateLoopGeometry(target, source, wrong, options);
  ASSERT_EQ(result.status, LoopStatus::kHighResidual);
  ASSERT_TRUE(result.forward.rmse_m);
  EXPECT_NEAR(*result.forward.rmse_m, 0.15, 1e-10);
}

TEST(LoopGeometryCheckTest, RejectsPlanarDegeneracyAndUnequalCoverage) {
  LoopRegistrationOptions options;
  options.voxel_size_m = 0.2;
  EXPECT_EQ(
      EvaluateLoopGeometry(loop_test::Scene(0, 0, {}, true),
                           loop_test::Scene(5, 100, {}, true), {}, options)
          .status,
      LoopStatus::kDegenerate);
  const auto target = loop_test::Scene();
  auto source = loop_test::Scene(5, 100);
  auto subset = std::make_shared<GeometryData>(*source.data);
  subset->voxels.resize(150);
  source.data = subset;
  options.min_coverage = 0.75;
  const auto result = EvaluateLoopGeometry(target, source, {}, options);
  EXPECT_EQ(result.status, LoopStatus::kLowOverlap);
  EXPECT_DOUBLE_EQ(result.forward.source_coverage, 1);
  EXPECT_DOUBLE_EQ(result.forward.target_coverage, 0.5);
  EXPECT_DOUBLE_EQ(result.reverse.source_coverage, 0.5);
  EXPECT_DOUBLE_EQ(result.reverse.target_coverage, 1);
}

TEST(LoopGeometryCheckTest, EnforcesGeometryAndCoarseBudgets) {
  const auto target = loop_test::Scene();
  auto source = loop_test::Scene(5, 100);
  LoopRegistrationOptions options;
  options.max_voxels = 299;
  EXPECT_EQ(EvaluateLoopGeometry(target, source, {}, options).status,
            LoopStatus::kResourceLimit);
  options = {};
  options.voxel_size_m = 0.2;
  options.max_points = 299;
  EXPECT_EQ(EvaluateLoopGeometry(target, source, {}, options).status,
            LoopStatus::kResourceLimit);
  options = {};
  auto empty = std::make_shared<GeometryData>(*source.data);
  empty->voxels.clear();
  source.data = empty;
  EXPECT_EQ(EvaluateLoopGeometry(target, source, {}, options).status,
            LoopStatus::kInsufficientPoints);
}

TEST(LoopGeometryCheckTest, RejectsInvalidAndUnsafeCoordinatesAndTransforms) {
  const auto target = loop_test::Scene();
  for (double value : {std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity(), 1e20}) {
    auto source = loop_test::Scene(5, 100);
    auto data = std::make_shared<GeometryData>(*source.data);
    data->voxels.front().centroid.x() = value;
    source.data = data;
    EXPECT_THROW(EvaluateLoopGeometry(target, source, {}),
                 std::invalid_argument);
    RigidTransform transform;
    transform.translation.x() = value;
    EXPECT_THROW(
        EvaluateLoopGeometry(target, loop_test::Scene(5, 100), transform),
        std::invalid_argument);
  }
  RigidTransform invalid;
  invalid.rotation.setZero();
  EXPECT_THROW(EvaluateLoopGeometry(target, loop_test::Scene(5, 100), invalid),
               std::invalid_argument);
}

TEST(LoopGeometryCheckTest, RejectsMixedViewsAndInvalidSources) {
  const auto target = loop_test::Scene();
  auto source = loop_test::Scene(5, 100);
  source.frame_count = 201;
  EXPECT_THROW(EvaluateLoopGeometry(target, source, {}), std::invalid_argument);
  source = loop_test::Scene(5, 100);
  auto data = std::make_shared<GeometryData>(*source.data);
  data->sources.front().keyframe_id = 99;
  source.data = data;
  EXPECT_THROW(EvaluateLoopGeometry(target, source, {}), std::invalid_argument);
  EXPECT_THROW(EvaluateLoopGeometry(source, target, {}), std::invalid_argument);
}

}  // namespace
}  // namespace livo_atlas
