#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "backend/loop_fixtures.h"
#include "livo_atlas/backend/loop_closure.h"

namespace livo_atlas {
namespace {

TEST(LoopRegistrationTest, RecoversKnownTransformWithoutChangingInputs) {
  const auto target = loop_test::Scene();
  const auto truth = loop_test::Transform();
  auto source = loop_test::Scene(5, 100, truth);
  source.parent_from_submap = truth;
  source.parent_from_submap.translation.x() += 0.05;
  const auto before = source.data->voxels.front().centroid;
  const auto result = VerifyLoop(target, source);
  ASSERT_EQ(result.status, LoopStatus::kVerified);
  ASSERT_TRUE(result.target_from_source);
  EXPECT_LT((result.target_from_source->translation - truth.translation).norm(),
            0.02);
  EXPECT_TRUE(
      result.target_from_source->rotation.isApprox(truth.rotation, 0.003));
  EXPECT_TRUE(source.data->voxels.front().centroid == before);
  EXPECT_EQ(result.source.anchor_keyframe_id, 100u);
}

TEST(LoopRegistrationTest, RejectsIterationLimitEvenIfPclReportsConverged) {
  const auto target = loop_test::Scene();
  auto source = loop_test::Scene(5, 100, loop_test::Transform());
  LoopRegistrationOptions options;
  options.max_iterations = 1;
  const auto result = VerifyLoop(target, source, options);
  EXPECT_EQ(result.status, LoopStatus::kIterationLimit);
  EXPECT_FALSE(result.target_from_source);
}

TEST(LoopRegistrationTest, ValidatesViewAndGeometryBeforeSolving) {
  const auto target = loop_test::Scene();
  auto source = loop_test::Scene(5, 100);
  source.pose_version = 1;
  source.optimized_frame_count = 150;
  source.parent_frame = PoseFrame::kMap;
  EXPECT_THROW(VerifyLoop(target, source), std::invalid_argument);
  auto corrected_target = target;
  corrected_target.pose_version = 1;
  corrected_target.optimized_frame_count = 150;
  corrected_target.parent_frame = PoseFrame::kMap;
  EXPECT_EQ(VerifyLoop(corrected_target, source).status, LoopStatus::kVerified);
  auto bad = std::make_shared<GeometryData>(*source.data);
  bad->sources[0].keyframe_id = 99;
  source.data = bad;
  EXPECT_THROW(VerifyLoop(corrected_target, source), std::invalid_argument);
  bad->sources[0].keyframe_id = 100;
  bad->voxels[0].centroid.x() = std::numeric_limits<double>::infinity();
  EXPECT_THROW(VerifyLoop(corrected_target, source), std::invalid_argument);
}

TEST(LoopRegistrationTest, RefusesInsufficientAndExcessGeometry) {
  const auto target = loop_test::Scene();
  auto source = loop_test::Scene(5, 100);
  auto small = std::make_shared<GeometryData>(*source.data);
  small->voxels.resize(5);
  source.data = small;
  EXPECT_EQ(VerifyLoop(target, source).status, LoopStatus::kInsufficientPoints);
  source = loop_test::Scene(5, 100);
  LoopRegistrationOptions options;
  options.max_voxels = 200;
  EXPECT_EQ(VerifyLoop(target, source, options).status,
            LoopStatus::kResourceLimit);
}

TEST(LoopRegistrationTest, RejectsLowResidualPlaneAsDegenerate) {
  const auto target = loop_test::Scene(0, 0, {}, true);
  auto source = loop_test::Scene(5, 100, {}, true);
  LoopRegistrationOptions options;
  options.voxel_size_m = 0.2;
  const auto result = VerifyLoop(target, source, options);
  EXPECT_EQ(result.status, LoopStatus::kDegenerate);
  EXPECT_FALSE(result.target_from_source);
}

TEST(LoopRegistrationTest, HandlesPartialOverlapAndRejectsInvalidNormals) {
  const auto target = loop_test::Scene();
  const auto truth = loop_test::Transform();
  auto source = loop_test::Scene(5, 100, truth);
  source.parent_from_submap = truth;
  auto data = std::make_shared<GeometryData>(*source.data);
  data->voxels.resize(240);
  source.data = data;
  EXPECT_EQ(VerifyLoop(target, source).status, LoopStatus::kVerified);
  data->voxels[0].normal = Eigen::Vector3d(2, 0, 0);
  EXPECT_THROW(VerifyLoop(target, source), std::invalid_argument);
}

TEST(LoopRegistrationTest, OutliersDoNotBecomeCorrespondences) {
  const auto target = loop_test::Scene();
  auto source = loop_test::Scene(5, 100);
  auto data = std::make_shared<GeometryData>(*source.data);
  for (int i = 0; i < 30; ++i) {
    auto voxel = data->voxels.front();
    voxel.centroid = {50.0 + i, 50, 50};
    voxel.key = {1000 + i, 1000, 1000};
    data->voxels.push_back(voxel);
  }
  source.data = data;
  const auto result = VerifyLoop(target, source);
  ASSERT_EQ(result.status, LoopStatus::kVerified);
  ASSERT_TRUE(result.forward.metrics);
  EXPECT_EQ(result.forward.metrics->inliers, 300u);
  EXPECT_LT(result.forward.metrics->source_coverage, 1);
}

TEST(LoopRegistrationTest, FloorsNegativeCoordinatesAndChecksCoarseBudget) {
  auto target = loop_test::Scene();
  auto source = loop_test::Scene(5, 100);
  auto few = std::make_shared<GeometryData>(*target.data);
  few->voxels.resize(2);
  few->voxels[0].centroid = {-0.01, 0, 0};
  few->voxels[0].key = {-1, 0, 0};
  few->voxels[1].centroid = {0.01, 0, 0};
  few->voxels[1].key = {0, 0, 0};
  target.data = few;
  const auto result = VerifyLoop(target, source);
  EXPECT_EQ(result.status, LoopStatus::kInsufficientPoints);
  EXPECT_EQ(result.target_points, 2u);
  target = loop_test::Scene();
  LoopRegistrationOptions options;
  options.max_points = 100;
  EXPECT_EQ(VerifyLoop(target, source, options).status,
            LoopStatus::kResourceLimit);
}

}  // namespace
}  // namespace livo_atlas
