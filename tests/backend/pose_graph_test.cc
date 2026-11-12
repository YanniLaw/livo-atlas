#include "livo_atlas/backend/pose_graph.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

namespace livo_atlas {
namespace {

RigidTransform Pose(const Eigen::Matrix3d& rotation, double x, double y,
                    double z) {
  return {rotation, Eigen::Vector3d(x, y, z)};
}

PoseSnapshot RawSnapshot(const std::vector<RigidTransform>& poses) {
  return MaterializePoseSnapshot({}, poses, poses.size());
}

PoseGraph MakeGraph(const std::vector<RigidTransform>& poses,
                    PoseGraphOptions options = {}) {
  PoseGraph graph(options);
  for (std::uint64_t i = 0; i < poses.size(); ++i) {
    graph.AddKeyframe(i, poses[i]);
  }
  return graph;
}

std::vector<RigidTransform> Square() {
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  return {Pose(identity, 0, 0, 0), Pose(identity, 1, 0, 0),
          Pose(identity, 1, 1, 0), Pose(identity, 0, 1, 0),
          Pose(identity, 0.4, 0, 0)};
}

LoopConstraint ClosingLoop() { return {0, 4, {}, 0.01, 0.01}; }

TEST(PoseGraphTest, OdometryDirectionAndAnchor) {
  constexpr double kHalfPi = 1.5707963267948966;
  const Eigen::Matrix3d rz =
      Eigen::AngleAxisd(kHalfPi, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Matrix3d rx =
      Eigen::AngleAxisd(kHalfPi, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const std::vector<RigidTransform> raw = {Pose(rz, 2, 3, 1),
                                           Pose(rz * rx, 2, 4, 1)};
  const auto snapshot = RawSnapshot(raw);
  const auto result = MakeGraph(raw).Optimize(snapshot);

  ASSERT_EQ(result.update.poses.size(), 2u);
  EXPECT_EQ(result.update.base_version, 0u);
  EXPECT_EQ(result.update.optimized_frame_count, 2u);
  EXPECT_EQ(result.update.tail_policy,
            PoseTailPolicy::kProjectedLastOptimizedAnchor);
  EXPECT_NEAR(result.initial_error, 0, 1e-18);
  EXPECT_NEAR(result.final_error, 0, 1e-18);
  for (std::size_t i = 0; i < raw.size(); ++i) {
    EXPECT_TRUE(result.update.poses[i].map_from_body.rotation.isApprox(
        raw[i].rotation, 1e-8));
    EXPECT_TRUE(result.update.poses[i].map_from_body.translation.isApprox(
        raw[i].translation, 1e-8));
  }
}

TEST(PoseGraphTest, SyntheticLoopReducesResidual) {
  const auto raw = Square();
  const auto snapshot = RawSnapshot(raw);
  auto graph = MakeGraph(raw);
  graph.AddLoopConstraint(ClosingLoop());
  const auto result = graph.Optimize(snapshot);

  ASSERT_EQ(result.update.poses.size(), raw.size());
  EXPECT_GT(result.initial_error, result.final_error);
  EXPECT_FALSE(result.iteration_limit_reached);
  const auto& first = result.update.poses.front().map_from_body;
  const auto& last = result.update.poses.back().map_from_body;
  EXPECT_LT(first.translation.norm(), 1e-3);
  EXPECT_LT((last.translation - first.translation).norm(), 0.02);
  EXPECT_DOUBLE_EQ(raw.back().translation.x(), 0.4);
  EXPECT_DOUBLE_EQ(snapshot.poses.back().map_from_body.translation.x(), 0.4);
}

TEST(PoseGraphTest, LoopDirectionAndNoiseOrder) {
  constexpr double kHalfPi = 1.5707963267948966;
  const Eigen::Matrix3d anchor =
      Eigen::AngleAxisd(kHalfPi, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const std::vector<RigidTransform> raw(3, Pose(anchor, 2, 3, 1));
  auto graph = MakeGraph(raw);
  RigidTransform measurement;
  measurement.rotation =
      Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX()).toRotationMatrix();
  measurement.translation.x() = 0.3;
  graph.AddLoopConstraint({0, 2, measurement, 0.2, 0.5});
  const auto result = graph.Optimize(RawSnapshot(raw));

  // The rotational and translational residuals are both along x, so the
  // SE(3) logarithm yields their signed magnitudes without cross terms.
  const double expected_error =
      0.5 * (std::pow(0.4 / 0.2, 2) + std::pow(0.3 / 0.5, 2));
  EXPECT_NEAR(result.initial_error, expected_error, 1e-10);
  EXPECT_LT(result.final_error, result.initial_error);
  ASSERT_EQ(result.update.poses.size(), 3u);
  const auto& first = result.update.poses.front().map_from_body;
  const auto& last = result.update.poses.back().map_from_body;
  const Eigen::Matrix3d relative_rotation =
      first.rotation.transpose() * last.rotation;
  const Eigen::Vector3d relative_translation =
      first.rotation.transpose() * (last.translation - first.translation);
  EXPECT_GT(relative_rotation(2, 1), 1e-3);
  EXPECT_GT(relative_translation.x(), 1e-3);
}

TEST(PoseGraphTest, InvalidInputsDoNotChangeTheGraph) {
  PoseGraphOptions bad_options;
  bad_options.max_iterations = 0;
  EXPECT_THROW(PoseGraph{bad_options}, std::invalid_argument);
  bad_options = {};
  bad_options.prior_translation_sigma_m =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(PoseGraph{bad_options}, std::invalid_argument);
  bad_options = {};
  bad_options.prior_rotation_sigma_rad =
      std::numeric_limits<double>::denorm_min();
  EXPECT_THROW(PoseGraph{bad_options}, std::invalid_argument);
  bad_options = {};
  bad_options.odometry_translation_sigma_m = std::numeric_limits<double>::max();
  EXPECT_THROW(PoseGraph{bad_options}, std::invalid_argument);

  const auto raw = Square();
  PoseGraph graph({});
  EXPECT_THROW(graph.AddKeyframe(1, raw[0]), std::invalid_argument);
  auto invalid = raw[0];
  invalid.rotation.setZero();
  EXPECT_THROW(graph.AddKeyframe(0, invalid), std::invalid_argument);
  invalid = raw[0];
  invalid.translation.x() = std::numeric_limits<double>::infinity();
  EXPECT_THROW(graph.AddKeyframe(0, invalid), std::invalid_argument);
  for (std::uint64_t i = 0; i < raw.size(); ++i) {
    graph.AddKeyframe(i, raw[i]);
  }
  EXPECT_THROW(graph.AddKeyframe(4, raw[4]), std::invalid_argument);
  auto loop = ClosingLoop();
  loop.to_id = 5;
  EXPECT_THROW(graph.AddLoopConstraint(loop), std::invalid_argument);
  loop = ClosingLoop();
  loop.from_id = 4;
  loop.to_id = 0;
  EXPECT_THROW(graph.AddLoopConstraint(loop), std::invalid_argument);
  loop = ClosingLoop();
  loop.to_id = 1;
  EXPECT_THROW(graph.AddLoopConstraint(loop), std::invalid_argument);
  loop = ClosingLoop();
  loop.rotation_sigma_rad = 0;
  EXPECT_THROW(graph.AddLoopConstraint(loop), std::invalid_argument);
  loop = ClosingLoop();
  loop.translation_sigma_m = std::numeric_limits<double>::infinity();
  EXPECT_THROW(graph.AddLoopConstraint(loop), std::invalid_argument);
  loop = ClosingLoop();
  loop.rotation_sigma_rad = std::numeric_limits<double>::denorm_min();
  EXPECT_THROW(graph.AddLoopConstraint(loop), std::invalid_argument);
  loop = ClosingLoop();
  loop.from_body_from_to_body.rotation.setZero();
  EXPECT_THROW(graph.AddLoopConstraint(loop), std::invalid_argument);
  graph.AddLoopConstraint(ClosingLoop());
  EXPECT_THROW(graph.AddLoopConstraint(ClosingLoop()), std::invalid_argument);
  auto reference = MakeGraph(raw);
  reference.AddLoopConstraint(ClosingLoop());
  const auto actual = graph.Optimize(RawSnapshot(raw));
  const auto expected = reference.Optimize(RawSnapshot(raw));
  EXPECT_DOUBLE_EQ(actual.final_error, expected.final_error);
  ASSERT_EQ(actual.update.poses.size(), expected.update.poses.size());
  for (std::size_t i = 0; i < actual.update.poses.size(); ++i) {
    EXPECT_TRUE(actual.update.poses[i].map_from_body.translation.isApprox(
        expected.update.poses[i].map_from_body.translation, 1e-12));
  }
}

TEST(PoseGraphTest, ProjectsOnlyBackendRotation) {
  auto raw = Square();
  raw[0].rotation(0, 0) = 1.000045;
  raw[1].rotation(0, 0) = 1.000045;
  const auto snapshot = RawSnapshot(raw);
  const auto result = MakeGraph(raw).Optimize(snapshot);
  for (const auto& pose : result.update.poses) {
    EXPECT_TRUE(pose.map_from_body.rotation.isApprox(
        Eigen::Matrix3d::Identity(), 1e-12));
  }
  EXPECT_DOUBLE_EQ(raw[0].rotation(0, 0), 1.000045);
  EXPECT_DOUBLE_EQ(snapshot.poses[0].map_from_body.rotation(0, 0), 1.000045);
  auto invalid = raw[0];
  invalid.rotation(0, 0) = 1.0001;
  PoseGraph graph({});
  EXPECT_THROW(graph.AddKeyframe(0, invalid), std::invalid_argument);
}

TEST(PoseGraphTest, ValidatesSnapshotAndUsesCurrentMapPoses) {
  const auto raw = Square();
  auto graph = MakeGraph(raw);
  auto wrong_raw = RawSnapshot(raw);
  wrong_raw.poses[1].map_from_body.rotation(0, 0) = 1.000001;
  EXPECT_THROW(graph.Optimize(wrong_raw), std::invalid_argument);
  auto too_short = RawSnapshot(raw);
  too_short.poses.pop_back();
  --too_short.frame_count;
  EXPECT_THROW(graph.Optimize(too_short), std::invalid_argument);
  EXPECT_THROW(PoseGraph({}).Optimize(RawSnapshot({})), std::invalid_argument);

  PoseUpdate update;
  update.optimized_frame_count = raw.size();
  for (std::uint64_t i = 0; i < raw.size(); ++i) {
    auto shifted = raw[i];
    shifted.translation.x() += 1;
    update.poses.push_back({i, shifted});
  }
  const auto corrected = MakePoseSnapshot(update, {}, raw);
  const auto result = graph.Optimize(corrected);
  EXPECT_EQ(result.update.base_version, 1u);
  EXPECT_GT(result.initial_error, result.final_error);
  EXPECT_LT(result.update.poses.front().map_from_body.translation.norm(), 1e-4);
  auto exhausted = corrected;
  exhausted.version = std::numeric_limits<std::uint64_t>::max();
  exhausted.base_version = exhausted.version - 1;
  EXPECT_THROW(graph.Optimize(exhausted), std::overflow_error);
  auto partial_graph = MakeGraph({raw[0], raw[1]});
  EXPECT_THROW(partial_graph.Optimize(corrected), std::invalid_argument);
  EXPECT_NO_THROW(graph.Optimize(RawSnapshot(raw)));
}

TEST(PoseGraphTest, ReportsIterationLimitWithoutClaimingConvergence) {
  const auto raw = Square();
  PoseGraphOptions options;
  options.max_iterations = 1;
  auto graph = MakeGraph(raw, options);
  graph.AddLoopConstraint(ClosingLoop());
  const auto result = graph.Optimize(RawSnapshot(raw));
  EXPECT_EQ(result.iterations, 1u);
  EXPECT_TRUE(result.iteration_limit_reached);
  EXPECT_TRUE(std::isfinite(result.final_error));
  EXPECT_LE(result.final_error, result.initial_error);
}

TEST(PoseGraphTest, RejectsNonfiniteObjective) {
  auto raw = Square();
  raw.back().translation.x() = 1e200;
  auto graph = MakeGraph(raw);
  graph.AddLoopConstraint(ClosingLoop());
  EXPECT_THROW(graph.Optimize(RawSnapshot(raw)), PoseGraphOptimizationError);
  EXPECT_THROW(graph.Optimize(RawSnapshot(raw)), PoseGraphOptimizationError);
  EXPECT_DOUBLE_EQ(raw.back().translation.x(), 1e200);
}

TEST(PoseGraphTest, RejectsInvalidHuberThreshold) {
  for (double delta : {-1.0, std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()}) {
    PoseGraphOptions options;
    options.loop_huber_delta = delta;
    EXPECT_THROW(PoseGraph{options}, std::invalid_argument);
  }
}

TEST(PoseGraphTest, HuberDownweightsTheWholeLoopAndRetainsRawResidual) {
  const std::vector<RigidTransform> raw(3);
  LoopConstraint loop{0, 2, {}, 0.1, 0.1};
  loop.from_body_from_to_body.translation = {3, 4, 0};
  auto gaussian = MakeGraph(raw);
  gaussian.AddLoopConstraint(loop);
  const auto gaussian_result = gaussian.Optimize(RawSnapshot(raw));
  PoseGraphOptions options;
  options.loop_huber_delta = 1;
  auto robust = MakeGraph(raw, options);
  robust.AddLoopConstraint(loop);
  const auto result = robust.Optimize(RawSnapshot(raw));

  EXPECT_NEAR(gaussian_result.initial_error, 1250, 1e-10);
  // Block Huber on norm(30, 40) = 50, rather than separate coordinate weights.
  EXPECT_NEAR(result.initial_error, 49.5, 1e-10);
  ASSERT_EQ(result.loop_residuals.size(), 1u);
  EXPECT_NEAR(result.loop_residuals[0].initial_whitened_norm, 50, 1e-10);
  EXPECT_GT(result.loop_residuals[0].final_whitened_norm, 40);
  EXPECT_LT(result.update.poses.back().map_from_body.translation.norm(), 0.3);
  EXPECT_GT(
      gaussian_result.update.poses.back().map_from_body.translation.norm(), 3);
  const auto& translation =
      result.update.poses.back().map_from_body.translation;
  EXPECT_NEAR(translation.y() / translation.x(), 4.0 / 3.0, 1e-6);
}

TEST(PoseGraphTest, HuberDoesNotChangePriorOrOdometry) {
  const auto raw = Square();
  PoseUpdate update;
  update.optimized_frame_count = raw.size();
  for (std::uint64_t i = 0; i < raw.size(); ++i) {
    auto shifted = raw[i];
    shifted.translation.x() += 1 + i;
    update.poses.push_back({i, shifted});
  }
  const auto current = MakePoseSnapshot(update, {}, raw);
  PoseGraphOptions options;
  options.loop_huber_delta = 1;
  const auto result = MakeGraph(raw, options).Optimize(current);
  const auto gaussian = MakeGraph(raw).Optimize(current);
  EXPECT_DOUBLE_EQ(result.initial_error, gaussian.initial_error);
  EXPECT_DOUBLE_EQ(result.final_error, gaussian.final_error);
  EXPECT_NEAR(result.initial_error, 500200, 1e-6);
  ASSERT_EQ(result.odometry_residuals.size(), 4u);
  EXPECT_TRUE(result.loop_residuals.empty());
  for (const auto& residual : result.odometry_residuals) {
    EXPECT_EQ(residual.to_id, residual.from_id + 1);
    EXPECT_NEAR(residual.initial_whitened_norm, 10, 1e-10);
    EXPECT_LT(residual.final_whitened_norm, 1e-3);
  }
}

TEST(PoseGraphTest, DiagnosticsUseActualFactorTangentAndMeasurementDirection) {
  auto raw = Square();
  raw.back().rotation =
      Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()).toRotationMatrix();
  auto loop = ClosingLoop();
  loop.rotation_sigma_rad = 0.2;
  loop.translation_sigma_m = 0.5;
  loop.from_body_from_to_body.rotation =
      Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX()).toRotationMatrix();
  loop.from_body_from_to_body.translation = {0.1, 0.2, 0.3};
  PoseGraphOptions options;
  options.loop_huber_delta = 1;
  auto graph = MakeGraph(raw, options);
  graph.AddLoopConstraint(loop);
  const auto result = graph.Optimize(RawSnapshot(raw));
  ASSERT_EQ(result.loop_residuals.size(), 1u);
  const auto& residual = result.loop_residuals.front();
  EXPECT_EQ(residual.from_id, 0u);
  EXPECT_EQ(residual.to_id, 4u);
  const auto to_gtsam = [](const RigidTransform& pose) {
    return gtsam::Pose3(gtsam::Rot3(pose.rotation), pose.translation);
  };
  const auto measurement = to_gtsam(loop.from_body_from_to_body);
  const gtsam::BetweenFactor<gtsam::Pose3> factor(
      0, 4, measurement, gtsam::noiseModel::Unit::Create(6));
  gtsam::Values initial;
  gtsam::Values final;
  for (std::size_t i = 0; i < raw.size(); ++i) {
    initial.insert(i, to_gtsam(raw[i]));
    final.insert(i, to_gtsam(result.update.poses[i].map_from_body));
  }
  const auto whitened_norm = [&](const gtsam::Values& values) {
    gtsam::Vector error = factor.unwhitenedError(values);
    error.head<3>() /= 0.2;
    error.tail<3>() /= 0.5;
    return error.norm();
  };
  EXPECT_NEAR(residual.initial_whitened_norm, whitened_norm(initial), 1e-10);
  EXPECT_NEAR(residual.final_whitened_norm, whitened_norm(final), 1e-10);
  const auto prediction =
      final.at<gtsam::Pose3>(0).between(final.at<gtsam::Pose3>(4));
  const auto error = measurement.between(prediction);
  EXPECT_NEAR(residual.final_translation_m, error.translation().norm(), 1e-10);
  EXPECT_NEAR(residual.final_rotation_rad,
              Eigen::AngleAxisd(error.rotation().matrix()).angle(), 1e-10);
}

}  // namespace
}  // namespace livo_atlas
