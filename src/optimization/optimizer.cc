#include "livo_atlas/optimization/optimizer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "livo_atlas/backend/pose_graph.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/LossFunctions.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "common/rotation_projection.h"

namespace livo_atlas {
namespace {

bool ValidSigma(double value) {
  if (!std::isfinite(value) || value <= 0) return false;
  const double inverse = 1.0 / value;
  const double precision = inverse * inverse;
  return std::isfinite(precision) && precision > 0;
}

void ValidatePose(const RigidTransform& pose) {
  const auto error = ValidateExtrinsic(pose);
  if (!error.empty()) throw std::invalid_argument(error);
}

gtsam::Pose3 ToGtsam(const RigidTransform& pose) {
  return {gtsam::Rot3(ProjectRotation(pose.rotation)), pose.translation};
}

gtsam::SharedNoiseModel Noise(double rotation, double translation) {
  // Pose3 tangent vectors put rotation before translation.
  gtsam::Vector6 sigmas;
  sigmas << rotation, rotation, rotation, translation, translation, translation;
  // Disable GTSAM's automatic conversion of tiny sigmas to constraints.
  return gtsam::noiseModel::Diagonal::Sigmas(sigmas, false);
}

bool SamePose(const RigidTransform& a, const RigidTransform& b) {
  return (a.rotation.array() == b.rotation.array()).all() &&
         (a.translation.array() == b.translation.array()).all();
}

FactorResidual Residual(const gtsam::BetweenFactor<gtsam::Pose3>& factor,
                        const gtsam::Values& initial,
                        const gtsam::Values& final, double rotation_sigma,
                        double translation_sigma) {
  const auto norm = [&](const gtsam::Values& values) {
    gtsam::Vector error = factor.unwhitenedError(values);
    error.head<3>() /= rotation_sigma;
    error.tail<3>() /= translation_sigma;
    return error.norm();
  };
  FactorResidual result;
  result.from_id = factor.key1();
  result.to_id = factor.key2();
  result.initial_whitened_norm = norm(initial);
  result.final_whitened_norm = norm(final);
  const auto prediction = final.at<gtsam::Pose3>(factor.key1())
                              .between(final.at<gtsam::Pose3>(factor.key2()));
  const auto error = factor.measured().between(prediction);
  result.final_rotation_rad = gtsam::Rot3::Logmap(error.rotation()).norm();
  result.final_translation_m = error.translation().norm();
  if (!std::isfinite(result.initial_whitened_norm) ||
      !std::isfinite(result.final_whitened_norm) ||
      !std::isfinite(result.final_rotation_rad) ||
      !std::isfinite(result.final_translation_m)) {
    throw PoseGraphOptimizationError("Non-finite pose graph residual");
  }
  return result;
}

}  // namespace

Optimizer::Optimizer(PoseGraphOptions options) : options_(options) {
  if (!ValidSigma(options.prior_rotation_sigma_rad) ||
      !ValidSigma(options.prior_translation_sigma_m) ||
      !ValidSigma(options.odometry_rotation_sigma_rad) ||
      !ValidSigma(options.odometry_translation_sigma_m) ||
      options.max_iterations <= 0 || !std::isfinite(options.loop_huber_delta) ||
      options.loop_huber_delta < 0) {
    throw std::invalid_argument("Invalid pose graph options");
  }
}

void Optimizer::AddKeyframe(std::uint64_t id,
                            const RigidTransform& odom_from_body) {
  if (id != raw_poses_.size()) {
    throw std::invalid_argument("Keyframe IDs must be contiguous from zero");
  }
  ValidatePose(odom_from_body);
  raw_poses_.push_back(odom_from_body);
}

void Optimizer::AddLoopConstraint(const LoopConstraint& constraint) {
  if (constraint.from_id >= constraint.to_id ||
      constraint.to_id >= raw_poses_.size() ||
      constraint.to_id - constraint.from_id <= 1 ||
      !ValidSigma(constraint.rotation_sigma_rad) ||
      !ValidSigma(constraint.translation_sigma_m)) {
    throw std::invalid_argument("Invalid loop constraint");
  }
  ValidatePose(constraint.from_body_from_to_body);
  for (const auto& existing : constraints_) {
    if (existing.from_id == constraint.from_id &&
        existing.to_id == constraint.to_id) {
      throw std::invalid_argument("Duplicate loop constraint");
    }
  }
  constraints_.push_back(constraint);
}

PoseGraphResult Optimizer::Optimize(const PoseSnapshot& current) const {
  ValidatePoseSnapshot(current);
  const auto count = raw_poses_.size();
  if (count == 0 || count > current.frame_count ||
      count < current.optimized_frame_count) {
    throw std::invalid_argument("Pose graph does not cover a valid prefix");
  }
  if (current.version == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("Pose version exhausted");
  }
  if (current.version == 0) {
    for (std::size_t i = 0; i < count; ++i) {
      if (!SamePose(current.poses[i].parent_from_body(), raw_poses_[i])) {
        throw std::invalid_argument("Version zero differs from raw poses");
      }
    }
  }

  const auto build_start = std::chrono::steady_clock::now();
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;
  std::vector<gtsam::Pose3> odometry;
  odometry.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    odometry.push_back(ToGtsam(raw_poses_[i]));
    initial.insert(i, ToGtsam(current.poses[i].parent_from_body()));
  }
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      0, odometry.front(),
      Noise(options_.prior_rotation_sigma_rad,
            options_.prior_translation_sigma_m));
  const auto odometry_noise = Noise(options_.odometry_rotation_sigma_rad,
                                    options_.odometry_translation_sigma_m);
  for (std::size_t i = 1; i < count; ++i) {
    // O_(i-1)^-1 * O_i maps the later body frame into the earlier one.
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        i - 1, i, odometry[i - 1].between(odometry[i]), odometry_noise);
  }
  for (const auto& loop : constraints_) {
    auto noise = Noise(loop.rotation_sigma_rad, loop.translation_sigma_m);
    if (options_.loop_huber_delta > 0) {
      noise = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create(
              options_.loop_huber_delta,
              gtsam::noiseModel::mEstimator::Base::Block),
          noise);
    }
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        loop.from_id, loop.to_id, ToGtsam(loop.from_body_from_to_body), noise);
  }

  PoseGraphResult result;
  result.initial_error = graph.error(initial);
  if (!std::isfinite(result.initial_error)) {
    throw PoseGraphOptimizationError("Non-finite initial pose graph error");
  }
  result.graph_build_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    build_start)
          .count();
  gtsam::LevenbergMarquardtParams parameters;
  parameters.setMaxIterations(options_.max_iterations);
  gtsam::Values values;
  const auto solve_start = std::chrono::steady_clock::now();
  try {
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, parameters);
    values = optimizer.optimize();
    result.iterations = optimizer.iterations();
  } catch (const gtsam::IndeterminantLinearSystemException& error) {
    throw PoseGraphOptimizationError(error.what());
  } catch (const std::runtime_error& error) {
    // Only solver failures are normal trial rejections. Allocation failures and
    // unknown exception types propagate to the caller.
    throw PoseGraphOptimizationError(error.what());
  }
  result.solve_seconds = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - solve_start)
                             .count();
  result.iteration_limit_reached =
      result.iterations >= static_cast<std::size_t>(options_.max_iterations);
  result.final_error = graph.error(values);
  if (!std::isfinite(result.final_error) ||
      result.final_error - result.initial_error >
          1e-12 * std::max(1.0, result.initial_error)) {
    throw PoseGraphOptimizationError("Invalid optimized pose graph error");
  }

  result.update.base_version = current.version;
  result.update.optimized_frame_count = count;
  result.update.tail_policy = PoseTailPolicy::kProjectedLastOptimizedAnchor;
  result.update.poses.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const auto& value = values.at<gtsam::Pose3>(i);
    RigidTransform pose{value.rotation().matrix(), value.translation()};
    const auto error = ValidateExtrinsic(pose);
    if (!error.empty()) throw PoseGraphOptimizationError(error);
    result.update.poses.push_back({i, pose});
  }
  result.odometry_residuals.reserve(count - 1);
  result.loop_residuals.reserve(constraints_.size());
  // Factor order is prior, adjacent odometry, then the accepted loop order.
  for (std::size_t i = 1; i < count; ++i) {
    const auto& factor =
        static_cast<const gtsam::BetweenFactor<gtsam::Pose3>&>(*graph[i]);
    result.odometry_residuals.push_back(
        Residual(factor, initial, values, options_.odometry_rotation_sigma_rad,
                 options_.odometry_translation_sigma_m));
  }
  for (std::size_t i = 0; i < constraints_.size(); ++i) {
    const auto& factor = static_cast<const gtsam::BetweenFactor<gtsam::Pose3>&>(
        *graph[count + i]);
    result.loop_residuals.push_back(
        Residual(factor, initial, values, constraints_[i].rotation_sigma_rad,
                 constraints_[i].translation_sigma_m));
  }
  return result;
}

PoseGraph::PoseGraph(PoseGraphOptions options) : optimizer_(options) {}

void PoseGraph::AddKeyframe(std::uint64_t id,
                            const RigidTransform& odom_from_body) {
  optimizer_.AddKeyframe(id, odom_from_body);
}

void PoseGraph::AddLoopConstraint(const LoopConstraint& constraint) {
  optimizer_.AddLoopConstraint(constraint);
}

PoseGraphResult PoseGraph::Optimize(const PoseSnapshot& current) const {
  return optimizer_.Optimize(current);
}

}  // namespace livo_atlas
