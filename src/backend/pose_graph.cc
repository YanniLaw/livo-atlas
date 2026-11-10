#include "livo_atlas/backend/pose_graph.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtsam/geometry/Pose3.h>
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

}  // namespace

PoseGraph::PoseGraph(PoseGraphOptions options) : options_(options) {
  if (!ValidSigma(options.prior_rotation_sigma_rad) ||
      !ValidSigma(options.prior_translation_sigma_m) ||
      !ValidSigma(options.odometry_rotation_sigma_rad) ||
      !ValidSigma(options.odometry_translation_sigma_m) ||
      options.max_iterations <= 0) {
    throw std::invalid_argument("Invalid pose graph options");
  }
}

void PoseGraph::AddKeyframe(std::uint64_t id,
                            const RigidTransform& odom_from_body) {
  if (id != raw_poses_.size()) {
    throw std::invalid_argument("Keyframe IDs must be contiguous from zero");
  }
  ValidatePose(odom_from_body);
  raw_poses_.push_back(odom_from_body);
}

void PoseGraph::AddLoopConstraint(const LoopConstraint& constraint) {
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

PoseGraphResult PoseGraph::Optimize(const PoseSnapshot& current) const {
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
      if (!SamePose(current.poses[i].map_from_body, raw_poses_[i])) {
        throw std::invalid_argument("Version zero differs from raw poses");
      }
    }
  }

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;
  std::vector<gtsam::Pose3> odometry;
  odometry.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    odometry.push_back(ToGtsam(raw_poses_[i]));
    initial.insert(i, ToGtsam(current.poses[i].map_from_body));
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
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        loop.from_id, loop.to_id, ToGtsam(loop.from_body_from_to_body),
        Noise(loop.rotation_sigma_rad, loop.translation_sigma_m));
  }

  PoseGraphResult result;
  result.initial_error = graph.error(initial);
  if (!std::isfinite(result.initial_error)) {
    throw std::runtime_error("Non-finite initial pose graph error");
  }
  gtsam::LevenbergMarquardtParams parameters;
  parameters.setMaxIterations(options_.max_iterations);
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, parameters);
  const auto values = optimizer.optimize();
  result.iterations = optimizer.iterations();
  result.iteration_limit_reached =
      result.iterations >= static_cast<std::size_t>(options_.max_iterations);
  result.final_error = graph.error(values);
  if (!std::isfinite(result.final_error) ||
      result.final_error - result.initial_error >
          1e-12 * std::max(1.0, result.initial_error)) {
    throw std::runtime_error("Invalid optimized pose graph error");
  }

  result.update.base_version = current.version;
  result.update.optimized_frame_count = count;
  result.update.tail_policy = PoseTailPolicy::kProjectedLastOptimizedAnchor;
  result.update.poses.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const auto& value = values.at<gtsam::Pose3>(i);
    RigidTransform pose{value.rotation().matrix(), value.translation()};
    ValidatePose(pose);
    result.update.poses.push_back({i, pose});
  }
  return result;
}

}  // namespace livo_atlas
