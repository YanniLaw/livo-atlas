#include "backend/loop_internal.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/Eigenvalues>
#include <pcl/kdtree/kdtree_flann.h>

#include "mapping/pose_math.h"

namespace livo_atlas::loop_internal {
namespace {

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

pcl::PointXYZ Point(const Eigen::Vector3d& value) {
  const auto point = value.cast<float>().eval();
  if (!point.allFinite()) {
    throw std::invalid_argument("Loop point cannot be represented by PCL");
  }
  return pcl::PointXYZ(point.x(), point.y(), point.z());
}

LoopTransformError Error(const RigidTransform& transform) {
  const double cosine =
      std::clamp((transform.rotation.trace() - 1) / 2, -1.0, 1.0);
  return {transform.translation.stableNorm(),
          std::acos(cosine) * (180.0 / 3.14159265358979323846)};
}

}  // namespace

std::array<LoopTransformError, 4> TransformErrors(
    const RigidTransform& initial, const RigidTransform& forward,
    const RigidTransform& reverse) {
  return {Error(pose_math::Compose(forward, reverse)),
          Error(pose_math::Compose(reverse, forward)),
          Error(pose_math::Compose(pose_math::Inverse(initial), forward)),
          Error(pose_math::Compose(initial, reverse))};
}

LoopMetrics EvaluateMetrics(const GeometryData& target,
                            const GeometryData& source,
                            const RigidTransform& target_from_source,
                            const LoopRegistrationOptions& options) {
  LoopMetrics result;
  if (target.voxels.empty() || source.voxels.empty()) return result;
  Cloud::Ptr target_cloud(new Cloud), source_cloud(new Cloud);
  std::vector<Eigen::Vector3d> transformed;
  transformed.reserve(source.voxels.size());
  for (const auto& voxel : target.voxels)
    target_cloud->push_back(Point(voxel.centroid));
  for (const auto& voxel : source.voxels) {
    transformed.push_back(target_from_source.rotation * voxel.centroid +
                          target_from_source.translation);
    source_cloud->push_back(Point(transformed.back()));
  }
  pcl::KdTreeFLANN<pcl::PointXYZ> target_tree, source_tree;
  target_tree.setInputCloud(target_cloud);
  source_tree.setInputCloud(source_cloud);
  std::vector<int> index(1), reverse_index(1);
  std::vector<float> distance(1), reverse_distance(1);
  std::vector<double> residuals;
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> planes;
  const double radius_squared =
      options.match_distance_m * options.match_distance_m;
  double sum_squared = 0;
  for (std::size_t i = 0; i < source_cloud->size(); ++i) {
    if (!target_tree.nearestKSearch((*source_cloud)[i], 1, index, distance))
      continue;
    const auto j = static_cast<std::size_t>(index[0]);
    if (!source_tree.nearestKSearch((*target_cloud)[j], 1, reverse_index,
                                    reverse_distance) ||
        reverse_index[0] != static_cast<int>(i)) {
      continue;
    }
    const double squared =
        (transformed[i] - target.voxels[j].centroid).squaredNorm();
    if (squared > radius_squared) continue;
    residuals.push_back(std::sqrt(squared));
    sum_squared += squared;
    if (target.voxels[j].normal) {
      planes.emplace_back(transformed[i], *target.voxels[j].normal);
    }
  }
  result.inliers = residuals.size();
  result.normal_pairs = planes.size();
  result.source_coverage =
      static_cast<double>(result.inliers) / source.voxels.size();
  result.target_coverage =
      static_cast<double>(result.inliers) / target.voxels.size();
  if (!residuals.empty()) {
    result.rmse_m = std::sqrt(sum_squared / residuals.size());
    std::sort(residuals.begin(), residuals.end());
    result.p95_m =
        residuals[static_cast<std::size_t>(std::ceil(0.95 * residuals.size())) -
                  1];
  }
  if (planes.size() < options.min_normal_pairs) return result;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for (const auto& pair : planes) center += pair.first;
  center /= planes.size();
  double radius = 0;
  for (const auto& pair : planes) radius += (pair.first - center).squaredNorm();
  const double length = std::max(1.0, std::sqrt(radius / planes.size()));
  Eigen::Matrix<double, 6, 6> hessian = Eigen::Matrix<double, 6, 6>::Zero();
  for (const auto& pair : planes) {
    Eigen::Matrix<double, 6, 1> row;
    row.head<3>() = (pair.first - center).cross(pair.second) / length;
    row.tail<3>() = pair.second;
    hessian.noalias() += row * row.transpose();
  }
  hessian /= planes.size();
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(
      hessian);
  if (solver.info() == Eigen::Success && solver.eigenvalues().allFinite() &&
      solver.eigenvalues()[5] > 0) {
    result.eigenvalue_ratio =
        std::max(0.0, solver.eigenvalues()[0]) / solver.eigenvalues()[5];
  }
  return result;
}

LoopStatus ClassifyQuality(
    const LoopMetrics& forward, const LoopMetrics& reverse,
    const std::optional<std::array<LoopTransformError, 4>>& errors,
    const LoopRegistrationOptions& options) {
  for (const auto* metrics : {&forward, &reverse}) {
    if (metrics->inliers < options.min_inliers ||
        metrics->source_coverage < options.min_coverage ||
        metrics->target_coverage < options.min_coverage)
      return LoopStatus::kLowOverlap;
  }
  for (const auto* metrics : {&forward, &reverse}) {
    if (!metrics->rmse_m || !metrics->p95_m ||
        !std::isfinite(*metrics->rmse_m) || !std::isfinite(*metrics->p95_m) ||
        *metrics->rmse_m > options.max_rmse_m ||
        *metrics->p95_m > options.max_p95_m)
      return LoopStatus::kHighResidual;
  }
  // Fixed-transform checks have no independent cycle or fitted correction.
  for (std::size_t i = 0; errors && i < errors->size(); ++i) {
    if (!std::isfinite((*errors)[i].translation_m) ||
        !std::isfinite((*errors)[i].rotation_deg) ||
        (*errors)[i].translation_m > (i < 2 ? options.max_cycle_translation_m
                                            : options.max_correction_m) ||
        (*errors)[i].rotation_deg > (i < 2 ? options.max_cycle_rotation_deg
                                           : options.max_correction_deg)) {
      return i < 2 ? LoopStatus::kInconsistentTransform
                   : LoopStatus::kLargeCorrection;
    }
  }
  for (const auto* metrics : {&forward, &reverse}) {
    if (!metrics->eigenvalue_ratio ||
        !std::isfinite(*metrics->eigenvalue_ratio) ||
        *metrics->eigenvalue_ratio < options.min_eigenvalue_ratio)
      return LoopStatus::kDegenerate;
  }
  return LoopStatus::kVerified;
}

}  // namespace livo_atlas::loop_internal
