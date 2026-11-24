#include "livo_atlas/backend/loop_closure.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>

#include <Eigen/Eigenvalues>
#include <pcl/registration/gicp.h>

#include "loop_closure/loop_internal.h"
#include "common/rotation_projection.h"
#include "mapping/pose_math.h"

namespace livo_atlas {
namespace {

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

double PclCoordinateLimit() {
  // Reserve headroom for float covariance products and squared distances.
  return std::sqrt(static_cast<double>(std::numeric_limits<float>::max())) / 16;
}

bool SafePclPoint(const Eigen::Vector3d& point) {
  return point.allFinite() &&
         point.cwiseAbs().maxCoeff() <= PclCoordinateLimit();
}

bool SafeTransform(const Cloud& cloud, const Eigen::Matrix4f& transform) {
  if (!transform.allFinite() ||
      transform.topLeftCorner<3, 3>().cwiseAbs().maxCoeff() > 2 ||
      transform.topRightCorner<3, 1>().cast<double>().cwiseAbs().maxCoeff() >
          4 * PclCoordinateLimit())
    return false;
  const auto rotation = transform.topLeftCorner<3, 3>().cast<double>().eval();
  const auto translation =
      transform.topRightCorner<3, 1>().cast<double>().eval();
  for (const auto& point : cloud) {
    if (!SafePclPoint(rotation * point.getVector3fMap().cast<double>() +
                      translation))
      return false;
  }
  return true;
}

Eigen::Matrix4f InitialGuess(const Cloud& cloud,
                             const RigidTransform& initial) {
  Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
  guess.topLeftCorner<3, 3>() = initial.rotation.cast<float>();
  guess.topRightCorner<3, 1>() = initial.translation.cast<float>();
  if (!SafeTransform(cloud, guess)) {
    throw std::invalid_argument(
        "Loop initial guess exceeds PCL arithmetic limits");
  }
  return guess;
}

bool Positive(double value) {
  return std::isfinite(value) && value > 0 && std::isfinite(1.0 / value);
}

RigidTransform Projected(RigidTransform value) {
  const auto error = ValidateExtrinsic(value);
  if (!error.empty()) throw std::invalid_argument(error);
  value.rotation = ProjectRotation(value.rotation);
  return value;
}

LoopProvenance Provenance(const GeometrySubmap& submap) {
  return {submap.membership.primary.id,
          submap.membership.primary.anchor_keyframe_id,
          submap.membership.membership_revision,
          submap.representation_revision};
}

void ValidateGeometry(const GeometrySubmap& submap) {
  const auto& group = submap.membership;
  ValidateSubmap(group.primary);
  const auto& ids = group.primary.keyframe_ids;
  if (!submap.data || group.state != OnlineSubmapState::kSealed ||
      ids.size() > kMaxGeometryKeyframes ||
      group.membership_revision != ids.size() + 1 ||
      !submap.representation_revision || ids.back() >= submap.frame_count ||
      submap.optimized_frame_count > submap.frame_count ||
      ((submap.pose_version == 0) != (submap.optimized_frame_count == 0)) ||
      submap.parent_frame !=
          (submap.pose_version ? PoseFrame::kMap : PoseFrame::kOdom) ||
      submap.data->sources.size() != ids.size()) {
    throw std::invalid_argument("Invalid loop geometry view");
  }
  (void)Projected(submap.parent_from_submap);
  for (std::size_t i = 0; i < ids.size(); ++i) {
    if (submap.data->sources[i].keyframe_id != ids[i]) {
      throw std::invalid_argument(
          "Loop geometry sources differ from membership");
    }
    (void)Projected(submap.data->sources[i].submap_from_body);
  }
}

void ValidateVoxels(const GeometryData& data) {
  std::set<GeometryVoxelKey> keys;
  for (const auto& voxel : data.voxels) {
    if (!voxel.point_count || !SafePclPoint(voxel.centroid) ||
        !std::isfinite(voxel.mean_intensity) ||
        !voxel.spatial_covariance.allFinite() ||
        !voxel.spatial_covariance.isApprox(voxel.spatial_covariance.transpose(),
                                           1e-10) ||
        !keys.insert(voxel.key).second || voxel.keyframe_ids.empty()) {
      throw std::invalid_argument("Invalid loop geometry voxel");
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(
        voxel.spatial_covariance);
    if (solver.info() != Eigen::Success ||
        solver.eigenvalues().minCoeff() < -1e-10) {
      throw std::invalid_argument("Invalid loop point distribution");
    }
    if (voxel.normal && (!voxel.normal->allFinite() ||
                         std::abs(voxel.normal->norm() - 1) > 1e-4)) {
      throw std::invalid_argument("Invalid loop geometry normal");
    }
    for (std::size_t i = 0; i < voxel.keyframe_ids.size(); ++i) {
      const auto id = voxel.keyframe_ids[i];
      if ((i && id <= voxel.keyframe_ids[i - 1]) ||
          std::none_of(
              data.sources.begin(), data.sources.end(),
              [id](const auto& source) { return source.keyframe_id == id; })) {
        throw std::invalid_argument("Unknown loop voxel source");
      }
    }
  }
}

Cloud::Ptr Downsample(const GeometryData& data, double size) {
  struct Mean {
    std::size_t count = 0;
    Eigen::Vector3d value = Eigen::Vector3d::Zero();
  };

  std::map<GeometryVoxelKey, Mean> cells;
  for (const auto& voxel : data.voxels) {
    const auto scaled = (voxel.centroid / size).array().floor().eval();
    if (!scaled.allFinite() ||
        (scaled < static_cast<double>(std::numeric_limits<std::int64_t>::min()))
            .any() ||
        (scaled >=
         static_cast<double>(std::numeric_limits<std::int64_t>::max()))
            .any()) {
      throw std::invalid_argument("Loop voxel coordinates exceed index range");
    }
    auto& mean = cells[{static_cast<std::int64_t>(scaled.x()),
                        static_cast<std::int64_t>(scaled.y()),
                        static_cast<std::int64_t>(scaled.z())}];
    mean.value += (voxel.centroid - mean.value) / ++mean.count;
  }
  Cloud::Ptr cloud(new Cloud);
  cloud->reserve(cells.size());
  for (const auto& entry : cells) {
    const auto p = entry.second.value.cast<float>().eval();
    cloud->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
  }
  return cloud;
}

// PCL 1.10/1.12 expose the iteration counter only to derived implementations.
class Gicp : public pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ,
                                                          pcl::PointXYZ> {
 public:
  Gicp() {
    const auto estimate = rigid_transformation_estimation_;
    rigid_transformation_estimation_ =
        [estimate](const Cloud& source, const std::vector<int>& source_indices,
                   const Cloud& target, const std::vector<int>& target_indices,
                   Eigen::Matrix4f& transform) {
          estimate(source, source_indices, target, target_indices, transform);
          // PCL catches this before the next float nearest-neighbor query.
          if (!SafeTransform(source, transform)) {
            throw pcl::PCLException("Unsafe loop registration iterate");
          }
        };
  }

  int iterations() const { return nr_iterations_; }
};

LoopAlignment Align(const Cloud::ConstPtr& target,
                    const Cloud::ConstPtr& source, const Eigen::Matrix4f& guess,
                    const LoopRegistrationOptions& options) {
  Gicp solver;
  solver.setInputSource(source);
  solver.setInputTarget(target);
  solver.setCorrespondenceRandomness(options.neighbors);
  solver.setMaxCorrespondenceDistance(options.correspondence_distance_m);
  solver.setMaximumIterations(options.max_iterations);
  solver.setMaximumOptimizerIterations(options.max_optimizer_iterations);
  solver.setTransformationEpsilon(options.translation_epsilon);
  solver.setRotationEpsilon(options.rotation_epsilon);
  LoopAlignment result;
  Cloud output;
  try {
    solver.align(output, guess);
  } catch (const pcl::PCLException&) {
    return result;
  }
  result.iterations = solver.iterations();
  result.converged = solver.hasConverged();
  const auto transform = solver.getFinalTransformation();
  if (result.converged && transform.allFinite()) {
    RigidTransform pose{transform.topLeftCorner<3, 3>().cast<double>(),
                        transform.topRightCorner<3, 1>().cast<double>()};
    if (ValidateExtrinsic(pose).empty()) result.transform = Projected(pose);
  }
  return result;
}

struct PreparedGeometry {
  LoopStatus status = LoopStatus::kVerified;
  Cloud::Ptr target, source;
};

PreparedGeometry PrepareGeometry(const GeometrySubmap& target,
                                 const GeometrySubmap& source,
                                 const LoopRegistrationOptions& options) {
  ValidateLoopRegistrationOptions(options);
  ValidateGeometry(target);
  ValidateGeometry(source);
  if (target.pose_version != source.pose_version ||
      target.frame_count != source.frame_count ||
      target.optimized_frame_count != source.optimized_frame_count ||
      target.membership.primary.id >= source.membership.primary.id ||
      target.membership.primary.keyframe_ids.back() >=
          source.membership.primary.keyframe_ids.front()) {
    throw std::invalid_argument("Mixed or overlapping loop geometry views");
  }
  PreparedGeometry result;
  if (target.data->voxels.size() > options.max_voxels ||
      source.data->voxels.size() > options.max_voxels) {
    result.status = LoopStatus::kResourceLimit;
    return result;
  }
  ValidateVoxels(*target.data);
  ValidateVoxels(*source.data);
  result.target = Downsample(*target.data, options.voxel_size_m);
  result.source = Downsample(*source.data, options.voxel_size_m);
  if (result.target->size() > options.max_points ||
      result.source->size() > options.max_points) {
    result.status = LoopStatus::kResourceLimit;
    return result;
  }
  if (result.target->size() < options.min_points ||
      result.source->size() < options.min_points) {
    result.status = LoopStatus::kInsufficientPoints;
  }
  return result;
}

void ValidateTransformedVoxels(const GeometryData& data,
                               const RigidTransform& transform) {
  for (const auto& voxel : data.voxels) {
    if (!SafePclPoint(transform.rotation * voxel.centroid +
                      transform.translation)) {
      throw std::invalid_argument(
          "Loop transform exceeds PCL arithmetic limits");
    }
  }
}

LoopVerification Solve(const GeometrySubmap& target,
                       const GeometrySubmap& source,
                       const LoopRegistrationOptions& options) {
  const auto prepared = PrepareGeometry(target, source, options);
  LoopVerification result;
  result.target = Provenance(target);
  result.source = Provenance(source);
  result.pose_version = target.pose_version;
  result.frame_count = target.frame_count;
  result.optimized_frame_count = target.optimized_frame_count;
  result.initial_target_from_source = pose_math::Compose(
      pose_math::Inverse(Projected(target.parent_from_submap)),
      Projected(source.parent_from_submap));
  result.target_points = prepared.target ? prepared.target->size() : 0;
  result.source_points = prepared.source ? prepared.source->size() : 0;
  if (prepared.status != LoopStatus::kVerified) {
    result.status = prepared.status;
    return result;
  }
  const auto& target_cloud = prepared.target;
  const auto& source_cloud = prepared.source;
  // Check both directions before entering either solver.
  const auto forward_guess =
      InitialGuess(*source_cloud, result.initial_target_from_source);
  const auto reverse_guess = InitialGuess(
      *target_cloud, pose_math::Inverse(result.initial_target_from_source));
  result.forward = Align(target_cloud, source_cloud, forward_guess, options);
  if (result.forward.iterations >= options.max_iterations) {
    result.status = LoopStatus::kIterationLimit;
    return result;
  }
  if (!result.forward.transform) return result;
  result.reverse = Align(source_cloud, target_cloud, reverse_guess, options);
  if (result.reverse.iterations >= options.max_iterations) {
    result.status = LoopStatus::kIterationLimit;
    return result;
  }
  if (!result.reverse.transform) return result;
  result.forward.metrics = loop_internal::EvaluateMetrics(
      *target.data, *source.data, *result.forward.transform, options);
  result.reverse.metrics = loop_internal::EvaluateMetrics(
      *source.data, *target.data, *result.reverse.transform, options);
  result.transform_errors = loop_internal::TransformErrors(
      result.initial_target_from_source, *result.forward.transform,
      *result.reverse.transform);
  result.status = loop_internal::ClassifyQuality(
      *result.forward.metrics, *result.reverse.metrics,
      *result.transform_errors, options);
  if (result.status == LoopStatus::kVerified)
    result.target_from_source = result.forward.transform;
  return result;
}

}  // namespace

void ValidateLoopRegistrationOptions(const LoopRegistrationOptions& o) {
  if (!Positive(o.voxel_size_m) || !Positive(o.correspondence_distance_m) ||
      !Positive(o.correspondence_distance_m * o.correspondence_distance_m) ||
      !Positive(o.translation_epsilon) || !Positive(o.rotation_epsilon) ||
      !Positive(o.match_distance_m) ||
      !Positive(o.match_distance_m * o.match_distance_m) ||
      !Positive(o.max_rmse_m) || !Positive(o.max_p95_m) ||
      !Positive(o.max_cycle_translation_m) ||
      !Positive(o.max_cycle_rotation_deg) || !Positive(o.max_correction_m) ||
      !Positive(o.max_correction_deg) || !Positive(o.min_coverage) ||
      o.min_coverage > 1 || !Positive(o.min_eigenvalue_ratio) ||
      o.min_eigenvalue_ratio > 1 || o.max_cycle_rotation_deg > 180 ||
      o.max_correction_deg > 180 || o.neighbors < 4 ||
      o.min_points < static_cast<std::size_t>(o.neighbors) ||
      o.max_points < o.min_points || o.max_points > 20000 || !o.max_voxels ||
      o.max_voxels > 100000 || !o.min_inliers || !o.min_normal_pairs ||
      o.max_iterations <= 0 || o.max_optimizer_iterations <= 0) {
    throw std::invalid_argument("Invalid loop registration options");
  }
}

LoopVerification VerifyLoop(const GeometrySubmap& target,
                            const GeometrySubmap& source,
                            const LoopRegistrationOptions& options) {
  const auto start = std::chrono::steady_clock::now();
  auto result = Solve(target, source, options);
  result.elapsed_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count();
  return result;
}

LoopGeometryCheck EvaluateLoopGeometry(const GeometrySubmap& target,
                                       const GeometrySubmap& source,
                                       const RigidTransform& target_from_source,
                                       const LoopRegistrationOptions& options) {
  const auto transform = Projected(target_from_source);
  const auto prepared = PrepareGeometry(target, source, options);
  LoopGeometryCheck result;
  result.status = prepared.status;
  if (result.status != LoopStatus::kVerified) return result;
  const auto inverse = pose_math::Inverse(transform);
  // Quality uses all original voxels, so validate their transformed positions,
  // not just the downsampled means, before any float nearest-neighbor search.
  ValidateTransformedVoxels(*source.data, transform);
  ValidateTransformedVoxels(*target.data, inverse);
  result.forward = loop_internal::EvaluateMetrics(*target.data, *source.data,
                                                  transform, options);
  result.reverse = loop_internal::EvaluateMetrics(*source.data, *target.data,
                                                  inverse, options);
  result.status = loop_internal::ClassifyQuality(result.forward, result.reverse,
                                                 std::nullopt, options);
  return result;
}

const char* LoopStatusName(LoopStatus status) {
  switch (status) {
    case LoopStatus::kVerified:
      return "verified";
    case LoopStatus::kInsufficientPoints:
      return "insufficient_points";
    case LoopStatus::kResourceLimit:
      return "resource_limit";
    case LoopStatus::kNotConverged:
      return "not_converged";
    case LoopStatus::kIterationLimit:
      return "iteration_limit";
    case LoopStatus::kLowOverlap:
      return "low_overlap";
    case LoopStatus::kHighResidual:
      return "high_residual";
    case LoopStatus::kInconsistentTransform:
      return "inconsistent_transform";
    case LoopStatus::kLargeCorrection:
      return "large_correction";
    case LoopStatus::kDegenerate:
      return "degenerate";
  }
  throw std::invalid_argument("Unknown loop status");
}

}  // namespace livo_atlas
