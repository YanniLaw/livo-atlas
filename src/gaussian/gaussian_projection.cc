#include "livo_atlas/gaussian/gaussian_projection.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

#include "gaussian/projection_envelope.h"
#include "gaussian/render_validation.h"
#include "mapping/pose_math.h"

namespace livo_atlas {
namespace {

void Require(bool condition, const char* detail) {
  if (!condition) throw std::invalid_argument(detail);
}

GaussianEnvelope Uncertain(const GaussianCamera& camera) {
  return {0, 0, camera.width, camera.height, true};
}

double Radius(double focal, double center, int size, double z, double scale) {
  // gsplat1.5.3 clamps the pinhole Jacobian outside its padded image plane.
  // The largest 3D axis scale bounds every rotated covariance eigenvalue.
  const double limit =
      std::max(std::abs((size - center) / focal + .15 * size / focal),
               std::abs(center / focal + .15 * size / focal));
  const double sigma = scale * (focal / z);
  return std::ceil(3.33 * std::hypot(sigma, sigma * limit, std::sqrt(.3))) + 2;
}

bool UnstableProjection(const Eigen::Vector3d& low, const Eigen::Vector3d& high,
                        const GaussianCamera& camera, double scale, double ru,
                        double rv) {
  constexpr double kFloatSafe = std::numeric_limits<float>::max() / 64.;
  const double covariance_safe = std::sqrt(kFloatSafe) / 64.;
  const double radius_safe = std::numeric_limits<int>::max() / 4.;
  if (scale * scale > kFloatSafe || ru > radius_safe || rv > radius_safe)
    return true;
  for (int axis = 0; axis < 2; ++axis) {
    const double focal = camera.K(axis, axis);
    const double center = camera.K(axis, 2) + .5;
    const double size = axis == 0 ? camera.width : camera.height;
    const double limit =
        std::max(std::abs((size - center) / focal) + .15 * size / focal,
                 std::abs(center / focal) + .15 * size / focal);
    const double position = std::max(std::abs(low[axis]), std::abs(high[axis]));
    const double jacobian = focal / low.z();
    const double tx = high.z() * limit;
    const double sigma = scale * jacobian;
    const double variance = sigma * sigma * (1 + limit * limit) + .3;
    // Pinned gsplat uses float products before division, J*cov*J^T,
    // a quadratic determinant, and finally int32 footprint radii.
    for (const double value :
         {limit, tx, jacobian, focal * tx, focal * tx / (low.z() * low.z()),
          focal * position, focal * position / low.z() + std::abs(center)})
      if (!std::isfinite(value) || value > kFloatSafe) return true;
    if (!std::isfinite(variance) || variance > covariance_safe) return true;
  }
  return false;
}

std::uint64_t UnionArea(const std::vector<GaussianEnvelope>& rectangles) {
  std::vector<int> edges;
  for (const auto& r : rectangles) {
    edges.push_back(r.left);
    edges.push_back(r.right);
  }
  std::sort(edges.begin(), edges.end());
  edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
  std::uint64_t area = 0;
  for (std::size_t i = 1; i < edges.size(); ++i) {
    std::vector<std::pair<int, int>> spans;
    for (const auto& r : rectangles)
      if (r.left < edges[i] && r.right > edges[i - 1])
        spans.emplace_back(r.top, r.bottom);
    std::sort(spans.begin(), spans.end());
    int length = 0, end = 0;
    for (const auto& span : spans) {
      length += std::max(0, span.second - std::max(end, span.first));
      end = std::max(end, span.second);
    }
    area += static_cast<std::uint64_t>(edges[i] - edges[i - 1]) * length;
  }
  return area;
}

double Distance(const std::vector<GaussianExtent>& extents,
                const RigidTransform& camera_from_submap) {
  const Eigen::Vector3d center =
      -camera_from_submap.rotation.transpose() * camera_from_submap.translation;
  double distance = std::numeric_limits<double>::max();
  if (!center.allFinite()) return distance;
  for (const auto& extent : extents) {
    const Eigen::Vector3d delta =
        center - center.cwiseMax(extent.min).cwiseMin(extent.max);
    const double value = std::hypot(delta.x(), delta.y(), delta.z());
    if (std::isfinite(value)) distance = std::min(distance, value);
  }
  return distance;
}

double CoordinateBound(const RigidTransform& query,
                       const RigidTransform& parent,
                       const RigidTransform& camera) {
  long double sum = 0;
  for (const auto* pose : {&query, &parent, &camera})
    for (int i = 0; i < 3; ++i)
      sum += std::abs(static_cast<long double>(pose->translation[i]));
  return sum > std::numeric_limits<double>::max()
             ? std::numeric_limits<double>::infinity()
             : static_cast<double>(sum);
}

}  // namespace

GaussianEnvelope ProjectGaussianEnvelope(
    const GaussianExtent& extent, const GaussianCamera& camera,
    const RigidTransform& camera_from_submap, double coordinate_abs_upper) {
  constexpr double kFloatMax = std::numeric_limits<float>::max();
  const double fx = camera.K(0, 0), fy = camera.K(1, 1);
  if (!std::isfinite(coordinate_abs_upper) ||
      camera.K.cwiseAbs().maxCoeff() > kFloatMax ||
      fx < std::numeric_limits<float>::min() ||
      fy < std::numeric_limits<float>::min())
    return Uncertain(camera);
  Eigen::Vector3d low =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d high = -low;
  for (int i = 0; i < 8; ++i) {
    const Eigen::Vector3d corner{i & 1 ? extent.max.x() : extent.min.x(),
                                 i & 2 ? extent.max.y() : extent.min.y(),
                                 i & 4 ? extent.max.z() : extent.min.z()};
    const Eigen::Vector3d point =
        camera_from_submap.rotation * corner + camera_from_submap.translation;
    if (!point.allFinite()) return Uncertain(camera);
    low = low.cwiseMin(point);
    high = high.cwiseMax(point);
  }
  const double means_upper =
      extent.min.cwiseAbs().cwiseMax(extent.max.cwiseAbs()).sum();
  const double allowance = 64 * std::numeric_limits<float>::epsilon() *
                           (1 + means_upper + coordinate_abs_upper);
  low.array() -= allowance;
  high.array() += allowance;
  if (!low.allFinite() || !high.allFinite() ||
      std::max(low.cwiseAbs().maxCoeff(), high.cwiseAbs().maxCoeff()) >
          kFloatMax)
    return Uncertain(camera);
  // Clip the center box before projection, including a box crossing the near
  // plane. Dropping its negative-z corners would lose real screen support.
  low.z() = std::max(low.z(), .01);
  high.z() = std::min(high.z(), 1e10);
  if (low.z() > high.z()) return {};
  const double cx = camera.K(0, 2) + .5, cy = camera.K(1, 2) + .5;
  const double scale = extent.max_scale * (1 + 1e-4);
  const double ru = Radius(fx, cx, camera.width, low.z(), scale);
  const double rv = Radius(fy, cy, camera.height, low.z(), scale);
  if (!std::isfinite(ru) || !std::isfinite(rv) ||
      UnstableProjection(low, high, camera, scale, ru, rv))
    return Uncertain(camera);
  double min_u = std::numeric_limits<double>::infinity(), min_v = min_u;
  double max_u = -min_u, max_v = -min_u;
  for (int i = 0; i < 8; ++i) {
    const double z = i & 4 ? high.z() : low.z();
    const double u = fx * ((i & 1 ? high.x() : low.x()) / z) + cx;
    const double v = fy * ((i & 2 ? high.y() : low.y()) / z) + cy;
    if (!std::isfinite(u) || !std::isfinite(v)) return Uncertain(camera);
    min_u = std::min(min_u, u);
    max_u = std::max(max_u, u);
    min_v = std::min(min_v, v);
    max_v = std::max(max_v, v);
  }
  min_u -= ru;
  max_u += ru;
  min_v -= rv;
  max_v += rv;
  if (!std::isfinite(min_u) || !std::isfinite(max_u) || !std::isfinite(min_v) ||
      !std::isfinite(max_v))
    return Uncertain(camera);
  // Clamp in floating point before integer conversion; large valid bounds
  // must never overflow an int and accidentally become an empty rectangle.
  const double width = camera.width, height = camera.height;
  return {static_cast<int>(std::floor(std::clamp(min_u, 0., width))),
          static_cast<int>(std::floor(std::clamp(min_v, 0., height))),
          static_cast<int>(std::ceil(std::clamp(max_u, 0., width))),
          static_cast<int>(std::ceil(std::clamp(max_v, 0., height))), false};
}

GaussianProjection ProjectGaussianExtents(
    const std::vector<GaussianExtent>& extents, const GaussianCamera& camera,
    const RigidTransform& camera_from_submap, double coordinate_abs_upper) {
  Require(!extents.empty() && extents.size() <= 10 &&
              !std::isnan(coordinate_abs_upper) && coordinate_abs_upper >= 0,
          "Invalid spatial projection budget or coordinate bound");
  ValidateRenderCamera(camera);
  ValidateRenderPose(camera_from_submap);
  for (const auto& extent : extents)
    Require(extent.min.allFinite() && extent.max.allFinite() &&
                (extent.min.array() <= extent.max.array()).all() &&
                std::isfinite(extent.max_scale) && extent.max_scale > 0,
            "Invalid Gaussian extent for projection");
  GaussianProjection result;
  result.distance = Distance(extents, camera_from_submap);
  std::vector<GaussianEnvelope> rectangles;
  for (const auto& extent : extents) {
    const auto rectangle = ProjectGaussianEnvelope(
        extent, camera, camera_from_submap, coordinate_abs_upper);
    if (rectangle.uncertain)
      return {static_cast<std::uint64_t>(camera.width) * camera.height,
              result.distance, true};
    if (rectangle.left < rectangle.right && rectangle.top < rectangle.bottom)
      rectangles.push_back(rectangle);
  }
  result.candidate_pixels = UnionArea(rectangles);
  return result;
}

GaussianSpatialSelection SelectGaussianSpatialNeighbor(
    const std::vector<GaussianSpatialCandidate>& candidates,
    const GaussianCamera& camera, const RigidTransform& parent_from_body,
    std::uint64_t owner) {
  ValidateRenderCamera(camera);
  Require(candidates.size() <= 32 && owner < 512,
          "Spatial candidate budget exceeded");
  const auto camera_from_parent = pose_math::Compose(
      camera.camera_from_body, pose_math::Inverse(parent_from_body));
  GaussianSpatialSelection result;
  std::set<std::uint64_t> seen;
  for (const auto& candidate : candidates) {
    const auto id = candidate.submap_id;
    Require(id < 512 && seen.insert(id).second,
            "Invalid spatial candidate identity");
    if (id >= owner) continue;
    const auto transform =
        pose_math::Compose(camera_from_parent, candidate.parent_from_submap);
    GaussianProjection projection;
    if (!transform.rotation.allFinite() || !transform.translation.allFinite())
      projection = {static_cast<std::uint64_t>(camera.width) * camera.height,
                    std::numeric_limits<double>::max(), true};
    else
      projection = ProjectGaussianExtents(
          candidate.extents, camera, transform,
          CoordinateBound(parent_from_body, candidate.parent_from_submap,
                          camera.camera_from_body));
    if (!projection.candidate_pixels) continue;
    ++result.candidate_count;
    const auto& best = result.projection;
    if (!result.submap_id ||
        projection.candidate_pixels > best.candidate_pixels ||
        (projection.candidate_pixels == best.candidate_pixels &&
         (projection.distance < best.distance ||
          (projection.distance == best.distance && id < *result.submap_id)))) {
      result.submap_id = id;
      result.projection = projection;
    }
  }
  return result;
}

}  // namespace livo_atlas
