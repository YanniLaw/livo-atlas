#include "livo_atlas/mapping/geometry_submap.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

#include <Eigen/Eigenvalues>

namespace livo_atlas {
namespace {

std::int64_t VoxelIndex(double coordinate, double size) {
  const double index = std::floor(coordinate / size);
  const double limit = std::ldexp(1.0, 63);
  if (!std::isfinite(index) || index < -limit || index >= limit) {
    throw std::invalid_argument("Geometry voxel coordinate out of range");
  }
  return static_cast<std::int64_t>(index);
}

}  // namespace

void ValidateGeometryOptions(const GeometryOptions& options) {
  const double h2 = options.voxel_size * options.voxel_size;
  const double rms2 = options.max_plane_rms * options.max_plane_rms;
  if (!std::isfinite(options.voxel_size) || options.voxel_size <= 0 ||
      !std::isfinite(h2) || h2 <= 0 || 0.01 * h2 <= 0 || 1e-12 * h2 <= 0 ||
      !std::isfinite(options.max_plane_rms) || options.max_plane_rms <= 0 ||
      !std::isfinite(rms2) || rms2 <= 0 ||
      !std::isfinite(options.max_plane_ratio) || options.max_plane_ratio <= 0 ||
      options.max_plane_ratio >= 1 || options.min_points_per_voxel < 3 ||
      options.max_voxels_per_submap <= 0 ||
      options.max_points_per_keyframe <= 0 ||
      options.max_resident_submaps < 2) {
    throw std::invalid_argument("Invalid geometry options");
  }
}

bool GeometryVoxelKey::operator==(const GeometryVoxelKey& other) const {
  return x == other.x && y == other.y && z == other.z;
}

bool GeometryVoxelKey::operator<(const GeometryVoxelKey& other) const {
  return std::tie(x, y, z) < std::tie(other.x, other.y, other.z);
}

std::size_t GeometrySubmapBuilder::KeyHash::operator()(
    const GeometryVoxelKey& key) const {
  std::uint64_t hash = 14695981039346656037ull;
  for (const auto value : {key.x, key.y, key.z}) {
    hash ^= static_cast<std::uint64_t>(value);
    hash *= 1099511628211ull;
  }
  return static_cast<std::size_t>(hash);
}

GeometrySubmapBuilder::GeometrySubmapBuilder(GeometryOptions options)
    : options_(options) {
  ValidateGeometryOptions(options_);
}

void GeometrySubmapBuilder::AddKeyframe(
    std::uint64_t id, const PointCloud& body_cloud,
    const RigidTransform& submap_from_body) {
  if (sources_.size() >= kMaxGeometryKeyframes ||
      (!sources_.empty() && id <= sources_.back().keyframe_id) ||
      body_cloud.empty() ||
      body_cloud.size() >
          static_cast<std::size_t>(options_.max_points_per_keyframe) ||
      body_cloud.size() >
          std::numeric_limits<std::uint64_t>::max() - input_point_count_) {
    throw std::invalid_argument("Invalid geometry keyframe " +
                                std::to_string(id));
  }
  const auto error = ValidateExtrinsic(submap_from_body);
  if (!error.empty()) throw std::invalid_argument("Geometry pose: " + error);

  // Candidate state prevents a malformed point from partially changing a block.
  auto candidate = voxels_;
  for (const auto& point : body_cloud) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || !std::isfinite(point.intensity)) {
      throw std::invalid_argument("Nonfinite point in geometry keyframe " +
                                  std::to_string(id));
    }
    const Eigen::Vector3d p =
        submap_from_body.rotation * Eigen::Vector3d(point.x, point.y, point.z) +
        submap_from_body.translation;
    const GeometryVoxelKey key{VoxelIndex(p.x(), options_.voxel_size),
                               VoxelIndex(p.y(), options_.voxel_size),
                               VoxelIndex(p.z(), options_.voxel_size)};
    auto found = candidate.find(key);
    if (found == candidate.end()) {
      if (candidate.size() >=
          static_cast<std::size_t>(options_.max_voxels_per_submap)) {
        throw std::invalid_argument(
            "Geometry voxel limit exceeded by keyframe " + std::to_string(id));
      }
      found = candidate.emplace(key, Moments{}).first;
    }
    auto& m = found->second;
    if (m.count == std::numeric_limits<std::uint64_t>::max()) {
      throw std::invalid_argument("Geometry point count overflow");
    }
    ++m.count;
    const Eigen::Vector3d delta = p - m.mean;
    m.mean += delta / static_cast<double>(m.count);
    m.m2 += delta * (p - m.mean).transpose();
    m.mean_intensity += (point.intensity - m.mean_intensity) / m.count;
    if (!m.mean.allFinite() || !m.m2.allFinite() ||
        !std::isfinite(m.mean_intensity)) {
      throw std::invalid_argument("Geometry statistics overflow");
    }
    if (m.ids.empty() || m.ids.back() != id) m.ids.push_back(id);
  }
  auto sources = sources_;
  sources.push_back({id, submap_from_body});
  voxels_.swap(candidate);
  sources_.swap(sources);
  input_point_count_ += body_cloud.size();
}

std::shared_ptr<const GeometryData> GeometrySubmapBuilder::Snapshot() const {
  auto result = std::make_shared<GeometryData>();
  result->options = options_;
  result->sources = sources_;
  result->input_point_count = input_point_count_;
  result->voxels.reserve(voxels_.size());
  const double h2 = options_.voxel_size * options_.voxel_size;
  for (const auto& item : voxels_) {
    const auto& m = item.second;
    GeometryVoxel voxel;
    voxel.key = item.first;
    voxel.point_count = m.count;
    voxel.centroid = m.mean;
    voxel.spatial_covariance = (m.m2 + m.m2.transpose()) / (2.0 * m.count);
    voxel.mean_intensity = m.mean_intensity;
    voxel.keyframe_ids = m.ids;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(
        voxel.spatial_covariance);
    if (solver.info() != Eigen::Success ||
        solver.eigenvalues()[0] < -1e-12 * h2) {
      throw std::runtime_error("Invalid geometry covariance");
    }
    const Eigen::Vector3d values = solver.eigenvalues().cwiseMax(0.0);
    if (m.count >= static_cast<std::uint64_t>(options_.min_points_per_voxel) &&
        values[1] >= 0.01 * h2 &&
        values[0] <= options_.max_plane_rms * options_.max_plane_rms &&
        values[0] / values[1] <= options_.max_plane_ratio) {
      Eigen::Vector3d normal = solver.eigenvectors().col(0);
      Eigen::Index axis;
      normal.cwiseAbs().maxCoeff(&axis);
      if (normal[axis] < 0) normal = -normal;
      voxel.normal = normal;
    }
    result->voxels.push_back(std::move(voxel));
  }
  std::sort(result->voxels.begin(), result->voxels.end(),
            [](const GeometryVoxel& a, const GeometryVoxel& b) {
              return a.key < b.key;
            });
  return result;
}

}  // namespace livo_atlas
