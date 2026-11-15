#include "geometry_map_sampling.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace livo_atlas {

void ValidateGeometryMapOptions(const GeometryMapOptions& options) {
  if (options.max_points < 1 || options.max_points > 1000000 ||
      options.max_cached_points < 0 || options.max_cached_points > 3000000 ||
      options.max_message_bytes < 4096 || options.max_message_bytes > 67108864)
    throw std::invalid_argument("Invalid geometry map display budget");
}

namespace geometry_map_detail {
namespace {

GeometryMapRank Rank(const GeometryMapCandidate& candidate) {
  return {candidate.priority, candidate.submap_id, candidate.key};
}

bool RankLess(const GeometryMapCandidate& first,
              const GeometryMapCandidate& second) {
  return Rank(first) < Rank(second);
}

}  // namespace

bool GeometryMapRank::operator<(const GeometryMapRank& other) const {
  return std::tie(hash, submap_id, key.x, key.y, key.z) <
         std::tie(other.hash, other.submap_id, other.key.x, other.key.y,
                  other.key.z);
}

GeometryMapRank MakeGeometryMapRank(std::uint64_t submap_id,
                                    const GeometryVoxelKey& key) {
  std::uint64_t hash = 14695981039346656037ULL;
  for (const auto word :
       {submap_id, static_cast<std::uint64_t>(key.x),
        static_cast<std::uint64_t>(key.y), static_cast<std::uint64_t>(key.z)}) {
    hash = (hash ^ word) * 1099511628211ULL;
  }
  return {hash, submap_id, key};
}

std::size_t GeometryMapPointLimit(const GeometryMapOptions& options,
                                  std::uint64_t submap_count) {
  ValidateGeometryMapOptions(options);
  const auto available =
      static_cast<std::uint64_t>(options.max_message_bytes) - 4096;
  if (submap_count > available / 128) return 0;
  const auto points = (available - 128 * submap_count) / 16;
  return static_cast<std::size_t>(
      std::min<std::uint64_t>(options.max_points, points));
}

GeometryMapSampler::GeometryMapSampler(std::size_t capacity)
    : capacity_(capacity) {}

void GeometryMapSampler::Add(const GeometryMapCandidate& candidate) {
  if (!candidate.point.position.allFinite() ||
      !std::isfinite(candidate.point.intensity))
    throw std::invalid_argument("Nonfinite geometry map point");
  if (!capacity_) return;
  auto item = candidate;
  item.priority = MakeGeometryMapRank(item.submap_id, item.key).hash;
  if (candidates_.size() < capacity_) {
    candidates_.push_back(std::move(item));
    std::push_heap(candidates_.begin(), candidates_.end(), RankLess);
  } else if (RankLess(item, candidates_.front())) {
    std::pop_heap(candidates_.begin(), candidates_.end(), RankLess);
    candidates_.back() = std::move(item);
    std::push_heap(candidates_.begin(), candidates_.end(), RankLess);
  }
}

std::vector<GeometryMapCandidate> GeometryMapSampler::Take() {
  std::sort(candidates_.begin(), candidates_.end(),
            [](const auto& first, const auto& second) {
              return std::tie(first.submap_id, first.key.x, first.key.y,
                              first.key.z) <
                     std::tie(second.submap_id, second.key.x, second.key.y,
                              second.key.z);
            });
  auto result = std::move(candidates_);
  candidates_.clear();
  return result;
}

}  // namespace geometry_map_detail
}  // namespace livo_atlas
