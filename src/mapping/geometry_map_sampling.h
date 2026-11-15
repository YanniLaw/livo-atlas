#ifndef LIVO_ATLAS_SRC_MAPPING_GEOMETRY_MAP_SAMPLING_H_
#define LIVO_ATLAS_SRC_MAPPING_GEOMETRY_MAP_SAMPLING_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "livo_atlas/mapping/geometry_map_snapshot.h"

namespace livo_atlas::geometry_map_detail {

struct GeometryMapRank {
  std::uint64_t hash = 0;
  std::uint64_t submap_id = 0;
  GeometryVoxelKey key;
  bool operator<(const GeometryMapRank& other) const;
};

struct GeometryMapCandidate {
  std::uint64_t submap_id = 0;
  GeometryVoxelKey key;
  GeometryMapPoint point;
  std::uint64_t priority = 0;
};

GeometryMapRank MakeGeometryMapRank(std::uint64_t submap_id,
                                    const GeometryVoxelKey& key);
std::size_t GeometryMapPointLimit(const GeometryMapOptions& options,
                                  std::uint64_t submap_count);

class GeometryMapSampler {
 public:
  explicit GeometryMapSampler(std::size_t capacity);
  void Add(const GeometryMapCandidate& candidate);
  std::vector<GeometryMapCandidate> Take();

 private:
  std::size_t capacity_;
  std::vector<GeometryMapCandidate> candidates_;
};

}  // namespace livo_atlas::geometry_map_detail

#endif  // LIVO_ATLAS_SRC_MAPPING_GEOMETRY_MAP_SAMPLING_H_
