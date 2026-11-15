#ifndef LIVO_ATLAS_MAPPING_GEOMETRY_MAP_BUILDER_H_
#define LIVO_ATLAS_MAPPING_GEOMETRY_MAP_BUILDER_H_

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "livo_atlas/mapping/geometry_map_snapshot.h"
#include "livo_atlas/mapping/geometry_mapper.h"

namespace livo_atlas {

struct GeometryMapBuilderStats {
  std::uint64_t cache_hits = 0, cache_evictions = 0, cache_invalidations = 0;
  std::uint64_t cached_points = 0, peak_cached_points = 0;
  GeometryCacheStats geometry;
};

// The caller serializes access and keeps the reader's committed data immutable.
// Cached fragments own only local display fields and their actual sources.
class GeometryMapBuilder {
 public:
  GeometryMapBuilder(GeometryOptions geometry, GeometryMapOptions options,
                     GeometryMapper::CloudReader reader);

  GeometryMapSnapshot Build(const std::vector<OnlineSubmap>& groups,
                            std::shared_ptr<const PoseSnapshot> poses);
  GeometryMapBuilderStats GetStats() const;

 private:
  struct LocalPoint {
    GeometryVoxelKey key;
    Eigen::Vector3d centroid;
    double intensity = 0;
  };

  struct Fragment {
    std::vector<LocalPoint> points;
    std::vector<GeometrySource> sources;
    std::uint64_t representation_revision = 0;
    std::uint64_t built_pose_version = 0;
    std::uint64_t input_point_count = 0;
  };

  struct Entry {
    std::optional<Fragment> fragment;
    std::uint64_t last_access = 0;
    // Retained even when the compact fragment is evicted: the mapper may still
    // have a full representation built under an earlier pose version.
    std::uint64_t representation_revision = 0;
    std::uint64_t built_pose_version = 0;
  };

  bool Matches(const Fragment& fragment, const OnlineSubmap& group,
               const PoseSnapshot& poses) const;
  void EraseFragment(std::size_t id);
  void EvictFor(std::size_t points);
  void Touch(std::size_t id);

  GeometryOptions geometry_options_;
  GeometryMapOptions options_;
  GeometryMapper mapper_;
  std::vector<Entry> entries_;
  GeometryMapBuilderStats stats_;
  std::uint64_t access_clock_ = 0;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_MAPPING_GEOMETRY_MAP_BUILDER_H_
