#ifndef LIVO_ATLAS_MAPPING_GEOMETRY_MAPPER_H_
#define LIVO_ATLAS_MAPPING_GEOMETRY_MAPPER_H_

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "livo_atlas/mapping/geometry_submap.h"
#include "livo_atlas/mapping/keyframe.h"
#include "livo_atlas/sensor/point_types.h"

namespace livo_atlas {

// Calls are serialized by the owner. The reader accesses committed body clouds.
class GeometryMapper {
 public:
  using CloudReader = std::function<PointCloud::ConstPtr(std::uint64_t)>;

  GeometryMapper(GeometryOptions options, CloudReader cloud_reader);
  void Update(const std::vector<OnlineSubmap>& revisions,
              std::shared_ptr<const PoseSnapshot> poses,
              const Keyframe* new_keyframe = nullptr);
  GeometrySubmap GetSubmap(std::uint64_t id);
  GeometryCacheStats GetStats() const;

 private:
  struct Entry {
    OnlineSubmap membership;
    std::unique_ptr<GeometrySubmapBuilder> builder;
    std::shared_ptr<const GeometryData> data;
    std::vector<GeometrySource> built_sources;
    std::uint64_t representation_revision = 0;
    std::uint64_t last_access = 0;
  };

  void ResetTimings();
  void EvictFor(std::uint64_t id);
  void Build(std::uint64_t id, const Keyframe* new_keyframe);
  void Append(std::uint64_t id, const Keyframe& keyframe);

  GeometryOptions options_;
  CloudReader cloud_reader_;
  std::vector<Entry> entries_;
  std::shared_ptr<const PoseSnapshot> poses_;
  GeometryCacheStats stats_;
  std::uint64_t access_clock_ = 0;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_MAPPING_GEOMETRY_MAPPER_H_
