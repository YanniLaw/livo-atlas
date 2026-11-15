#ifndef LIVO_ATLAS_CORE_GEOMETRY_MAP_OUTPUT_OPTIONS_H_
#define LIVO_ATLAS_CORE_GEOMETRY_MAP_OUTPUT_OPTIONS_H_

#include "livo_atlas/mapping/geometry_map_snapshot.h"

namespace livo_atlas {

struct GeometryMapOutputOptions {
  bool enabled = false;
  double max_frequency_hz = 0.2;
  int max_keyframes = 20000;
  GeometryMapOptions map;
};

void ValidateGeometryMapOutputOptions(const GeometryMapOutputOptions& options);

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_CORE_GEOMETRY_MAP_OUTPUT_OPTIONS_H_
