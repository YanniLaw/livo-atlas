#ifndef LIVO_ATLAS_CORE_GLOBAL_OUTPUT_OPTIONS_H_
#define LIVO_ATLAS_CORE_GLOBAL_OUTPUT_OPTIONS_H_

#include <string>

#include "livo_atlas/core/geometry_map_output_options.h"
#include "livo_atlas/core/trajectory_output_options.h"

namespace livo_atlas {

struct GlobalOutputOptions {
  bool enabled = false;
  bool publish_tf = true;
  std::string map_frame = "map";
  TrajectoryOutputOptions trajectory;
  GeometryMapOutputOptions geometry;
};

// Validates the output's own frame contract when enabled. Storage requirements
// belong to the shared application configuration, not a standalone publisher.
void ValidateGlobalOutputOptions(const GlobalOutputOptions& options);

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_CORE_GLOBAL_OUTPUT_OPTIONS_H_
