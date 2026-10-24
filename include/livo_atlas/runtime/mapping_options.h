#ifndef LIVO_ATLAS_RUNTIME_MAPPING_OPTIONS_H_
#define LIVO_ATLAS_RUNTIME_MAPPING_OPTIONS_H_

#include <string>

#include "livo_atlas/runtime/options.h"
#include "livo_atlas/runtime/parameter_reader.h"

namespace livo_atlas {

struct MappingOptions {
  SessionOptions session;
  std::string lidar_topic = "/livox/lidar";
  std::string imu_topic = "/livox/imu";
  std::string image_topic = "/left_camera/image";
  double image_time_offset = 0;
  double lidar_time_offset = 0;
};

// Reads the shared schema and validates it without opening output files.
MappingOptions LoadMappingOptions(ParameterReader& reader);

}  // namespace livo_atlas
#endif  // LIVO_ATLAS_RUNTIME_MAPPING_OPTIONS_H_
