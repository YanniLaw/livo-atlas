#ifndef LIVO_ATLAS_CORE_LIVO_CONFIG_H_
#define LIVO_ATLAS_CORE_LIVO_CONFIG_H_

#include <string>

#include "livo_atlas/core/system_options.h"
#include "livo_atlas/core/parameter_reader.h"

namespace livo_atlas {

struct LivoConfig {
  SystemOptions system;
  std::string lidar_topic = "/livox/lidar";
  std::string imu_topic = "/livox/imu";
  std::string image_topic = "/left_camera/image";
  double image_time_offset = 0;
  double lidar_time_offset = 0;
};

// Reads the shared schema and validates it without opening output files.
LivoConfig LoadLivoConfig(ParameterReader& reader);

}  // namespace livo_atlas
#endif  // LIVO_ATLAS_CORE_LIVO_CONFIG_H_
