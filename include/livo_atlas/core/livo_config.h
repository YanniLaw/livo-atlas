#ifndef LIVO_ATLAS_CORE_LIVO_CONFIG_H_
#define LIVO_ATLAS_CORE_LIVO_CONFIG_H_

#include <string>

#include "livo_atlas/core/gaussian_render_output_options.h"
#include "livo_atlas/core/global_output_options.h"
#include "livo_atlas/core/online_gaussian_options.h"
#include "livo_atlas/core/parameter_reader.h"
#include "livo_atlas/core/system_options.h"

namespace livo_atlas {

struct LivoConfig {
  SystemOptions system;
  std::string lidar_topic = "/livox/lidar";
  std::string imu_topic = "/livox/imu";
  std::string image_topic = "/left_camera/image";
  int image_queue_size = 1;
  double image_time_offset = 0;
  double lidar_time_offset = 0;
  GlobalOutputOptions global_output;
  OnlineGaussianOptions gaussian;
  GaussianRenderOutputOptions gaussian_output;
  int executor_threads = 4;
  int path_max_poses = 2000;
};

// Reads the shared schema and validates it without opening output files.
LivoConfig LoadLivoConfig(ParameterReader& reader);

}  // namespace livo_atlas
#endif  // LIVO_ATLAS_CORE_LIVO_CONFIG_H_
