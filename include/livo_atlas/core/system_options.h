#ifndef LIVO_ATLAS_CORE_SYSTEM_OPTIONS_H_
#define LIVO_ATLAS_CORE_SYSTEM_OPTIONS_H_
#include "livo_atlas/frontend/frontend_options.h"
#include "livo_atlas/io/livo_exporter.h"
#include "livo_atlas/mapping/cloud_colorizer.h"
#include "livo_atlas/mapping/keyframe_selector.h"
#include "livo_atlas/mapping/submap.h"
#include "livo_atlas/sensor/point_types.h"
namespace livo_atlas {
struct LidarPreprocessorOptions {
  int lidar_type = kLivox;
  int scan_lines = 6;
  int point_filter = 3;
  double blind = 0.01;
  bool features = false;
};
struct TimestampRange {
  double minimum = -2147483648.0;
  double maximum = 2147483648.0;
};
struct SystemOptions {
  FrontendOptions frontend;
  ExportOptions exporter;
  CloudColorizerOptions colorizer;
  KeyframeOptions keyframes;
  SubmapOptions submaps;
  LidarPreprocessorOptions preprocess;
  bool lidar_enabled = true;
  bool image_enabled = true;
  bool hilti = false;
  bool driver_fix = false;
  bool propagate_imu = false;
  double exposure_time = 0;
  double imu_time_offset = 0;
  TimestampRange imu_time_range;
};

// Validates configuration and fills derived output mode/calibration without
// I/O.
SystemOptions PrepareSystemOptions(SystemOptions options);
}  // namespace livo_atlas
#endif  // LIVO_ATLAS_CORE_SYSTEM_OPTIONS_H_
