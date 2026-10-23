#ifndef LIVO_ATLAS_FRONTEND_FRONTEND_OPTIONS_H_
#define LIVO_ATLAS_FRONTEND_FRONTEND_OPTIONS_H_

#include "livo_atlas/frontend/calibration.h"
#include "livo_atlas/mapping/voxel_map_options.h"

namespace livo_atlas {

struct ImuOptions {
  bool enabled = false;
  bool estimate_gravity = true;
  bool estimate_bias = true;
  int initialization_frames = 3;
  double gyroscope_covariance = 1.0;
  double accelerometer_covariance = 1.0;
  double inverse_exposure_covariance = 0.2;
};

struct VisualOptions {
  bool normal_enabled = true;
  bool inverse_composition = false;
  int max_iterations = 5;
  double image_point_covariance = 100;
  bool raycast = false;
  bool estimate_exposure = true;
  int grid_size = 5;
  int grid_rows = 17;
  int patch_pyramid_levels = 3;
  int patch_size = 8;
  double outlier_threshold = 1000;
  bool colmap_output = false;
  double plot_time = -10;
  int plot_frame_count = 6;
};

struct FrontendOptions {
  Calibration calibration;
  VoxelMapOptions voxel_map;
  ImuOptions imu;
  VisualOptions visual;
  double surface_filter_size = 0.5;
  bool align_gravity = false;
  bool dense_cloud = false;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_FRONTEND_FRONTEND_OPTIONS_H_
