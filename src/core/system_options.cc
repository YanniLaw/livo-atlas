#include "livo_atlas/core/system_options.h"

#include <cmath>
#include <cstddef>
#include <stdexcept>

#include "livo_atlas/frontend/vision/image_options.h"
#include "livo_atlas/sensor/extrinsics.h"

namespace livo_atlas {

SystemOptions PrepareSystemOptions(SystemOptions options) {
  ValidateKeyframeOptions(options.keyframes);
  const auto& front = options.frontend;
  const auto& imu = front.imu;
  const auto& visual = front.visual;
  const auto& voxel_options = front.voxel_map;
  const auto& pre = options.preprocess;
  const auto error = ValidateImageOptions(front.calibration.camera, visual);
  if (!error.empty()) throw std::invalid_argument(error);
  for (const auto* transform : {&front.calibration.imu_from_lidar,
                                &front.calibration.camera_from_lidar}) {
    const auto transform_error = ValidateExtrinsic(*transform);
    if (!transform_error.empty()) throw std::invalid_argument(transform_error);
  }
  for (double value :
       {imu.gyroscope_covariance, imu.accelerometer_covariance,
        imu.inverse_exposure_covariance, visual.image_point_covariance,
        visual.outlier_threshold, visual.plot_time,
        options.colorizer.minimum_color_distance, pre.blind,
        front.surface_filter_size, options.imu_time_offset,
        options.exposure_time, voxel_options.max_voxel_size_,
        voxel_options.planner_threshold_, voxel_options.sigma_num_,
        voxel_options.beam_err_, voxel_options.dept_err_,
        voxel_options.sliding_thresh}) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("runtime configuration must be finite");
    }
  }
  if (visual.image_point_covariance <= 0 || imu.gyroscope_covariance < 0 ||
      imu.accelerometer_covariance < 0 || imu.inverse_exposure_covariance < 0) {
    throw std::invalid_argument("invalid image or IMU covariance");
  }
  if (pre.scan_lines <= 0 || pre.scan_lines > 128 || pre.point_filter <= 0 ||
      pre.blind < 0 || pre.lidar_type < kLivox || pre.lidar_type > kRobosense) {
    throw std::invalid_argument("invalid preprocess configuration");
  }
  if (front.surface_filter_size <= 0 || imu.initialization_frames <= 0 ||
      voxel_options.max_layer_ < 0 ||
      static_cast<std::size_t>(voxel_options.max_layer_) >=
          voxel_options.layer_init_num_.size() ||
      voxel_options.max_voxel_size_ <= 0 ||
      voxel_options.max_points_num_ <= 0 ||
      voxel_options.max_iterations_ <= 0 || visual.max_iterations <= 0 ||
      voxel_options.half_map_size <= 0) {
    throw std::invalid_argument("invalid frontend or map bounds");
  }
  for (int count : voxel_options.layer_init_num_) {
    if (count <= 0) {
      throw std::invalid_argument(
          "lio.layer_init_num must contain positive integers");
    }
  }
  const auto& range = options.imu_time_range;
  if (!std::isfinite(range.minimum) || !std::isfinite(range.maximum) ||
      range.minimum >= range.maximum ||
      std::floor(range.minimum) != range.minimum ||
      std::floor(range.maximum) != range.maximum) {
    throw std::invalid_argument("invalid IMU timestamp range");
  }
  if (options.colorizer.scan_interval <= 0) {
    throw std::invalid_argument("publish/pub_scan_num must be positive");
  }
  const auto mode = options.image_enabled && options.lidar_enabled
                        ? FrontendMode::kLidarInertialVisual
                    : imu.enabled ? FrontendMode::kLidarInertial
                                  : FrontendMode::kLidarOnly;
  options.exporter.mode = options.colorizer.mode = mode;
  options.exporter.calibration = front.calibration;
  options.colorizer.camera = front.calibration.camera;
  ValidateExportOptions(options.exporter);
  return options;
}

}  // namespace livo_atlas
