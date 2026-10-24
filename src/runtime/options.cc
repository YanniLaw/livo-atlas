#include <cmath>
#include <cstddef>
#include <stdexcept>

#include "livo_atlas/frontend/image_options.h"
#include "livo_atlas/runtime/options.h"

namespace livo_atlas {

SessionOptions PrepareSessionOptions(SessionOptions options) {
  const auto& front = options.frontend;
  const auto& imu = front.imu;
  const auto& visual = front.visual;
  const auto& map = front.voxel_map;
  const auto& pre = options.preprocess;
  const auto error = ValidateImageOptions(front.calibration.camera, visual);
  if (!error.empty()) throw std::invalid_argument(error);
  for (double value :
       {imu.gyroscope_covariance, imu.accelerometer_covariance,
        imu.inverse_exposure_covariance, visual.image_point_covariance,
        pre.blind, front.surface_filter_size, options.imu_time_offset,
        options.exposure_time, map.max_voxel_size_, map.planner_threshold_,
        map.sigma_num_, map.beam_err_, map.dept_err_, map.sliding_thresh}) {
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
      map.max_layer_ < 0 ||
      static_cast<std::size_t>(map.max_layer_) >= map.layer_init_num_.size() ||
      map.max_voxel_size_ <= 0 || map.max_points_num_ <= 0 ||
      map.max_iterations_ <= 0 || visual.max_iterations <= 0 ||
      map.half_map_size <= 0) {
    throw std::invalid_argument("invalid frontend or map bounds");
  }
  for (int count : map.layer_init_num_) {
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
  if (options.processor.scan_interval <= 0) {
    throw std::invalid_argument("publish/pub_scan_num must be positive");
  }
  const auto mode = options.image_enabled && options.lidar_enabled
                        ? FrontendMode::kLidarInertialVisual
                    : imu.enabled ? FrontendMode::kLidarInertial
                                  : FrontendMode::kLidarOnly;
  options.output.mode = options.processor.mode = mode;
  options.output.calibration = options.processor.calibration =
      front.calibration;
  options.processor.colmap = options.output.colmap;
  ValidateOutputOptions(options.output);
  return options;
}

}  // namespace livo_atlas
