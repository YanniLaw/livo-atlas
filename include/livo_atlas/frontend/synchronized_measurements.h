#ifndef LIVO_ATLAS_FRONTEND_SYNCHRONIZED_MEASUREMENTS_H_
#define LIVO_ATLAS_FRONTEND_SYNCHRONIZED_MEASUREMENTS_H_

#include <deque>
#include <opencv2/core/mat.hpp>

#include "livo_atlas/sensor/measurements.h"

namespace livo_atlas {

enum class FrontendMode {
  kLidarOnly = 0,
  kLidarInertial = 1,
  kLidarInertialVisual = 2
};

enum class UpdateKind {
  kWaiting = 0,
  kVisual = 1,
  kLidarInertial = 2,
  kLidarOnly = 3
};

// Field names remain compatible with the existing IMU/LIO/VIO consumers.
// Times are seconds; point curvature holds offsets in milliseconds.
struct UpdateMeasurements {
  double visual_update_time = 0.0;
  double lidar_update_time = 0.0;
  std::deque<Imu> imu;
  cv::Mat image;
};

struct SynchronizedMeasurements {
  double scan_start_time = -0.0;
  double scan_end_time = 0.0;
  // The estimator advances this after processing, not the synchronizer.
  double last_lidar_update_time = -1.0;
  PointCloud::Ptr lidar_cloud{new PointCloud};
  PointCloud::Ptr current_cloud{new PointCloud};
  PointCloud::Ptr next_cloud{new PointCloud};
  std::deque<UpdateMeasurements> updates;
  UpdateKind update_kind = UpdateKind::kWaiting;
  int scan_cursor = 0;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_FRONTEND_SYNCHRONIZED_MEASUREMENTS_H_
