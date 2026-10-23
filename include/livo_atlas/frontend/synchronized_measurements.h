#ifndef LIVO_ATLAS_FRONTEND_SYNCHRONIZED_MEASUREMENTS_H_
#define LIVO_ATLAS_FRONTEND_SYNCHRONIZED_MEASUREMENTS_H_

#include <deque>

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
struct MeasurementGroup {
  double vio_time = 0.0;
  double lio_time = 0.0;
  std::deque<ImuMeasurement> imu;
  cv::Mat img;
};

struct SynchronizedMeasurements {
  double lidar_frame_beg_time = -0.0;
  double lidar_frame_end_time = 0.0;
  // The estimator advances this after processing, not the synchronizer.
  double last_lio_update_time = -1.0;
  PointCloud::Ptr lidar{new PointCloud};
  PointCloud::Ptr pcl_proc_cur{new PointCloud};
  PointCloud::Ptr pcl_proc_next{new PointCloud};
  std::deque<MeasurementGroup> measures;
  UpdateKind lio_vio_flg = UpdateKind::kWaiting;
  int lidar_scan_index_now = 0;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_FRONTEND_SYNCHRONIZED_MEASUREMENTS_H_
