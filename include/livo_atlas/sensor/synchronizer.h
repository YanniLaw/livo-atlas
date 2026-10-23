#ifndef LIVO_ATLAS_SENSOR_SYNCHRONIZER_H_
#define LIVO_ATLAS_SENSOR_SYNCHRONIZER_H_

#include <deque>

#include "livo_atlas/frontend/synchronized_measurements.h"
#include "livo_atlas/sensor/measurements.h"

namespace livo_atlas {

struct SensorBuffers {
  std::deque<PointCloudMeasurement> point_clouds;
  std::deque<ImuMeasurement> imu;
  std::deque<ImageMeasurement> images;
  // LiDAR-inertial coverage time before any final adapter timestamp rounding.
  // An offline reader can use the latest IMU sample's timestamp.
  double latest_imu_time = -1.0;
};

enum class SynchronizationResult {
  kWaiting,
  kReady,
  kImageDropped,
  kInvalidCloud
};

class SensorSynchronizer {
 public:
  explicit SensorSynchronizer(
      FrontendMode mode = FrontendMode::kLidarInertialVisual,
      double exposure_time = 0.0);

  // Both pointers must be non-null. The caller exclusively owns or locks all
  // buffer/output access. Queues contain measurements in acquisition order.
  // LIVO retains the legacy three-queue readiness gate, even for pending VIO;
  // this interface does not drain the final visual update at end of input.
  SynchronizationResult TrySynchronize(
      SensorBuffers* buffers, SynchronizedMeasurements* measurements) const;

 private:
  FrontendMode mode_;
  double exposure_time_;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_SENSOR_SYNCHRONIZER_H_
