#ifndef LIVO_ATLAS_FRONTEND_FRONTEND_H_
#define LIVO_ATLAS_FRONTEND_FRONTEND_H_

#include <memory>

#include "livo_atlas/core/state.h"
#include "livo_atlas/frontend/frontend_options.h"
#include "livo_atlas/frontend/frontend_result.h"
#include "livo_atlas/frontend/synchronized_measurements.h"

class VIOManager;
class VoxelMapManager;

namespace livo_atlas {

class Frontend {
 public:
  // Options must satisfy the existing calibration/image/grid constraints.
  explicit Frontend(const FrontendOptions& options);
  ~Frontend();
  Frontend(const Frontend&) = delete;
  Frontend& operator=(const Frontend&) = delete;

  // One executor calls this once after each successful synchronization.
  // measurements must be non-null; the IMU advances its last update time.
  FrontendResult Process(SynchronizedMeasurements* measurements);
  const State& state() const;
  bool imu_needs_initialization() const;
  // Returns zero when IMU is disabled or initialization has not completed.
  double imu_mean_acceleration_norm() const;

  // Transitional borrowed access for sequential ROS output until P2-03/P3.
  // Do not retain these past this frontend's lifetime or use asynchronously.
  VIOManager& visual();
  VoxelMapManager& local_map();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_FRONTEND_FRONTEND_H_
