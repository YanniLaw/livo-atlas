#ifndef LIVO_ATLAS_FRONTEND_FRONTEND_H_
#define LIVO_ATLAS_FRONTEND_FRONTEND_H_

#include <memory>

#include "livo_atlas/common/filter_state.h"
#include "livo_atlas/frontend/frontend_options.h"
#include "livo_atlas/frontend/frontend_result.h"
#include "livo_atlas/frontend/synchronized_measurements.h"

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
  const FilterState& state() const;
  bool imu_needs_initialization() const;
  // Returns zero when IMU is disabled or initialization has not completed.
  double imu_mean_acceleration_norm() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_FRONTEND_FRONTEND_H_
