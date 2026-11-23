#ifndef LIVO_ATLAS_SRC_ROS_COMMON_INPUT_DIAGNOSTICS_H_
#define LIVO_ATLAS_SRC_ROS_COMMON_INPUT_DIAGNOSTICS_H_

#include <cstdint>
#include <iomanip>

#include "common/timing.h"
#include "livo_atlas/common/logging.h"
#include "livo_atlas/core/livo_system.h"

namespace livo_atlas::ros_detail {

class InputTiming {
 public:
  InputTiming(bool enabled, const char* conversion_stage,
              const char* lock_wait_stage)
      : conversion_stage_(conversion_stage),
        lock_wait_stage_(lock_wait_stage),
        start_(StartTiming(enabled)) {}

  void Converted() {
    converted_ = StartTiming(start_ != SteadyClock::time_point{});
  }

  void LockAcquired() {
    acquired_ = StartTiming(start_ != SteadyClock::time_point{});
  }

  // Declare this before the lock guard so logging happens after unlocking.
  ~InputTiming() {
    if (converted_ != SteadyClock::time_point{}) {
      LogStageTiming(conversion_stage_, converted_ - start_);
    }
    if (acquired_ != SteadyClock::time_point{}) {
      LogStageTiming(lock_wait_stage_, acquired_ - converted_);
    }
  }

 private:
  const char* conversion_stage_;
  const char* lock_wait_stage_;
  SteadyClock::time_point start_, converted_, acquired_;
};

inline void LogPropagationInterval(SteadyClock::time_point* previous) {
  const auto now = SteadyClock::now();
  if (*previous != SteadyClock::time_point{}) {
    LogStageTiming("imu_propagation_interval", now - *previous);
  }
  *previous = now;
}

inline void LogInputDiagnostics(std::uint64_t lidar, std::uint64_t imu,
                                std::uint64_t image,
                                const StartupBufferStats& startup) {
  LOG(INFO) << "livo_atlas inputs: lidar=" << lidar << " imu=" << imu
            << " image=" << image;
  LOG(INFO) << "livo_atlas startup: imu_buffered=" << startup.buffered_imu
            << " image_buffered=" << startup.buffered_images
            << " imu_evicted=" << startup.evicted_imu
            << " image_evicted=" << startup.evicted_images
            << " imu_rejected=" << startup.rejected_imu;
}

inline void LogFrontendUpdate(UpdateKind kind, double elapsed) {
  LOG(INFO) << "livo_atlas update kind="
            << (kind == UpdateKind::kVisual ? "vio" : "lio")
            << " frontend_process_seconds=" << std::fixed
            << std::setprecision(9) << elapsed;
}

}  // namespace livo_atlas::ros_detail

#endif  // LIVO_ATLAS_SRC_ROS_COMMON_INPUT_DIAGNOSTICS_H_
