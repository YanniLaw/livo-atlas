#ifndef LIVO_ATLAS_SRC_ROS_COMMON_INPUT_DIAGNOSTICS_H_
#define LIVO_ATLAS_SRC_ROS_COMMON_INPUT_DIAGNOSTICS_H_

#include <cstdint>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <utility>

#include "common/timing.h"
#include "livo_atlas/common/logging.h"
#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/ros_common/sensor_input.h"

namespace livo_atlas::ros_detail {

template <typename Sample, typename Converter>
void ReceiveInput(SensorInput& input, Sensor sensor, Converter convert) {
  constexpr const char* kStages[] = {"lidar_conversion", "imu_conversion",
                                     "image_conversion"};
  input.RecordReceived(sensor);
  const auto start = StartTiming(input.stage_timing());
  Sample sample;
  std::string error;
  try {
    error = convert(&sample);
  } catch (const std::invalid_argument& invalid) {
    error = invalid.what();
  }
  const auto converted = StartTiming(input.stage_timing());
  if (!error.empty()) {
    input.RecordConversionRejection(sensor);
    LOG(WARNING) << error;
  } else {
    input.Push(std::move(sample));
  }
  if (input.stage_timing()) {
    LogStageTiming(kStages[static_cast<std::size_t>(sensor)],
                   converted - start);
  }
}

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

inline void LogFrontendUpdate(UpdateKind kind, double elapsed, bool published) {
  LOG(INFO) << "livo_atlas update kind="
            << (kind == UpdateKind::kVisual ? "vio" : "lio")
            << " frontend_process_seconds=" << std::fixed
            << std::setprecision(9) << elapsed
            << " ros_published=" << (published ? 1 : 0);
}

}  // namespace livo_atlas::ros_detail

#endif  // LIVO_ATLAS_SRC_ROS_COMMON_INPUT_DIAGNOSTICS_H_
