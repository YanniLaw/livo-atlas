#ifndef LIVO_ATLAS_SRC_ROS_INPUT_DIAGNOSTICS_H_
#define LIVO_ATLAS_SRC_ROS_INPUT_DIAGNOSTICS_H_

#include <cstdint>
#include <iomanip>

#include "livo_atlas/common/logging.h"
#include "livo_atlas/core/livo_system.h"

namespace livo_atlas::ros_detail {

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

inline void LogUpdateTime(UpdateKind kind, double elapsed) {
  const char* stage =
      kind == UpdateKind::kVisual ? "VIO Time" : "LIO Mapping Time";
  LOG(INFO) << stage << "\n| Current Total Time | " << std::fixed
            << std::setprecision(6) << elapsed << " |";
}

}  // namespace livo_atlas::ros_detail

#endif  // LIVO_ATLAS_SRC_ROS_INPUT_DIAGNOSTICS_H_
