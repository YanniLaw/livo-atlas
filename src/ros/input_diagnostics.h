#ifndef LIVO_ATLAS_SRC_ROS_INPUT_DIAGNOSTICS_H_
#define LIVO_ATLAS_SRC_ROS_INPUT_DIAGNOSTICS_H_

#include <cstdint>
#include <cstdio>

#include "livo_atlas/core/livo_system.h"

namespace livo_atlas::ros_detail {

inline void PrintInputDiagnostics(std::uint64_t lidar, std::uint64_t imu,
                                  std::uint64_t image,
                                  const StartupBufferStats& startup) {
  std::printf("livo_atlas inputs: lidar=%llu imu=%llu image=%llu\n",
              static_cast<unsigned long long>(lidar),
              static_cast<unsigned long long>(imu),
              static_cast<unsigned long long>(image));
  std::printf(
      "livo_atlas startup: imu_buffered=%llu image_buffered=%llu "
      "imu_evicted=%llu image_evicted=%llu imu_rejected=%llu\n",
      static_cast<unsigned long long>(startup.buffered_imu),
      static_cast<unsigned long long>(startup.buffered_images),
      static_cast<unsigned long long>(startup.evicted_imu),
      static_cast<unsigned long long>(startup.evicted_images),
      static_cast<unsigned long long>(startup.rejected_imu));
}

}  // namespace livo_atlas::ros_detail

#endif  // LIVO_ATLAS_SRC_ROS_INPUT_DIAGNOSTICS_H_
