#ifndef LIVO_ATLAS_ROS_COMMON_SENSOR_INPUT_H_
#define LIVO_ATLAS_ROS_COMMON_SENSOR_INPUT_H_

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <exception>
#include <mutex>
#include <variant>

#include "livo_atlas/core/livo_system.h"

namespace livo_atlas::ros_detail {

enum class Sensor { kLidar, kImu, kImage };

// Subscription callbacks produce owned measurements. One system owner
// consumes them in enqueue order, including the LiDAR anchors for driver_fix.
class SensorInput {
 public:
  using Measurement = std::variant<LidarScan, Imu, TimedImage>;

  SensorInput(const SensorBufferOptions& limits, bool stage_timing);

  void RecordReceived(Sensor sensor);
  void RecordConversionRejection(Sensor sensor);
  void RecordTransportLoss(Sensor sensor, std::uint64_t count);
  bool stage_timing() const { return stage_timing_; }
  bool Push(Measurement measurement);
  void Fail(std::exception_ptr error);
  void RethrowFailure() const;

  // Owner only. Admit a bounded batch and process at most one update.
  SyncStatus ProcessNext(LivoSystem& system,
                         const LivoSystem::ResultCallback& callback);
  // Stop the subscription callbacks before draining.
  void Drain(LivoSystem& system);
  void Report(const LivoSystem& system) const;

 private:
  using Clock = std::chrono::steady_clock;

  struct Entry {
    Measurement measurement;
    Clock::time_point enqueued;
  };

  struct TransportCounters {
    std::atomic<std::uint64_t> received{0};
    std::atomic<std::uint64_t> conversion_rejected{0};
    std::atomic<std::uint64_t> lost{0};
  };

  struct QueueCounters {
    std::size_t pending = 0;
    std::size_t peak = 0;
    std::size_t combined_peak = 0;
    std::uint64_t overflow = 0;
    std::uint64_t closed = 0;
  };

  const Entry* Front() const;
  bool TransferFront(LivoSystem& system);
  void SampleCombinedPeaks(const LivoSystem& system);

  const bool stage_timing_;
  const std::array<std::size_t, 3> limits_;
  std::array<TransportCounters, 3> transport_;
  mutable std::mutex mutex_;
  std::array<QueueCounters, 3> counters_;
  std::deque<Entry> queue_;
  bool closed_ = false;
  std::exception_ptr failure_;
  std::array<std::uint64_t, 3> system_rejected_{};
};

}  // namespace livo_atlas::ros_detail

#endif  // LIVO_ATLAS_ROS_COMMON_SENSOR_INPUT_H_
