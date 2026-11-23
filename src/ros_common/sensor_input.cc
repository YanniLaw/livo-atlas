#include "livo_atlas/ros_common/sensor_input.h"

#include <algorithm>
#include <iomanip>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "common/timing.h"
#include "livo_atlas/core/livo_system.h"
#include "ros_common/input_diagnostics.h"

namespace livo_atlas::ros_detail {
namespace {

constexpr std::size_t kBatchSize = 32;
constexpr const char* kSensors[] = {"lidar", "imu", "image"};
constexpr const char* kEnqueueStages[] = {
    "lidar_enqueue_wait", "imu_enqueue_wait", "image_enqueue_wait"};
constexpr const char* kResidenceStages[] = {
    "lidar_queue_residence", "imu_queue_residence", "image_queue_residence"};

std::size_t Index(Sensor sensor) { return static_cast<std::size_t>(sensor); }

std::array<std::size_t, 3> Buffered(const SensorBufferCounts& counts) {
  return {counts.lidar, counts.imu + counts.propagation_imu, counts.image};
}

bool AddToSystem(SensorInput::Measurement& measurement, LivoSystem& system) {
  if (const auto* scan = std::get_if<LidarScan>(&measurement)) {
    return system.AddLidar(*scan);
  }
  if (auto* imu = std::get_if<Imu>(&measurement)) {
    try {
      return system.AddImu(std::move(*imu));
    } catch (const std::invalid_argument& error) {
      LOG(WARNING) << error.what();
      return false;
    }
  }
  return system.AddImage(std::move(std::get<TimedImage>(measurement)));
}

}  // namespace

SensorInput::SensorInput(const SensorBufferOptions& limits, bool stage_timing)
    : stage_timing_(stage_timing),
      limits_{static_cast<std::size_t>(limits.max_lidar_scans),
              static_cast<std::size_t>(limits.max_imu_samples),
              static_cast<std::size_t>(limits.max_images)} {}

void SensorInput::RecordReceived(Sensor sensor) {
  transport_[Index(sensor)].received.fetch_add(1, std::memory_order_relaxed);
}

void SensorInput::RecordConversionRejection(Sensor sensor) {
  transport_[Index(sensor)].conversion_rejected.fetch_add(
      1, std::memory_order_relaxed);
}

void SensorInput::RecordTransportLoss(Sensor sensor, std::uint64_t count) {
  transport_[Index(sensor)].lost.fetch_add(count, std::memory_order_relaxed);
}

bool SensorInput::Push(Measurement measurement) {
  const auto id = measurement.index();
  const auto start = StartTiming(stage_timing_);
  Clock::time_point enqueued;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    auto& counters = counters_[id];
    if (closed_) {
      ++counters.closed;
      return false;
    }
    if (counters.pending == limits_[id]) {
      std::ostringstream message;
      message << "ROS sensor input overflow: " << kSensors[id]
              << " count=" << counters.pending << " limit=" << limits_[id]
              << " timestamp=" << std::setprecision(17)
              << std::visit([](const auto& m) { return m.timestamp; },
                            measurement);
      failure_ = std::make_exception_ptr(std::overflow_error(message.str()));
      closed_ = true;
      ++counters.overflow;
      return false;
    }
    enqueued = StartTiming(stage_timing_);
    queue_.push_back({std::move(measurement), enqueued});
    ++counters.pending;
    counters.peak = std::max(counters.peak, counters.pending);
  }
  if (stage_timing_) LogStageTiming(kEnqueueStages[id], enqueued - start);
  return true;
}

void SensorInput::Fail(std::exception_ptr error) {
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!failure_) failure_ = std::move(error);
  closed_ = true;
}

void SensorInput::RethrowFailure() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  if (failure_) std::rethrow_exception(failure_);
}

const SensorInput::Entry* SensorInput::Front() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  if (failure_) std::rethrow_exception(failure_);
  // Only this owner pops. Producer appends preserve references to entries.
  return queue_.empty() ? nullptr : &queue_.front();
}

bool SensorInput::TransferFront(LivoSystem& system) {
  std::optional<Entry> entry;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (failure_) std::rethrow_exception(failure_);
    if (queue_.empty()) return false;
    entry.emplace(std::move(queue_.front()));
    --counters_[entry->measurement.index()].pending;
    queue_.pop_front();
  }
  const auto id = entry->measurement.index();
  LogStageTiming(kResidenceStages[id], entry->enqueued);
  if (!AddToSystem(entry->measurement, system)) ++system_rejected_[id];
  return true;
}

void SensorInput::SampleCombinedPeaks(const LivoSystem& system) {
  const auto buffered = Buffered(system.GetSensorBufferCounts());
  const std::lock_guard<std::mutex> lock(mutex_);
  for (std::size_t i = 0; i < counters_.size(); ++i) {
    auto& counters = counters_[i];
    counters.combined_peak =
        std::max(counters.combined_peak, buffered[i] + counters.pending);
  }
}

SyncStatus SensorInput::ProcessNext(
    LivoSystem& system, const LivoSystem::ResultCallback& callback) {
  bool at_limit = false;
  std::size_t transferred = 0;
  for (; transferred < kBatchSize; ++transferred) {
    const auto* entry = Front();
    if (!entry) break;
    at_limit = std::visit(
        [&system](const auto& sample) {
          return system.WouldExceedInputLimit(sample);
        },
        entry->measurement);
    if (at_limit) break;
    TransferFront(system);
  }
  if (transferred != 0) SampleCombinedPeaks(system);
  const auto status = system.ProcessNext(callback);
  if (at_limit && status == SyncStatus::kWaiting) {
    // When synchronization cannot free capacity, apply the system's ordinary
    // admission rules to the same head. FIFO prevents bypassing it.
    TransferFront(system);
    SampleCombinedPeaks(system);
  }
  return status;
}

void SensorInput::Drain(LivoSystem& system) {
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    closed_ = true;
  }
  try {
    RethrowFailure();
    const auto report = [](const auto& result, const auto&, double elapsed) {
      LogFrontendUpdate(result.kind, elapsed, /*published=*/false);
    };
    // The fixed tail can synchronize as it is admitted, even with tight
    // sensor-count and timestamp-span limits.
    while (system.ProcessNext(report) != SyncStatus::kWaiting ||
           TransferFront(system)) {
      SampleCombinedPeaks(system);
    }
  } catch (...) {
    system.RecordFailure(std::current_exception());
    throw;
  }
}

void SensorInput::Report(const LivoSystem& system) const {
  const auto buffered = Buffered(system.GetSensorBufferCounts());
  std::array<QueueCounters, 3> counters;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    counters = counters_;
  }
  LogInputDiagnostics(transport_[0].received.load(std::memory_order_relaxed),
                      transport_[1].received.load(std::memory_order_relaxed),
                      transport_[2].received.load(std::memory_order_relaxed),
                      system.GetStartupBufferStats());
  for (std::size_t i = 0; i < counters.size(); ++i) {
    const auto& queue = counters[i];
    LOG(INFO) << "livo_atlas sensor_input sensor=" << kSensors[i]
              << " peak=" << queue.peak
              << " combined_sampled_peak=" << queue.combined_peak
              << " pending=" << queue.pending
              << " system_pending=" << buffered[i] << " transport_lost="
              << transport_[i].lost.load(std::memory_order_relaxed)
              << " conversion_rejected="
              << transport_[i].conversion_rejected.load(
                     std::memory_order_relaxed)
              << " system_rejected=" << system_rejected_[i]
              << " overflow_rejected=" << queue.overflow
              << " closed_rejected=" << queue.closed;
  }
}

}  // namespace livo_atlas::ros_detail
