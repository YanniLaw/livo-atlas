#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <stdexcept>
#include <utility>

#include "livo_atlas/frontend/image_options.h"
#include "livo_atlas/frontend/imu_propagation.h"
#include "livo_atlas/sensor/synchronizer.h"
#include "preprocess.h"
#include "ros2/mapping_session.h"

namespace livo_atlas::ros2 {
namespace {
FrontendMode Mode(const SessionOptions& options) {
  if (options.image_enabled && options.lidar_enabled) {
    return FrontendMode::kLidarInertialVisual;
  }
  return options.frontend.imu.enabled ? FrontendMode::kLidarInertial
                                      : FrontendMode::kLidarOnly;
}
SessionOptions Prepare(SessionOptions options) {
  const auto error = ValidateImageOptions(options.frontend.calibration.camera,
                                          options.frontend.visual);
  if (!error.empty()) throw std::invalid_argument(error);
  options.output.mode = options.processor.mode = Mode(options);
  options.output.calibration = options.processor.calibration =
      options.frontend.calibration;
  options.processor.colmap = options.output.colmap;
  ValidateOutputOptions(options.output);
  return options;
}
// Preserve the two ROS1 fromSec/toSec stages without importing ROS into this
// ordinary session. The input range is the ROS2 signed seconds range.
double NanosecondTime(double value) {
  if (!std::isfinite(value) ||
      value < std::numeric_limits<std::int32_t>::min() ||
      value >= double{std::numeric_limits<std::int32_t>::max()} + 1) {
    throw std::invalid_argument("corrected IMU timestamp is out of range");
  }
  double whole = std::floor(value);
  auto nanoseconds = std::llround((value - whole) * 1e9);
  if (nanoseconds == 1000000000) {
    ++whole;
    nanoseconds = 0;
  }
  return whole + static_cast<double>(nanoseconds) * 1e-9;
}
}  // namespace
class MappingSession::Impl {
 public:
  explicit Impl(SessionOptions options)
      : options_(Prepare(std::move(options))),
        synchronizer_(Mode(options_), options_.exposure_time),
        frontend_(options_.frontend),
        processor_(options_.processor),
        writer_(options_.output) {
    const auto& pre = options_.preprocess;
    preprocess_.set(pre.features, pre.lidar_type, pre.blind, pre.point_filter);
    preprocess_.N_SCANS = pre.scan_lines;
  }
  bool AddLidar(const LidarScan& scan) {
    if (!options_.lidar_enabled) return false;
    PointCloudMeasurement measurement{scan.timestamp,
                                      PointCloud::Ptr(new PointCloud)};
    preprocess_.Process(scan, measurement.cloud.get());
    if (measurement.cloud->empty()) return false;
    if (scan.timestamp < last_lidar_) buffers_.point_clouds.clear();
    buffers_.point_clouds.push_back(std::move(measurement));
    last_lidar_ = scan.timestamp;
    return true;
  }
  bool AddImu(ImuMeasurement measurement) {
    if (!options_.frontend.imu.enabled || last_lidar_ < 0) return false;
    if (!measurement.linear_acceleration.allFinite() ||
        !measurement.angular_velocity.allFinite()) {
      return false;
    }
    double timestamp =
        NanosecondTime(measurement.timestamp - options_.imu_time_offset);
    if (options_.driver_fix) timestamp += std::round(last_lidar_ - timestamp);
    if (last_imu_ > 0 && timestamp < last_imu_) return false;
    measurement.timestamp = NanosecondTime(timestamp);
    last_imu_ = timestamp;
    buffers_.imu.push_back(measurement);
    buffers_.latest_imu_time = timestamp;
    if (options_.propagate_imu) {
      if (!frontend_.imu_needs_initialization())
        propagation_buffer_.push_back(measurement);
      newest_imu_ = measurement;
      new_imu_ = true;
    }
    return true;
  }
  bool AddImage(ImageMeasurement measurement) {
    if (!options_.image_enabled) return false;
    if (options_.hilti && ++image_count_ % 4 != 0) return false;
    const double timestamp = measurement.timestamp;
    if (std::abs(timestamp - last_image_) < 0.001 || last_lidar_ < 0 ||
        timestamp < last_image_ || timestamp - last_image_ < 0.02)
      return false;
    buffers_.images.push_back(std::move(measurement));
    last_image_ = timestamp;
    return true;
  }
  void ProcessAvailable(const ResultCallback& callback) {
    while (true) {
      const auto status =
          synchronizer_.TrySynchronize(&buffers_, &measurements_);
      if (status == SynchronizationResult::kWaiting) return;
      if (status != SynchronizationResult::kReady) continue;
      const auto start = std::chrono::steady_clock::now();
      const auto result = frontend_.Process(&measurements_);
      const double elapsed = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - start)
                                 .count();
      if (result.updated && options_.propagate_imu) {
        latest_state_ = result.estimate;
        latest_time_ = result.timestamp;
        state_updated_ = have_estimate_ = true;
      }
      const auto prepared = processor_.Process(result);
      writer_.Write(result, prepared);
      if (result.updated) callback(result, prepared, elapsed);
    }
  }
  std::optional<PropagationResult> PropagateLatest() {
    if (frontend_.imu_needs_initialization() || !new_imu_ || !have_estimate_)
      return {};
    new_imu_ = false;
    if (!options_.propagate_imu || propagation_buffer_.empty()) return {};
    if (state_updated_) {
      propagated_ = latest_state_;
      while (!propagation_buffer_.empty() &&
             propagation_buffer_.front().timestamp < latest_time_) {
        propagation_buffer_.pop_front();
      }
      last_from_end_ = 0;
      for (const auto& imu : propagation_buffer_) Propagate(imu);
      state_updated_ = false;
    } else {
      Propagate(newest_imu_);
    }
    return PropagationResult{newest_imu_.timestamp, propagated_};
  }
  void Propagate(const ImuMeasurement& measurement) {
    const double from_end = measurement.timestamp - latest_time_;
    PropagateImu(&propagated_, from_end - last_from_end_,
                 measurement.linear_acceleration, measurement.angular_velocity,
                 frontend_.imu_mean_acceleration_norm());
    last_from_end_ = from_end;
  }
  SessionOptions options_;
  Preprocess preprocess_;
  SensorBuffers buffers_;
  SensorSynchronizer synchronizer_;
  SynchronizedMeasurements measurements_;
  Frontend frontend_;
  OutputProcessor processor_;
  ResultWriter writer_;
  double last_lidar_ = -1, last_imu_ = -1, last_image_ = -1;
  std::uint64_t image_count_ = 0;
  std::deque<ImuMeasurement> propagation_buffer_;
  ImuMeasurement newest_imu_;
  bool new_imu_ = false, state_updated_ = false, have_estimate_ = false;
  State latest_state_, propagated_;
  double latest_time_ = 0, last_from_end_ = 0;
};
MappingSession::MappingSession(SessionOptions options)
    : impl_(std::make_unique<Impl>(std::move(options))) {}
MappingSession::~MappingSession() = default;
bool MappingSession::AddLidar(const LidarScan& scan) {
  return impl_->AddLidar(scan);
}
bool MappingSession::AddImu(ImuMeasurement measurement) {
  return impl_->AddImu(std::move(measurement));
}
bool MappingSession::AddImage(ImageMeasurement measurement) {
  return impl_->AddImage(std::move(measurement));
}
void MappingSession::ProcessAvailable(const ResultCallback& callback) {
  impl_->ProcessAvailable(callback);
}
std::optional<PropagationResult> MappingSession::PropagateLatest() {
  return impl_->PropagateLatest();
}
void MappingSession::Finish() { impl_->writer_.Finish(); }
}  // namespace livo_atlas::ros2
