#include "livo_atlas/core/livo_system.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <stdexcept>
#include <utility>

#include "livo_atlas/frontend/imu/imu_propagation.h"
#include "livo_atlas/frontend/vision/image_options.h"
#include "livo_atlas/io/keyframe_store.h"
#include "livo_atlas/io/submap_store.h"
#include "livo_atlas/sensor/lidar_preprocessor.h"
#include "livo_atlas/sensor/synchronizer.h"

namespace livo_atlas {
namespace {
// Preserve the two ROS1 fromSec/toSec stages without importing ROS into this
// ordinary system. The source supplies its whole-second transport range.
double NanosecondTime(double value, const TimestampRange& range) {
  if (!std::isfinite(value) || value < range.minimum ||
      value >= range.maximum) {
    throw std::invalid_argument("corrected IMU timestamp is out of range");
  }
  double whole = std::floor(value);
  auto nanoseconds = std::llround((value - whole) * 1e9);
  if (nanoseconds == 1000000000) {
    ++whole;
    nanoseconds = 0;
  }
  const double result = whole + static_cast<double>(nanoseconds) * 1e-9;
  if (result < range.minimum || result >= range.maximum) {
    throw std::invalid_argument("corrected IMU timestamp is out of range");
  }
  return result;
}
}  // namespace
class LivoSystem::Impl {
 public:
  explicit Impl(SystemOptions options)
      : options_(PrepareSystemOptions(std::move(options))),
        synchronizer_(options_.exporter.mode, options_.exposure_time),
        frontend_(options_.frontend),
        colorizer_(options_.colorizer),
        keyframe_selector_(options_.exporter.mode, options_.keyframes),
        keyframe_store_(options_.keyframes.enabled
                            ? std::make_unique<KeyframeStore>(
                                  options_.exporter.directory / "keyframes",
                                  options_.frontend.calibration)
                            : nullptr),
        submap_builder_(options_.submaps.max_keyframes),
        submap_store_(
            options_.submaps.enabled
                ? std::make_unique<SubmapStore>(
                      options_.exporter.directory / "keyframes/submaps",
                      *keyframe_store_, options_.submaps.max_keyframes)
                : nullptr),
        writer_(options_.exporter) {
    const auto& pre = options_.preprocess;
    preprocess_.Configure(pre.features, pre.lidar_type, pre.blind,
                          pre.point_filter);
    preprocess_.N_SCANS = pre.scan_lines;
  }
  bool AddLidar(const LidarScan& scan) {
    if (!options_.lidar_enabled || !std::isfinite(scan.timestamp)) return false;
    TimedPointCloud measurement{scan.timestamp,
                                PointCloud::Ptr(new PointCloud)};
    preprocess_.Process(scan, measurement.cloud.get());
    if (measurement.cloud->empty()) return false;
    if (scan.timestamp < last_lidar_) buffers_.point_clouds.clear();
    buffers_.point_clouds.push_back(std::move(measurement));
    last_lidar_ = scan.timestamp;
    return true;
  }
  bool AddImu(Imu measurement) {
    if (!options_.frontend.imu.enabled || last_lidar_ < 0) return false;
    if (!measurement.linear_acceleration.allFinite() ||
        !measurement.angular_velocity.allFinite()) {
      return false;
    }
    double timestamp =
        NanosecondTime(measurement.timestamp - options_.imu_time_offset,
                       options_.imu_time_range);
    if (options_.driver_fix) timestamp += std::round(last_lidar_ - timestamp);
    if (last_imu_ > 0 && timestamp < last_imu_) return false;
    measurement.timestamp = NanosecondTime(timestamp, options_.imu_time_range);
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
  bool AddImage(TimedImage measurement) {
    if (!options_.image_enabled || !std::isfinite(measurement.timestamp))
      return false;
    if (options_.hilti && ++image_count_ % 4 != 0) return false;
    const double timestamp = measurement.timestamp;
    if (std::abs(timestamp - last_image_) < 0.001 || last_lidar_ < 0 ||
        timestamp < last_image_ || timestamp - last_image_ < 0.02)
      return false;
    buffers_.images.push_back(std::move(measurement));
    last_image_ = timestamp;
    return true;
  }
  SyncStatus ProcessNext(const ResultCallback& callback) {
    const auto status = synchronizer_.TrySynchronize(&buffers_, &measurements_);
    if (status != SyncStatus::kReady) return status;
    const auto start = std::chrono::steady_clock::now();
    const auto result = frontend_.Process(&measurements_);
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
            .count();
    if (result.updated && options_.propagate_imu) {
      latest_state_ = result.estimate;
      latest_time_ = result.timestamp;
      state_updated_ = have_estimate_ = true;
    }
    const auto colored_cloud = colorizer_.Process(result);
    writer_.Write(result, colored_cloud);
    if (keyframe_store_ && keyframe_selector_.Select(result)) {
      keyframe_store_->Append(CaptureKeyframe(next_keyframe_id_, result));
      if (submap_store_) {
        if (const auto submap = submap_builder_.AddKeyframe(next_keyframe_id_))
          submap_store_->Append(*submap);
      }
      ++next_keyframe_id_;
    }
    if (result.updated && callback) callback(result, colored_cloud, elapsed);
    return status;
  }
  void ProcessAvailable(const ResultCallback& callback) {
    while (ProcessNext(callback) != SyncStatus::kWaiting) {
    }
  }
  void Finish() {
    writer_.Finish();
    if (submap_store_) {
      if (const auto tail = submap_builder_.Finish())
        submap_store_->Append(*tail);
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
  void Propagate(const Imu& measurement) {
    const double from_end = measurement.timestamp - latest_time_;
    PropagateImu(&propagated_, from_end - last_from_end_,
                 measurement.linear_acceleration, measurement.angular_velocity,
                 frontend_.imu_mean_acceleration_norm());
    last_from_end_ = from_end;
  }
  SystemOptions options_;
  LidarPreprocessor preprocess_;
  SensorBuffers buffers_;
  SensorSynchronizer synchronizer_;
  SynchronizedMeasurements measurements_;
  Frontend frontend_;
  CloudColorizer colorizer_;
  KeyframeSelector keyframe_selector_;
  std::unique_ptr<KeyframeStore> keyframe_store_;
  SubmapBuilder submap_builder_;
  std::unique_ptr<SubmapStore> submap_store_;
  LivoExporter writer_;
  std::uint64_t next_keyframe_id_ = 0;
  double last_lidar_ = -1, last_imu_ = -1, last_image_ = -1;
  std::uint64_t image_count_ = 0;
  std::deque<Imu> propagation_buffer_;
  Imu newest_imu_;
  bool new_imu_ = false, state_updated_ = false, have_estimate_ = false;
  FilterState latest_state_, propagated_;
  double latest_time_ = 0, last_from_end_ = 0;
};
LivoSystem::LivoSystem(SystemOptions options)
    : impl_(std::make_unique<Impl>(std::move(options))) {}
LivoSystem::~LivoSystem() = default;
bool LivoSystem::AddLidar(const LidarScan& scan) {
  return impl_->AddLidar(scan);
}
bool LivoSystem::AddImu(Imu measurement) {
  return impl_->AddImu(std::move(measurement));
}
bool LivoSystem::AddImage(TimedImage measurement) {
  return impl_->AddImage(std::move(measurement));
}
SyncStatus LivoSystem::ProcessNext(const ResultCallback& callback) {
  return impl_->ProcessNext(callback);
}
void LivoSystem::ProcessAvailable(const ResultCallback& callback) {
  impl_->ProcessAvailable(callback);
}
std::optional<PropagationResult> LivoSystem::PropagateLatest() {
  return impl_->PropagateLatest();
}
void LivoSystem::Finish() { impl_->Finish(); }
}  // namespace livo_atlas
