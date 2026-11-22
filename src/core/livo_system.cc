#include "livo_atlas/core/livo_system.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <exception>
#include <limits>
#include <stdexcept>
#include <utility>

#include "livo_atlas/frontend/imu/imu_propagation.h"
#include "livo_atlas/frontend/vision/image_options.h"
#include "livo_atlas/io/keyframe_store.h"
#include "livo_atlas/io/submap_store.h"
#include "livo_atlas/sensor/lidar_preprocessor.h"
#include "livo_atlas/sensor/synchronizer.h"
#include "online_submap_manager.h"

namespace livo_atlas {
namespace {
constexpr std::size_t kStartupImuLimit = 2000;
constexpr std::size_t kStartupImageLimit = 8;

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

LivoSystem::LivoSystem(SystemOptions options)
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
      submap_store_(options_.submaps.enabled &&
                            options_.submap_mode == SubmapMode::kKeyframeGroups
                        ? std::make_unique<SubmapStore>(
                              options_.exporter.directory / "keyframes/submaps",
                              *keyframe_store_, options_.submaps.max_keyframes)
                        : nullptr),
      online_(
          options_.submaps.enabled &&
                  options_.submap_mode == SubmapMode::kOnline
              ? std::make_unique<OnlineSubmapManager>(
                    options_.exporter.directory / "keyframes/online_submaps",
                    *keyframe_store_, options_.submaps.max_keyframes,
                    options_.online_submaps, options_.geometry)
              : nullptr),
      writer_(options_.exporter) {
  const auto& pre = options_.preprocess;
  preprocess_.Configure(pre.features, pre.lidar_type, pre.blind,
                        pre.point_filter);
  preprocess_.N_SCANS = pre.scan_lines;
  if (options_.loop_closure) {
    loop_backend_.emplace(*this, *options_.loop_closure);
    Online().SetLoopBackend(*loop_backend_);
  }
}

void LivoSystem::CheckInputOpen() const {
  if (submap_error_) std::rethrow_exception(submap_error_);
  if (online_) online_->CheckRunning();
  if (input_sealed_) throw std::logic_error("System input has been sealed");
}

bool LivoSystem::AddLidar(const LidarScan& scan) {
  CheckInputOpen();
  if (!options_.lidar_enabled || !std::isfinite(scan.timestamp)) return false;
  TimedPointCloud measurement{scan.timestamp, PointCloud::Ptr(new PointCloud)};
  preprocess_.Process(scan, measurement.cloud.get());
  if (measurement.cloud->empty()) return false;
  const bool first_lidar = last_lidar_ < 0;
  if (scan.timestamp < last_lidar_) buffers_.point_clouds.clear();
  buffers_.point_clouds.push_back(std::move(measurement));
  last_lidar_ = scan.timestamp;
  if (first_lidar && last_lidar_ >= 0) FlushStartupImu();
  return true;
}

bool LivoSystem::AddImu(Imu measurement) {
  CheckInputOpen();
  if (!options_.frontend.imu.enabled ||
      (last_lidar_ < 0 && !options_.buffer_before_first_lidar))
    return false;
  if (!measurement.linear_acceleration.allFinite() ||
      !measurement.angular_velocity.allFinite()) {
    return false;
  }
  measurement.timestamp =
      NanosecondTime(measurement.timestamp - options_.imu_time_offset,
                     options_.imu_time_range);
  if (last_lidar_ < 0) {
    startup_imu_.push_back(std::move(measurement));
    ++startup_stats_.buffered_imu;
    if (startup_imu_.size() > kStartupImuLimit) {
      startup_imu_.pop_front();
      ++startup_stats_.evicted_imu;
    }
    return true;
  }
  return EnqueueImu(std::move(measurement));
}

bool LivoSystem::EnqueueImu(Imu measurement) {
  double timestamp = measurement.timestamp;
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

void LivoSystem::FlushStartupImu() {
  // Driver correction needs the first valid LiDAR timestamp as its anchor.
  while (!startup_imu_.empty()) {
    auto measurement = std::move(startup_imu_.front());
    startup_imu_.pop_front();
    try {
      if (!EnqueueImu(std::move(measurement))) ++startup_stats_.rejected_imu;
    } catch (const std::invalid_argument&) {
      ++startup_stats_.rejected_imu;
    }
  }
}

bool LivoSystem::AddImage(TimedImage measurement) {
  CheckInputOpen();
  if (!options_.image_enabled || !std::isfinite(measurement.timestamp))
    return false;
  if (options_.hilti && ++image_count_ % 4 != 0) return false;
  const double timestamp = measurement.timestamp;
  if (std::abs(timestamp - last_image_) < 0.001 ||
      (last_lidar_ < 0 && !options_.buffer_before_first_lidar) ||
      timestamp < last_image_ || timestamp - last_image_ < 0.02) {
    return false;
  }
  buffers_.images.push_back(std::move(measurement));
  if (last_lidar_ < 0) {
    ++startup_stats_.buffered_images;
    if (buffers_.images.size() > kStartupImageLimit) {
      buffers_.images.pop_front();
      ++startup_stats_.evicted_images;
    }
  }
  last_image_ = timestamp;
  return true;
}

StartupBufferStats LivoSystem::GetStartupBufferStats() const noexcept {
  auto stats = startup_stats_;
  stats.pending_imu = startup_imu_.size();
  stats.pending_images = last_lidar_ < 0 ? buffers_.images.size() : 0;
  return stats;
}

SyncStatus LivoSystem::ProcessNext(const ResultCallback& callback) {
  const auto status = ProcessMeasurement(callback);
  if (loop_backend_) loop_backend_->Poll();
  return status;
}

SyncStatus LivoSystem::ProcessMeasurement(const ResultCallback& callback) {
  CheckInputOpen();
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
    if (online_) {
      try {
        auto keyframe = std::make_shared<const Keyframe>(
            CaptureKeyframe(next_keyframe_id_, result));
        keyframe_store_->Append(*keyframe);
        online_->AppendCommittedKeyframe(std::move(keyframe));
      } catch (...) {
        submap_error_ = std::current_exception();
        throw;
      }
    } else {
      keyframe_store_->Append(CaptureKeyframe(next_keyframe_id_, result));
    }
    if (submap_store_) {
      if (const auto submap = submap_builder_.AddKeyframe(next_keyframe_id_))
        AppendSubmap(*submap);
    }
    ++next_keyframe_id_;
  }
  if (result.updated && callback) callback(result, colored_cloud, elapsed);
  return status;
}

void LivoSystem::ProcessAvailable(const ResultCallback& callback) {
  while (ProcessNext(callback) != SyncStatus::kWaiting) {
  }
}

void LivoSystem::SealInput() {
  if (submap_error_) std::rethrow_exception(submap_error_);
  if (online_) online_->RethrowFailure();
  if (input_sealed_) return;
  try {
    if (online_) online_->SealInput();
    if (submap_store_) {
      if (const auto tail = submap_builder_.Finish()) AppendSubmap(*tail);
    }
    if (last_lidar_ < 0) {
      startup_imu_.clear();
      buffers_.images.clear();
    }
    input_sealed_ = true;
  } catch (...) {
    submap_error_ = std::current_exception();
    throw;
  }
}

void LivoSystem::Finish() {
  if (IsFinished()) return;
  FinishSession(loop_backend_ ? &*loop_backend_ : nullptr);
}

void LivoSystem::FinishSession(LoopBackend* backend) {
  try {
    if (submap_error_) std::rethrow_exception(submap_error_);
    if (online_) online_->RethrowFailure();
    if (backend) backend->Drain();
    writer_.Finish();
    SealInput();
    if (online_) online_->Finish();
    if (backend) backend->FinishJournal();
    finished_ = true;
  } catch (...) {
    const auto error = std::current_exception();
    if (backend) backend->Fail(error);
    submap_error_ = error;
    throw;
  }
}

bool LivoSystem::IsFinished() const noexcept {
  return finished_ && !submap_error_;
}

void LivoSystem::AppendSubmap(const Submap& submap) {
  try {
    submap_store_->Append(submap);
  } catch (...) {
    submap_error_ = std::current_exception();
    throw;
  }
}

void LivoSystem::SetOnlineSubmapCallback(OnlineSubmapCallback callback) {
  if (callback && submap_error_) std::rethrow_exception(submap_error_);
  Online().SetCallback(std::move(callback));
}

void LivoSystem::ApplyPoseUpdate(const PoseUpdate& update) {
  if (submap_error_) std::rethrow_exception(submap_error_);
  if (loop_pose_updates_claimed_)
    throw std::logic_error("Online loop backend owns pose updates");
  Online().ApplyPoseUpdate(update);
}

void LivoSystem::ClaimLoopPoseUpdates() {
  CheckInputOpen();
  if (loop_pose_updates_claimed_ || next_keyframe_id_ ||
      Online().GetPoseSnapshot()->version)
    throw std::logic_error("Loop backend requires a new unclaimed session");
  loop_pose_updates_claimed_ = true;
}

void LivoSystem::ApplyLoopPoseUpdate(const PoseUpdate& update) {
  if (submap_error_) std::rethrow_exception(submap_error_);
  if (!loop_pose_updates_claimed_)
    throw std::logic_error("Loop backend has not claimed pose updates");
  Online().ApplyPoseUpdate(update);
}

void LivoSystem::FailLoopBackend(std::exception_ptr error) noexcept {
  if (!submap_error_) submap_error_ = std::move(error);
}

GeometrySubmap LivoSystem::GetGeometrySubmap(std::uint64_t id) {
  if (submap_error_) std::rethrow_exception(submap_error_);
  return Online().GetGeometrySubmap(id);
}

GeometryCacheStats LivoSystem::GetGeometryStats() const {
  return Online().GetGeometryStats();
}

OnlineSubmapManager& LivoSystem::Online() const {
  if (!online_) throw std::logic_error("Online submaps are not enabled");
  return *online_;
}

std::optional<PropagationResult> LivoSystem::PropagateLatest() {
  CheckInputOpen();
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

void LivoSystem::Propagate(const Imu& measurement) {
  const double from_end = measurement.timestamp - latest_time_;
  PropagateImu(&propagated_, from_end - last_from_end_,
               measurement.linear_acceleration, measurement.angular_velocity,
               frontend_.imu_mean_acceleration_norm());
  last_from_end_ = from_end;
}

LivoSystem::~LivoSystem() = default;

std::shared_ptr<const PoseSnapshot> LivoSystem::GetPoseSnapshot() const {
  return Online().GetPoseSnapshot();
}

MapCorrection LivoSystem::GetMapCorrection() const {
  return Online().GetMapCorrection();
}

KeyframeReadView LivoSystem::GetKeyframeReadView() const {
  if (!keyframe_store_)
    throw std::logic_error("Keyframe storage is not enabled");
  return keyframe_store_->CaptureReadView();
}

std::optional<LoopSessionStats> LivoSystem::GetLoopStats() const {
  if (!loop_backend_) return std::nullopt;
  return loop_backend_->GetStats();
}

}  // namespace livo_atlas
