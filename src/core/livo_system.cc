#include "livo_atlas/core/livo_system.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <exception>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "common/timing.h"
#include "livo_atlas/common/logging.h"
#include "livo_atlas/frontend/imu/imu_propagation.h"
#include "livo_atlas/frontend/vision/image_options.h"
#include "livo_atlas/mapping/geometry_mapper.h"
#include "livo_atlas/sensor/lidar_preprocessor.h"
#include "livo_atlas/sensor/synchronizer.h"
#include "pose_graph/pose_graph.h"

namespace livo_atlas {
namespace {

constexpr int kStartupImuLimit = 2000;
constexpr int kStartupImageLimit = 8;

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

// Keep the newest contiguous suffix. Startup IMU times can still be reversed
// around zero, so consider both endpoints of the timestamp range.
template <typename Sample>
std::size_t TrimStartupSpan(std::deque<Sample>* buffer, double max_span_s) {
  if (buffer->empty()) return 0;
  auto first = buffer->end();
  double earliest = buffer->back().timestamp;
  double latest = earliest;
  while (first != buffer->begin()) {
    const double timestamp = (first - 1)->timestamp;
    earliest = std::min(earliest, timestamp);
    latest = std::max(latest, timestamp);
    if (latest - earliest > max_span_s) break;
    --first;
  }
  const auto dropped = first - buffer->begin();
  buffer->erase(buffer->begin(), first);
  return dropped;
}

void DiscardCoveredImu(double estimate_time, std::deque<Imu>* buffer) {
  while (!buffer->empty() && buffer->front().timestamp < estimate_time) {
    buffer->pop_front();
  }
}

template <typename Sample>
double BufferSpan(const std::deque<Sample>& buffer, double timestamp,
                  bool ordered = true) {
  double earliest = buffer.front().timestamp;
  double latest = buffer.back().timestamp;
  // Startup IMU samples can be out of order before clock correction. The
  // active queue still accepts the legacy ordering across nonpositive times.
  if (!ordered || earliest <= 0) {
    const auto limits = std::minmax_element(
        buffer.begin(), buffer.end(), [](const auto& lhs, const auto& rhs) {
          return lhs.timestamp < rhs.timestamp;
        });
    earliest = limits.first->timestamp;
    latest = limits.second->timestamp;
  }
  return std::max(latest, timestamp) - std::min(earliest, timestamp);
}

template <typename Sample>
bool ExceedsBufferLimit(const std::deque<Sample>& buffer, int max_count,
                        double timestamp, double max_span) {
  return !buffer.empty() &&
         (buffer.size() >= static_cast<std::size_t>(max_count) ||
          !(BufferSpan(buffer, timestamp) <= max_span));
}

}  // namespace

LivoSystem::LivoSystem(SystemOptions options)
    : options_(PrepareSystemOptions(std::move(options))),
      synchronizer_(options_.frontend_mode(), options_.exposure_time),
      frontend_(options_.frontend),
      colorizer_(options_.colorizer),
      keyframe_selector_(options_.frontend_mode(), options_.keyframes),
      pose_graph_(options_.keyframes.enabled
                      ? std::make_unique<pose_graph::PoseGraph>(options_)
                      : nullptr),
      writer_(options_.exporter) {
  const auto& pre = options_.preprocess;
  preprocess_.Configure(pre.features, pre.lidar_type, pre.blind,
                        pre.point_filter);
  preprocess_.N_SCANS = pre.scan_lines;
  if (pose_graph_) committed_keyframes_ = pose_graph_->InitialReadView();
  if (options_.submaps.enabled && options_.submap_mode == SubmapMode::kOnline)
    frozen_poses_ = std::make_shared<const PoseSnapshot>();
  if (options_.geometry.enabled) {
    geometry_ = std::make_unique<GeometryMapper>(
        options_.geometry, [this](std::uint64_t id) {
          return committed_keyframes_->ReadBodyCloud(
              id, options_.geometry.max_points_per_keyframe);
        });
  }
  if (options_.loop_closure) {
    EnableLoops(*options_.loop_closure, {}, nullptr);
  }
}

void LivoSystem::CheckInputOpen() {
  if (session_error_) std::rethrow_exception(session_error_);
  if (pose_graph_) {
    try {
      pose_graph_->RethrowFailure();
    } catch (...) {
      PumpArchive();
      throw;
    }
  }
  if (input_sealed_) throw std::logic_error("System input has been sealed");
}

template <typename Sample>
void LivoSystem::CheckBufferLimit(const char* name,
                                  const std::deque<Sample>& buffer,
                                  int max_count, double timestamp) {
  const double max_span = options_.sensor_buffer.max_span_s;
  if (!ExceedsBufferLimit(buffer, max_count, timestamp, max_span)) return;
  const double oldest = buffer.front().timestamp;
  const double span = BufferSpan(buffer, timestamp);
  std::ostringstream message;
  message << std::setprecision(std::numeric_limits<double>::max_digits10)
          << "sensor_buffer overflow in " << name << " queue"
          << " queued=" << buffer.size() << " max_count=" << max_count
          << " oldest_timestamp=" << oldest << " new_timestamp=" << timestamp
          << " span_s=" << span << " max_span_s=" << max_span;
  RecordFailure(std::make_exception_ptr(std::overflow_error(message.str())));
  std::rethrow_exception(session_error_);
}

template <typename Sample>
void LivoSystem::RecordBufferPeak(const std::deque<Sample>& buffer,
                                 BufferPeak* peak, bool ordered) {
  if (!options_.stage_timing || buffer.empty()) return;
  peak->count = std::max(peak->count, buffer.size());
  peak->span_s = std::max(
      peak->span_s, BufferSpan(buffer, buffer.back().timestamp, ordered));
}

void LivoSystem::ReportBufferPeaks() const {
  if (!options_.stage_timing) return;
  const auto log = [](const char* queue, const BufferPeak& peak) {
    LOG(INFO) << "livo_atlas sensor_buffer_peak queue=" << queue
              << " count=" << peak.count << " span_s=" << std::fixed
              << std::setprecision(9) << peak.span_s;
  };
  log("lidar", lidar_peak_);
  log("imu", imu_peak_);
  log("image", image_peak_);
  log("imu_propagation", propagation_peak_);
  log("startup_imu", startup_imu_peak_);
  log("startup_image", startup_image_peak_);
}

bool LivoSystem::AddLidar(const LidarScan& scan) {
  CheckInputOpen();
  if (!options_.lidar_enabled || !std::isfinite(scan.timestamp)) return false;
  TimedPointCloud measurement{scan.timestamp, PointCloud::Ptr(new PointCloud)};
  const auto start = StartTiming(options_.stage_timing);
  preprocess_.Process(scan, measurement.cloud.get());
  LogStageTiming("lidar_preprocess", start);
  if (measurement.cloud->empty() ||
      (measurement.cloud->size() == 1 &&
       options_.frontend_mode() == FrontendMode::kLidarInertial)) {
    return false;
  }
  if (last_lidar_ < 0 && scan.timestamp >= 0) {
    AcceptLidarAnchor(std::move(measurement));
    have_first_lidar_ = true;
  } else {
    if (scan.timestamp < last_lidar_) {
      buffers_.point_clouds.clear();
    } else {
      CheckBufferLimit("LiDAR", buffers_.point_clouds,
                       options_.sensor_buffer.max_lidar_scans, scan.timestamp);
    }
    buffers_.point_clouds.push_back(std::move(measurement));
  }
  last_lidar_ = scan.timestamp;
  RecordBufferPeak(buffers_.point_clouds, &lidar_peak_);
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
  const auto& limits = options_.sensor_buffer;
  if (last_lidar_ < 0) {
    startup_imu_.push_back(std::move(measurement));
    ++startup_stats_.buffered_imu;
    const auto capacity = std::min(kStartupImuLimit, limits.max_imu_samples);
    if (startup_imu_.size() > static_cast<std::size_t>(capacity)) {
      startup_imu_.pop_front();
      ++startup_stats_.evicted_imu;
    }
    RecordBufferPeak(startup_imu_, &startup_imu_peak_, /*ordered=*/false);
    return true;
  }
  const auto corrected =
      CorrectImuTimestamp(last_lidar_, last_imu_, &measurement);
  if (!corrected) return false;
  const bool propagate = options_.propagate_imu &&
                         !frontend_.imu_needs_initialization();
  BufferImu(measurement, propagate, &buffers_.imu, &propagation_buffer_);
  RecordBufferPeak(buffers_.imu, &imu_peak_);
  if (propagate) RecordBufferPeak(propagation_buffer_, &propagation_peak_);
  last_imu_ = buffers_.latest_imu_time = *corrected;
  if (propagate) {
    newest_imu_ = measurement;
    new_imu_ = true;
  }
  return true;
}

void LivoSystem::BufferImu(const Imu& measurement, bool propagate,
                           std::deque<Imu>* imu, std::deque<Imu>* propagation) {
  const int capacity = options_.sensor_buffer.max_imu_samples;
  CheckBufferLimit("IMU", *imu, capacity, measurement.timestamp);
  const bool retain_for_propagation =
      propagate && (!have_estimate_ || measurement.timestamp >= latest_time_);
  if (retain_for_propagation) {
    if (have_estimate_) DiscardCoveredImu(latest_time_, propagation);
    CheckBufferLimit("IMU propagation", *propagation, capacity,
                     measurement.timestamp);
  }
  imu->push_back(measurement);
  if (retain_for_propagation) {
    try {
      propagation->push_back(measurement);
    } catch (...) {
      imu->pop_back();
      throw;
    }
  }
}

std::optional<double> LivoSystem::CorrectImuTimestamp(
    double lidar_time, double previous_imu_time, Imu* measurement) const {
  double timestamp = measurement->timestamp;
  if (options_.driver_fix) timestamp += std::round(lidar_time - timestamp);
  if (previous_imu_time > 0 && timestamp < previous_imu_time) {
    return std::nullopt;
  }
  measurement->timestamp = NanosecondTime(timestamp, options_.imu_time_range);
  return timestamp;
}

void LivoSystem::AcceptLidarAnchor(TimedPointCloud measurement) {
  // Use the new LiDAR anchor for correction. After a clock rollback, include
  // queued measurements when checking the batch about to join the active IMU.
  auto imu_buffer = buffers_.imu;
  auto propagation = propagation_buffer_;
  const bool propagate = options_.propagate_imu &&
                         !frontend_.imu_needs_initialization();
  double imu_time = last_imu_;
  std::size_t rejected = 0;
  for (auto imu : startup_imu_) {
    try {
      const auto corrected =
          CorrectImuTimestamp(measurement.timestamp, imu_time, &imu);
      if (!corrected) {
        ++rejected;
        continue;
      }
      if (have_first_lidar_) {
        BufferImu(imu, propagate, &imu_buffer, &propagation);
      } else {
        imu_buffer.push_back(imu);
      }
      imu_time = *corrected;
    } catch (const std::invalid_argument&) {
      ++rejected;
    }
  }
  buffers_.point_clouds.push_back(std::move(measurement));
  const auto& limits = options_.sensor_buffer;
  // Keep the anchor and the newest usable scans from the negative-time prefix.
  TrimStartupSpan(&buffers_.point_clouds, limits.max_span_s);
  if (buffers_.point_clouds.size() >
      static_cast<std::size_t>(limits.max_lidar_scans)) {
    buffers_.point_clouds.pop_front();
  }
  if (!have_first_lidar_) {
    startup_stats_.evicted_imu +=
        TrimStartupSpan(&imu_buffer, limits.max_span_s);
    startup_stats_.evicted_images +=
        TrimStartupSpan(&buffers_.images, limits.max_span_s);
  }
  if (propagate && imu_buffer.size() > buffers_.imu.size()) {
    newest_imu_ = imu_buffer.back();
    new_imu_ = true;
  }
  startup_stats_.rejected_imu += rejected;
  startup_imu_.clear();
  buffers_.imu.swap(imu_buffer);
  propagation_buffer_.swap(propagation);
  last_imu_ = buffers_.latest_imu_time = imu_time;
  RecordBufferPeak(buffers_.imu, &imu_peak_);
  RecordBufferPeak(buffers_.images, &image_peak_);
  RecordBufferPeak(propagation_buffer_, &propagation_peak_);
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
  const int max_images = options_.sensor_buffer.max_images;
  if (have_first_lidar_) {
    CheckBufferLimit("image", buffers_.images, max_images, timestamp);
  }
  buffers_.images.push_back(std::move(measurement));
  if (!have_first_lidar_) {
    ++startup_stats_.buffered_images;
    const auto capacity = std::min(kStartupImageLimit, max_images);
    if (buffers_.images.size() > static_cast<std::size_t>(capacity)) {
      buffers_.images.pop_front();
      ++startup_stats_.evicted_images;
    }
    RecordBufferPeak(buffers_.images, &startup_image_peak_);
  } else {
    RecordBufferPeak(buffers_.images, &image_peak_);
  }
  last_image_ = timestamp;
  return true;
}

StartupBufferStats LivoSystem::GetStartupBufferStats() const noexcept {
  auto stats = startup_stats_;
  stats.pending_imu = startup_imu_.size();
  stats.pending_images = have_first_lidar_ ? 0 : buffers_.images.size();
  return stats;
}

SensorBufferCounts LivoSystem::GetSensorBufferCounts() const noexcept {
  return {buffers_.point_clouds.size(),
          buffers_.imu.size() + startup_imu_.size(), buffers_.images.size(),
          propagation_buffer_.size()};
}

bool LivoSystem::WouldExceedInputLimit(const LidarScan& scan) const {
  const auto& limits = options_.sensor_buffer;
  return scan.timestamp >= last_lidar_ &&
         (last_lidar_ >= 0 || scan.timestamp < 0) &&
         ExceedsBufferLimit(buffers_.point_clouds, limits.max_lidar_scans,
                            scan.timestamp, limits.max_span_s);
}

bool LivoSystem::WouldExceedInputLimit(const Imu& imu) const {
  if (last_lidar_ < 0 ||
      (buffers_.imu.empty() && propagation_buffer_.empty())) {
    return false;
  }
  auto corrected = imu;
  try {
    corrected.timestamp = NanosecondTime(
        imu.timestamp - options_.imu_time_offset, options_.imu_time_range);
    if (!CorrectImuTimestamp(last_lidar_, last_imu_, &corrected)) return false;
  } catch (const std::invalid_argument&) {
    return false;
  }
  const auto& limits = options_.sensor_buffer;
  return ExceedsBufferLimit(buffers_.imu, limits.max_imu_samples,
                            corrected.timestamp, limits.max_span_s) ||
         (options_.propagate_imu && !frontend_.imu_needs_initialization() &&
          (!have_estimate_ || corrected.timestamp >= latest_time_) &&
          ExceedsBufferLimit(propagation_buffer_, limits.max_imu_samples,
                             corrected.timestamp, limits.max_span_s));
}

bool LivoSystem::WouldExceedInputLimit(const TimedImage& image) const {
  const auto& limits = options_.sensor_buffer;
  return have_first_lidar_ && image.timestamp - last_image_ >= 0.02 &&
         ExceedsBufferLimit(buffers_.images, limits.max_images, image.timestamp,
                            limits.max_span_s);
}

SyncStatus LivoSystem::ProcessNext(const ResultCallback& callback) {
  PumpArchive();
  return ProcessMeasurement(callback);
}

SyncStatus LivoSystem::ProcessMeasurement(const ResultCallback& callback) {
  CheckInputOpen();
  const auto sync_start = StartTiming(options_.stage_timing);
  const auto status = synchronizer_.TrySynchronize(&buffers_, &measurements_);
  if (status != SyncStatus::kReady) return status;
  LogStageTiming("synchronization", sync_start);
  const auto start = std::chrono::steady_clock::now();
  const auto result = frontend_.Process(&measurements_);
  const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count();
  if (options_.stage_timing) LogStageSeconds("frontend_process", elapsed);
  if (result.updated && options_.propagate_imu) {
    latest_state_ = result.estimate;
    latest_time_ = result.timestamp;
    state_updated_ = have_estimate_ = true;
  }
  const auto colored_cloud = colorizer_.Process(result);
  writer_.Write(result, colored_cloud);
  if (pose_graph_ && keyframe_selector_.Select(result)) {
    try {
      AppendKeyframe(std::make_shared<const Keyframe>(
          CaptureKeyframe(next_keyframe_id_, result)));
    } catch (...) {
      RecordFailure(std::current_exception());
      throw;
    }
  }
  if (result.updated && callback) callback(result, colored_cloud, elapsed);
  return status;
}

void LivoSystem::ProcessAvailable(const ResultCallback& callback) {
  while (ProcessNext(callback) != SyncStatus::kWaiting) {
  }
  WaitForKeyframes();
}

void LivoSystem::WaitForKeyframes() {
  if (!pose_graph_) return;
  while (committed_keyframes_->frame_count() < next_keyframe_id_) {
    PumpArchive();
    if (committed_keyframes_->frame_count() < next_keyframe_id_)
      pose_graph_->Wait();
  }
}

std::exception_ptr LivoSystem::WaitForControl(
    const pose_graph::ControlReceipt& receipt) {
  for (;;) {
    PumpArchive();
    if (auto result = pose_graph_->CheckControl(receipt)) {
      // The receipt may have arrived just after the first publication check.
      PumpArchive();
      return *result;
    }
    pose_graph_->Wait(&receipt);
  }
}

void LivoSystem::SealInput() {
  PumpArchive();
  if (input_sealed_) return;
  try {
    if (pose_graph_) WaitForControl(*pose_graph_->SealInput());
    if (last_lidar_ < 0) {
      startup_imu_.clear();
      buffers_.images.clear();
    }
    input_sealed_ = true;
  } catch (...) {
    RecordFailure(std::current_exception());
    PumpArchive();
    throw;
  }
}

void LivoSystem::Finish() {
  if (IsFinished()) return;
  if (!session_error_ && active_loop_backend_)
    throw std::logic_error(
        "Finish the standalone backend with LoopBackend::Finish");
  FinishSession(nullptr);
}

void LivoSystem::FinishSession(LoopBackend* backend) {
  try {
    PumpArchive();
    if (!loop_pose_updates_claimed_) writer_.Finish();
    SealInput();
    if (pose_graph_) {
      for (;;) {
        if (backend) {
          backend->Poll();
        } else {
          PumpArchive();
        }
        if (pose_graph_->Drained()) break;
        pose_graph_->Wait(nullptr, backend != nullptr);
      }
    }
    if (loop_pose_updates_claimed_) writer_.Finish();
    if (pose_graph_) {
      WaitForControl(*pose_graph_->FinishArchive());
      pose_graph_->Stop();
    }
    ReportBufferPeaks();
    finished_ = true;
  } catch (...) {
    RecordFailure(std::current_exception());
    if (pose_graph_) {
      pose_graph_->Stop();
      // A write that was already running when the owner failed can finish.
      // Keep its committed view and preserve the first exception.
      try {
        auto progress = pose_graph_->TakeProgress();
        if (progress.commit) DeliverArchiveCommit(std::move(*progress.commit));
      } catch (...) {
      }
    }
    std::rethrow_exception(session_error_);
  }
}

bool LivoSystem::IsFinished() const noexcept {
  return finished_ && !session_error_;
}

void LivoSystem::AppendKeyframe(std::shared_ptr<const Keyframe> keyframe) {
  try {
    pose_graph_->AddKeyframe(std::move(keyframe));
    ++next_keyframe_id_;
  } catch (...) {
    RecordFailure(std::current_exception());
    PumpArchive();
    throw;
  }
}

void LivoSystem::CheckOnlineMapping() const {
  if (!frozen_poses_) throw std::logic_error("Online submaps are not enabled");
}

void LivoSystem::CheckCallbackRegistration() {
  CheckInputOpen();
  if (next_keyframe_id_)
    throw std::logic_error("Register online callback before processing frames");
}

void LivoSystem::SetOnlineSubmapCallback(OnlineSubmapCallback callback) {
  CheckOnlineMapping();
  if (callback) CheckCallbackRegistration();
  submap_callback_ = std::move(callback);
}

void LivoSystem::SetOnlineSubmapDeltaCallback(
    OnlineSubmapDeltaCallback callback) {
  CheckOnlineMapping();
  if (callback) CheckCallbackRegistration();
  submap_delta_callback_ = std::move(callback);
}

void LivoSystem::ApplyPoseUpdate(const PoseUpdate& update) {
  PumpArchive();
  CheckOnlineMapping();
  if (finished_) throw std::logic_error("Online mapping has finished");
  if (loop_pose_updates_claimed_)
    throw std::logic_error("Online loop backend owns pose updates");
  std::exception_ptr rejection;
  try {
    rejection = WaitForControl(*pose_graph_->ApplyPoseUpdate(update));
  } catch (...) {
    RecordFailure(std::current_exception());
    PumpArchive();
    throw;
  }
  if (rejection) std::rethrow_exception(rejection);
}

void LivoSystem::EnableLoops(LoopAdmissionOptions options,
                            LoopTaskRunner runner, LoopBackend* backend) {
  CheckInputOpen();
  CheckOnlineMapping();
  if (loop_pose_updates_claimed_ || next_keyframe_id_ ||
      committed_frontier_.version)
    throw std::logic_error("Loop backend requires a new unclaimed session");
  if (options_.max_pending_keyframes) {
    options.max_pending_keyframes = *options_.max_pending_keyframes;
  }
  pose_graph_->EnableLoops(std::move(options), std::move(runner),
                           backend != nullptr);
  loop_pose_updates_claimed_ = true;
  active_loop_backend_ = backend;
}

void LivoSystem::ClearLoopBackend(const LoopBackend& backend) noexcept {
  if (active_loop_backend_ != &backend) return;
  active_loop_backend_ = nullptr;
  if (!finished_) pose_graph_->AbandonStandalone();
}

void LivoSystem::RecordFailure(std::exception_ptr error) noexcept {
  if (!session_error_ && error) {
    session_error_ = pose_graph_ ? pose_graph_->Cancel(std::move(error))
                                 : std::move(error);
  }
}

void LivoSystem::PumpArchive() {
  try {
    if (pose_graph_) {
      auto progress = pose_graph_->TakeProgress();
      if (progress.failure) RecordFailure(std::move(progress.failure));
      if (progress.commit) DeliverArchiveCommit(std::move(*progress.commit));
    }
    if (session_error_) std::rethrow_exception(session_error_);
  } catch (...) {
    RecordFailure(std::current_exception());
    std::rethrow_exception(session_error_);
  }
}

void LivoSystem::DeliverArchiveCommit(pose_graph::ArchiveCommit commit) {
  if (commit.submaps) {
    auto& update = *commit.submaps;
    if (update.keyframe) {
      appended_poses_.push_back({update.keyframe->id, *update.appended_pose});
    } else if (update.poses) {
      frozen_poses_ = update.poses;
      appended_poses_.clear();
    }
    committed_frontier_ = update.frontier;
    map_correction_ = commit.correction;
    for (const auto& revision : update.revisions) {
      submap_count_ = std::max(submap_count_, revision.primary.id + 1);
    }
  }
  committed_keyframes_ = std::move(commit.keyframes);
  if (commit.submaps) {
    auto& update = *commit.submaps;
    if (geometry_) UpdateGeometry(update);
    if (submap_delta_callback_) submap_delta_callback_(update);
    if (submap_callback_) {
      auto poses = GetPoseSnapshot();
      submap_callback_({std::move(update.keyframe), std::move(update.revisions),
                        std::move(poses), std::move(update.pose_changes)});
    }
  }
  pose_graph_->AcknowledgeCommit();
}

void LivoSystem::UpdateGeometry(const OnlineSubmapDelta& update) {
  const auto start = std::chrono::steady_clock::now();
  if (update.poses) {
    geometry_->Update(update.revisions, update.poses);
  } else {
    geometry_->UpdateDelta(update.revisions, update.frontier,
                           update.keyframe.get(),
                           update.appended_pose ? &*update.appended_pose
                                                : nullptr);
  }
  const auto total =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count();
  const auto stats = geometry_->GetStats();
  std::ostringstream message;
  message.imbue(std::locale::classic());
  message << std::setprecision(17) << "geometry_total_seconds=" << total
          << " geometry_update_seconds=" << stats.last_update_seconds
          << " geometry_cloud_read_seconds=" << stats.last_cloud_read_seconds
          << " geometry_full_build_seconds=" << stats.last_full_build_seconds
          << " geometry_snapshot_seconds=" << stats.last_snapshot_seconds;
  LOG(INFO) << message.str();
}

GeometrySubmap LivoSystem::GetGeometrySubmap(std::uint64_t id) {
  if (session_error_) std::rethrow_exception(session_error_);
  if (!geometry_) throw std::logic_error("Geometry submaps are not enabled");
  if (id >= submap_count_)
    throw std::out_of_range("Unknown geometry submap ID");
  try {
    return geometry_->GetSubmap(id);
  } catch (...) {
    RecordFailure(std::current_exception());
    throw;
  }
}

GeometryCacheStats LivoSystem::GetGeometryStats() const {
  if (!geometry_) throw std::logic_error("Geometry submaps are not enabled");
  return geometry_->GetStats();
}

std::optional<PropagationResult> LivoSystem::PropagateLatest() {
  CheckInputOpen();
  if (frontend_.imu_needs_initialization() || !new_imu_ || !have_estimate_)
    return {};
  new_imu_ = false;
  if (state_updated_) {
    propagated_ = latest_state_;
    DiscardCoveredImu(latest_time_, &propagation_buffer_);
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

LivoSystem::~LivoSystem() {
  if (pose_graph_) pose_graph_->Stop();
}

PoseFrontier LivoSystem::GetPoseFrontier() const {
  CheckOnlineMapping();
  return committed_frontier_;
}

std::shared_ptr<const PoseSnapshot> LivoSystem::GetPoseSnapshot() const {
  CheckOnlineMapping();
  if (appended_poses_.empty()) return frozen_poses_;
  auto snapshot = std::make_shared<PoseSnapshot>(*frozen_poses_);
  snapshot->poses.insert(snapshot->poses.end(), appended_poses_.begin(),
                         appended_poses_.end());
  snapshot->frame_count = committed_frontier_.frame_count;
  return snapshot;
}

MapCorrection LivoSystem::GetMapCorrection() const {
  CheckOnlineMapping();
  if (!map_correction_)
    throw std::invalid_argument(
        "Map correction requires a projected optimized anchor");
  if (!map_correction_->map_from_odom.rotation.allFinite() ||
      !map_correction_->map_from_odom.translation.allFinite())
    throw std::invalid_argument("Derived map correction must be finite");
  return *map_correction_;
}

KeyframeReadView LivoSystem::GetKeyframeReadView() const {
  if (!committed_keyframes_)
    throw std::logic_error("Keyframe storage is not enabled");
  return *committed_keyframes_;
}

LoopSessionStats LivoSystem::LoopStats() const {
  auto stats = pose_graph_->GetStats();
  // A durable pose can be waiting for delivery while the owner reads the
  // previous snapshot. Expose the counts of that same delivered frontier.
  if (stats.accepted > committed_frontier_.version) {
    stats.completed -= stats.accepted - committed_frontier_.version;
    stats.accepted = committed_frontier_.version;
  }
  stats.pose_version = committed_frontier_.version;
  stats.frame_count = committed_frontier_.frame_count;
  return stats;
}

std::optional<LoopSessionStats> LivoSystem::GetLoopStats() const {
  if (!options_.loop_closure) return std::nullopt;
  return LoopStats();
}

}  // namespace livo_atlas
