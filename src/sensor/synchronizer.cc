#include "livo_atlas/sensor/synchronizer.h"

#include <cassert>
#include <cstddef>

namespace livo_atlas {
namespace {

bool IsEmpty(const TimedPointCloud& measurement) {
  return !measurement.cloud || measurement.cloud->empty();
}

SyncStatus SynchronizeScan(FrontendMode mode, SensorBuffers* buffers,
                           SynchronizedMeasurements* measurements) {
  auto& output = *measurements;
  const bool use_imu = mode == FrontendMode::kLidarInertial;
  if (use_imu && buffers->point_clouds.front().cloud->size() <= 1) {
    buffers->point_clouds.pop_front();
    return SyncStatus::kInvalidCloud;
  }
  if (use_imu && output.last_lidar_update_time < 0.0) {
    output.last_lidar_update_time = buffers->point_clouds.front().timestamp;
  }
  output.lidar_cloud = buffers->point_clouds.front().cloud;
  output.scan_start_time = buffers->point_clouds.front().timestamp;
  output.scan_end_time =
      output.scan_start_time + output.lidar_cloud->back().curvature / 1000.0;
  if (use_imu) {
    output.current_cloud = output.lidar_cloud;
    if (buffers->latest_imu_time < output.scan_end_time) {
      return SyncStatus::kWaiting;
    }
  }

  UpdateMeasurements group;
  group.lidar_update_time = output.scan_end_time;
  if (use_imu) {
    while (!buffers->imu.empty()) {
      if (buffers->imu.front().timestamp > group.lidar_update_time) break;
      group.imu.push_back(buffers->imu.front());
      buffers->imu.pop_front();
    }
  }
  buffers->point_clouds.pop_front();
  output.update_kind =
      use_imu ? UpdateKind::kLidarInertial : UpdateKind::kLidarOnly;
  output.updates.push_back(group);
  return SyncStatus::kReady;
}

SyncStatus SynchronizeLivo(double exposure_time, SensorBuffers* buffers,
                           SynchronizedMeasurements* measurements) {
  auto& output = *measurements;
  const double image_time = buffers->images.front().timestamp + exposure_time;
  if (output.update_kind == UpdateKind::kLidarInertial) {
    output.update_kind = UpdateKind::kVisual;
    output.updates.clear();
    UpdateMeasurements group;
    group.visual_update_time = image_time;
    group.lidar_update_time = output.last_lidar_update_time;
    group.image = buffers->images.front().bgr;
    buffers->images.pop_front();
    output.updates.push_back(group);
    return SyncStatus::kReady;
  }
  if (output.update_kind != UpdateKind::kWaiting &&
      output.update_kind != UpdateKind::kVisual) {
    return SyncStatus::kWaiting;
  }
  if (output.last_lidar_update_time < 0.0) {
    output.last_lidar_update_time = buffers->point_clouds.front().timestamp;
  }
  const auto& latest_cloud = buffers->point_clouds.back();
  const double latest_cloud_time =
      latest_cloud.timestamp + latest_cloud.cloud->back().curvature / 1000.0;
  if (image_time < output.last_lidar_update_time + 0.00001) {
    buffers->images.pop_front();
    return SyncStatus::kImageDropped;
  }
  if (image_time > latest_cloud_time ||
      image_time > buffers->imu.back().timestamp) {
    return SyncStatus::kWaiting;
  }

  UpdateMeasurements group;
  group.lidar_update_time = image_time;
  while (!buffers->imu.empty()) {
    if (buffers->imu.front().timestamp > group.lidar_update_time) break;
    if (buffers->imu.front().timestamp > output.last_lidar_update_time) {
      group.imu.push_back(buffers->imu.front());
    }
    buffers->imu.pop_front();
  }

  // Preserve the original remainder semantics: copy it in full, without
  // repartitioning it against the next image's time.
  *output.current_cloud = *output.next_cloud;
  PointCloud().swap(*output.next_cloud);
  const std::size_t capacity =
      output.current_cloud->size() + 24000 * buffers->point_clouds.size();
  output.current_cloud->reserve(capacity);
  output.next_cloud->reserve(capacity);
  while (!buffers->point_clouds.empty()) {
    const auto& scan = buffers->point_clouds.front();
    if (scan.timestamp > image_time) break;
    if (!IsEmpty(scan)) {
      const double frame_time = scan.timestamp;
      const float cutoff_ms = (group.lidar_update_time - frame_time) * 1000.0f;
      for (auto point : scan.cloud->points) {
        if (point.curvature < cutoff_ms) {
          point.curvature +=
              (frame_time - output.last_lidar_update_time) * 1000.0f;
          output.current_cloud->points.push_back(point);
        } else {
          point.curvature += (frame_time - group.lidar_update_time) * 1000.0f;
          output.next_cloud->points.push_back(point);
        }
      }
    }
    buffers->point_clouds.pop_front();
  }
  output.updates.push_back(group);
  output.update_kind = UpdateKind::kLidarInertial;
  return SyncStatus::kReady;
}

}  // namespace

SensorSynchronizer::SensorSynchronizer(FrontendMode mode, double exposure_time)
    : mode_(mode), exposure_time_(exposure_time) {}

SyncStatus SensorSynchronizer::TrySynchronize(
    SensorBuffers* buffers, SynchronizedMeasurements* measurements) const {
  assert(buffers != nullptr);
  assert(measurements != nullptr);
  if (buffers->point_clouds.empty()) return SyncStatus::kWaiting;
  if (IsEmpty(buffers->point_clouds.front())) {
    buffers->point_clouds.pop_front();
    return SyncStatus::kInvalidCloud;
  }
  if (IsEmpty(buffers->point_clouds.back())) {
    buffers->point_clouds.pop_back();
    return SyncStatus::kInvalidCloud;
  }
  if (mode_ != FrontendMode::kLidarOnly && buffers->imu.empty()) {
    return SyncStatus::kWaiting;
  }
  switch (mode_) {
    case FrontendMode::kLidarOnly:
    case FrontendMode::kLidarInertial:
      return SynchronizeScan(mode_, buffers, measurements);
    case FrontendMode::kLidarInertialVisual:
      if (buffers->images.empty()) return SyncStatus::kWaiting;
      return SynchronizeLivo(exposure_time_, buffers, measurements);
  }
  return SyncStatus::kWaiting;
}

}  // namespace livo_atlas
