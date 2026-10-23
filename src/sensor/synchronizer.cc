#include "livo_atlas/sensor/synchronizer.h"

#include <cassert>
#include <cstddef>

namespace livo_atlas {
namespace {

bool IsEmpty(const PointCloudMeasurement& measurement) {
  return !measurement.cloud || measurement.cloud->empty();
}

SynchronizationResult SynchronizeScan(
    FrontendMode mode, SensorBuffers* buffers,
    SynchronizedMeasurements* measurements) {
  auto& output = *measurements;
  const bool use_imu = mode == FrontendMode::kLidarInertial;
  if (use_imu && buffers->point_clouds.front().cloud->size() <= 1) {
    buffers->point_clouds.pop_front();
    return SynchronizationResult::kInvalidCloud;
  }
  if (use_imu && output.last_lio_update_time < 0.0) {
    output.last_lio_update_time = buffers->point_clouds.front().timestamp;
  }
  output.lidar = buffers->point_clouds.front().cloud;
  output.lidar_frame_beg_time = buffers->point_clouds.front().timestamp;
  output.lidar_frame_end_time =
      output.lidar_frame_beg_time + output.lidar->back().curvature / 1000.0;
  if (use_imu) {
    output.pcl_proc_cur = output.lidar;
    if (buffers->latest_imu_time < output.lidar_frame_end_time) {
      return SynchronizationResult::kWaiting;
    }
  }

  MeasurementGroup group;
  group.lio_time = output.lidar_frame_end_time;
  if (use_imu) {
    while (!buffers->imu.empty()) {
      if (buffers->imu.front().timestamp > group.lio_time) break;
      group.imu.push_back(buffers->imu.front());
      buffers->imu.pop_front();
    }
  }
  buffers->point_clouds.pop_front();
  output.lio_vio_flg =
      use_imu ? UpdateKind::kLidarInertial : UpdateKind::kLidarOnly;
  output.measures.push_back(group);
  return SynchronizationResult::kReady;
}

SynchronizationResult SynchronizeLivo(
    double exposure_time, SensorBuffers* buffers,
    SynchronizedMeasurements* measurements) {
  auto& output = *measurements;
  const double image_time = buffers->images.front().timestamp + exposure_time;
  if (output.lio_vio_flg == UpdateKind::kLidarInertial) {
    output.lio_vio_flg = UpdateKind::kVisual;
    output.measures.clear();
    MeasurementGroup group;
    group.vio_time = image_time;
    group.lio_time = output.last_lio_update_time;
    group.img = buffers->images.front().bgr;
    buffers->images.pop_front();
    output.measures.push_back(group);
    return SynchronizationResult::kReady;
  }
  if (output.lio_vio_flg != UpdateKind::kWaiting &&
      output.lio_vio_flg != UpdateKind::kVisual) {
    return SynchronizationResult::kWaiting;
  }
  if (output.last_lio_update_time < 0.0) {
    output.last_lio_update_time = buffers->point_clouds.front().timestamp;
  }
  const auto& latest_cloud = buffers->point_clouds.back();
  const double latest_cloud_time =
      latest_cloud.timestamp + latest_cloud.cloud->back().curvature / 1000.0;
  if (image_time < output.last_lio_update_time + 0.00001) {
    buffers->images.pop_front();
    return SynchronizationResult::kImageDropped;
  }
  if (image_time > latest_cloud_time ||
      image_time > buffers->imu.back().timestamp) {
    return SynchronizationResult::kWaiting;
  }

  MeasurementGroup group;
  group.lio_time = image_time;
  while (!buffers->imu.empty()) {
    if (buffers->imu.front().timestamp > group.lio_time) break;
    if (buffers->imu.front().timestamp > output.last_lio_update_time) {
      group.imu.push_back(buffers->imu.front());
    }
    buffers->imu.pop_front();
  }

  // Preserve the original remainder semantics: copy it in full, without
  // repartitioning it against the next image's time.
  *output.pcl_proc_cur = *output.pcl_proc_next;
  PointCloud().swap(*output.pcl_proc_next);
  const std::size_t capacity =
      output.pcl_proc_cur->size() + 24000 * buffers->point_clouds.size();
  output.pcl_proc_cur->reserve(capacity);
  output.pcl_proc_next->reserve(capacity);
  while (!buffers->point_clouds.empty()) {
    const auto& scan = buffers->point_clouds.front();
    if (scan.timestamp > image_time) break;
    if (!IsEmpty(scan)) {
      const double frame_time = scan.timestamp;
      const float cutoff_ms = (group.lio_time - frame_time) * 1000.0f;
      for (auto point : scan.cloud->points) {
        if (point.curvature < cutoff_ms) {
          point.curvature +=
              (frame_time - output.last_lio_update_time) * 1000.0f;
          output.pcl_proc_cur->points.push_back(point);
        } else {
          point.curvature += (frame_time - group.lio_time) * 1000.0f;
          output.pcl_proc_next->points.push_back(point);
        }
      }
    }
    buffers->point_clouds.pop_front();
  }
  output.measures.push_back(group);
  output.lio_vio_flg = UpdateKind::kLidarInertial;
  return SynchronizationResult::kReady;
}

}  // namespace

SensorSynchronizer::SensorSynchronizer(FrontendMode mode, double exposure_time)
    : mode_(mode), exposure_time_(exposure_time) {}

SynchronizationResult SensorSynchronizer::TrySynchronize(
    SensorBuffers* buffers, SynchronizedMeasurements* measurements) const {
  assert(buffers != nullptr);
  assert(measurements != nullptr);
  if (buffers->point_clouds.empty()) return SynchronizationResult::kWaiting;
  if (IsEmpty(buffers->point_clouds.front())) {
    buffers->point_clouds.pop_front();
    return SynchronizationResult::kInvalidCloud;
  }
  if (IsEmpty(buffers->point_clouds.back())) {
    buffers->point_clouds.pop_back();
    return SynchronizationResult::kInvalidCloud;
  }
  if (mode_ != FrontendMode::kLidarOnly && buffers->imu.empty()) {
    return SynchronizationResult::kWaiting;
  }
  switch (mode_) {
    case FrontendMode::kLidarOnly:
    case FrontendMode::kLidarInertial:
      return SynchronizeScan(mode_, buffers, measurements);
    case FrontendMode::kLidarInertialVisual:
      if (buffers->images.empty()) return SynchronizationResult::kWaiting;
      return SynchronizeLivo(exposure_time_, buffers, measurements);
  }
  return SynchronizationResult::kWaiting;
}

}  // namespace livo_atlas
