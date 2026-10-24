#ifndef LIVO_ATLAS_TESTS_INSTALLED_CONSUMER_SCENARIO_H_
#define LIVO_ATLAS_TESTS_INSTALLED_CONSUMER_SCENARIO_H_

#include <opencv2/core.hpp>

#include "livo_atlas/frontend/frontend.h"

namespace livo_atlas::test_data {

// An ordinary caller exercises repeated updates without any publisher or ROS.
inline bool RunFrontendScenario(FrontendMode mode) {
  const bool inertial = mode != FrontendMode::kLidarOnly;
  const bool visual = mode == FrontendMode::kLidarInertialVisual;
  FrontendOptions options;
  options.calibration.camera = {"Pinhole", 512, 512, 1, 200, 200, 256, 256, {}};
  options.imu.enabled = inertial;
  options.imu.initialization_frames = 1;
  options.surface_filter_size = 0.1;
  options.visual.grid_size = 32;
  options.visual.normal_enabled = false;
  Frontend frontend(options);
  int lidar_updates = 0;
  int visual_updates = 0;
  bool tracked = false;
  for (int frame = 0; frame < 4; ++frame) {
    const double begin = 100 + frame * 0.25;
    SynchronizedMeasurements input;
    input.lio_vio_flg = inertial ? UpdateKind::kLidarInertial : UpdateKind::kLidarOnly;
    input.last_lio_update_time = begin;
    input.lidar_frame_beg_time = begin;
    input.lidar_frame_end_time = begin + 0.125;
    for (int y = 0; y < 8; ++y) {
      for (int x = 0; x < 8; ++x) {
        Point point{};
        point.x = (x - 3.5) * 0.25;
        point.y = (y - 3.5) * 0.25;
        point.z = 4;
        point.curvature = 125;
        input.lidar->push_back(point);
      }
    }
    input.pcl_proc_cur = input.lidar;
    input.measures.emplace_back();
    auto& group = input.measures.back();
    group.lio_time = begin + 0.125;
    if (inertial) {
      for (double timestamp : {begin + 0.0625, begin + 0.125}) {
        group.imu.push_back({timestamp, Eigen::Vector3d::Zero(),
                            Eigen::Vector3d(0, 0, -9.81)});
      }
    }
    const auto lidar = frontend.Process(&input);
    if (frame == 0 && inertial) {
      if (lidar.updated || !lidar.imu_initialized) return false;
      continue;
    }
    if (!lidar.updated || !lidar.world_cloud || lidar.world_cloud->empty() ||
        !lidar.estimate.pos_end.allFinite() || !lidar.estimate.cov.allFinite()) return false;
    ++lidar_updates;
    if (!visual) continue;
    input.lio_vio_flg = UpdateKind::kVisual;
    group.vio_time = group.lio_time;
    group.imu.clear();
    group.img = cv::Mat(512, 512, CV_8UC3);
    for (int y = 0; y < group.img.rows; ++y) {
      for (int x = 0; x < group.img.cols; ++x) {
        const unsigned char gray = (x * 31 + y * 17 + x * y * 13) % 256;
        group.img.at<cv::Vec3b>(y, x) = {gray, gray, gray};
      }
    }
    const auto image = frontend.Process(&input);
    if (!image.updated || image.color_image.empty() || image.tracking_image.empty() ||
        !image.estimate.pos_end.allFinite() || !image.estimate.cov.allFinite()) return false;
    tracked |= cv::norm(image.color_image, image.tracking_image, cv::NORM_INF) > 0;
    ++visual_updates;
    // A caller that skips publishing must still consume this frame's cloud.
    if (frontend.Process(&input).updated) return false;
  }
  return lidar_updates == (inertial ? 3 : 4) &&
         (!visual || (visual_updates == 3 && tracked));
}

}  // namespace livo_atlas::test_data

#endif  // LIVO_ATLAS_TESTS_INSTALLED_CONSUMER_SCENARIO_H_
