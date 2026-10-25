#ifndef LIVO_ATLAS_TESTS_INSTALLED_CONSUMER_RUNTIME_SCENARIO_H_
#define LIVO_ATLAS_TESTS_INSTALLED_CONSUMER_RUNTIME_SCENARIO_H_

#include <Eigen/Core>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <opencv2/core.hpp>
#include <string>
#include <utility>
#include <vector>

#include "livo_atlas/core/livo_system.h"

namespace livo_atlas::test_data {

inline LidarScan RuntimeScan(double timestamp) {
  std::vector<LivoxPoint> points;
  for (int y = 0; y < 16; ++y) {
    for (int x = 0; x < 16; ++x) {
      LivoxPoint point;
      point.x = (x - 7.5) * 0.125;
      point.y = (y - 7.5) * 0.125;
      point.z = 4;
      point.offset_time = static_cast<std::uint32_t>(
          (std::int64_t{y * 16 + x} * 125000000) / 255);
      point.line = x % 6;
      point.tag = 0x10;
      points.push_back(point);
    }
  }
  return {timestamp, std::move(points)};
}

// Exercise synchronization, estimation and file output through the installed
// runtime. This consumer has no source-tree helpers or ROS publishers.
inline bool RunSystemScenario(FrontendMode mode) {
  const bool inertial = mode != FrontendMode::kLidarOnly;
  const bool visual = mode == FrontendMode::kLidarInertialVisual;
  SystemOptions options;
  options.frontend.calibration.camera = {"Pinhole", 512, 512, 1, 200,
                                         200,       256, 256, {}};
  options.frontend.imu.enabled = inertial;
  options.frontend.imu.initialization_frames = 1;
  options.frontend.surface_filter_size = 0.1;
  options.frontend.visual.grid_size = 32;
  options.frontend.visual.normal_enabled = false;
  options.image_enabled = visual;
  options.preprocess.point_filter = 1;
  options.exporter.directory =
      std::filesystem::temp_directory_path() /
      ("livo_atlas_consumer_" +
       std::to_string(
           std::chrono::steady_clock::now().time_since_epoch().count()));
  const auto directory = options.exporter.directory;
  int lidar_updates = 0;
  int visual_updates = 0;
  bool valid = true;
  bool tracked = false;
  {
    LivoSystem session(options);
    for (int frame = 0; frame < 6; ++frame) {
      const double timestamp = 100 + frame * 0.25;
      valid &= session.AddLidar(RuntimeScan(timestamp));
      if (inertial) {
        for (double offset : {0.0625, 0.125, 0.1875}) {
          valid &= session.AddImu(
              {timestamp + offset, Eigen::Vector3d::Zero(), {0, 0, -9.81}});
        }
      }
      if (visual) {
        cv::Mat image(512, 512, CV_8UC3);
        for (int y = 0; y < image.rows; ++y) {
          for (int x = 0; x < image.cols; ++x) {
            const unsigned char gray = (x * 31 + y * 17 + x * y * 13) % 256;
            image.at<cv::Vec3b>(y, x) = {gray, gray, gray};
          }
        }
        valid &= session.AddImage({timestamp + 0.125, std::move(image)});
      }
      session.ProcessAvailable([&](const auto& result, const auto&, double) {
        valid &= result.estimate.pos_end.allFinite() &&
                 result.estimate.cov.allFinite();
        if (result.kind == UpdateKind::kVisual) {
          ++visual_updates;
          valid &=
              !result.color_image.empty() && !result.tracking_image.empty();
          if (!result.color_image.empty() && !result.tracking_image.empty()) {
            tracked |= cv::norm(result.color_image, result.tracking_image,
                                cv::NORM_INF) > 0;
          }
        } else {
          ++lidar_updates;
          valid &= result.world_cloud && !result.world_cloud->empty();
        }
      });
    }
    session.Finish();
  }
  valid &= std::filesystem::file_size(directory / "mat_out.txt") > 0;
  std::filesystem::remove_all(directory);
  return valid && lidar_updates > 2 &&
         (!visual || (visual_updates > 2 && tracked));
}

}  // namespace livo_atlas::test_data

#endif  // LIVO_ATLAS_TESTS_INSTALLED_CONSUMER_RUNTIME_SCENARIO_H_
