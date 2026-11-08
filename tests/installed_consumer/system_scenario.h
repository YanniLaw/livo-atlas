#ifndef LIVO_ATLAS_TESTS_INSTALLED_CONSUMER_RUNTIME_SCENARIO_H_
#define LIVO_ATLAS_TESTS_INSTALLED_CONSUMER_RUNTIME_SCENARIO_H_

#include <Eigen/Core>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <opencv2/core.hpp>
#include <string>
#include <utility>
#include <vector>

#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/io/keyframe_store.h"
#include "livo_atlas/io/online_submap_store.h"
#include "livo_atlas/io/submap_store.h"

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
inline bool RunSystemScenario(FrontendMode mode, bool online = false) {
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
  options.keyframes.enabled = true;
  options.keyframes.time_threshold = 0;
  options.submaps = {true, 2};
  if (online) options.submap_mode = SubmapMode::kOnline;
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
  std::shared_ptr<const Keyframe> held;
  {
    LivoSystem session(options);
    if (online)
      session.SetOnlineSubmapCallback([&](const auto& update) {
        if (update.keyframe && !held) held = update.keyframe;
      });
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
      if (online && (frame == 3 || frame == 5)) {
        const auto snapshot = session.GetPoseSnapshot();
        PoseUpdate update{snapshot->version, snapshot->frame_count,
                          snapshot->poses};
        for (auto& pose : update.poses)
          pose.map_from_body.translation.x() += 10;
        session.ApplyPoseUpdate(update);
      }
    }
    session.Finish();
    if (online) valid &= session.GetPoseSnapshot()->version == 2;
  }
  valid &= std::filesystem::file_size(directory / "mat_out.txt") > 0;
  KeyframeStore dataset(directory / "keyframes");
  valid &= dataset.ids().size() ==
           static_cast<std::size_t>(visual ? visual_updates : lidar_updates);
  std::vector<KeyframePose> poses;
  for (auto id : dataset.ids()) {
    const auto frame = dataset.Read(id);
    auto corrected = frame.odom_from_body;
    corrected.translation.x() += 10;
    poses.push_back({id, corrected});
    const auto cloud = TransformKeyframeCloud(frame, corrected);
    valid &= !cloud.empty() && std::isfinite(cloud.front().x);
    if (visual) valid &= !frame.image.empty();
  }
  const std::uint64_t version = online ? 2 : 1;
  if (online) {
    OnlineSubmapStore index(directory / "keyframes/online_submaps", dataset);
    const auto materialized =
        index.MaterializePoseSnapshot(2, dataset.ids().size());
    poses = materialized.poses;
    valid &= held && !held->cloud->empty();
    if (visual)
      valid &= held && !held->image.empty() &&
               cv::norm(held->image, dataset.Read(held->id).image,
                        cv::NORM_INF) == 0;
    SubmapStore compatible(directory / "keyframes/submaps", dataset, 2);
    SubmapBuilder builder(2);
    for (auto id : dataset.ids())
      if (const auto group = builder.AddKeyframe(id)) compatible.Append(*group);
    if (const auto tail = builder.Finish()) compatible.Append(*tail);
    for (std::size_t id = 0; id < index.size(); ++id)
      valid &= index.Read(id).state == OnlineSubmapState::kSealed;
  }
  dataset.WritePoses(version, poses);
  valid &= dataset.ReadPoses(version).size() == dataset.ids().size();
  SubmapStore submaps(directory / "keyframes/submaps", dataset);
  valid &= submaps.size() == (dataset.ids().size() + 1) / 2;
  std::size_t grouped = 0;
  for (std::size_t id = 0; id < submaps.size(); ++id) {
    grouped += submaps.Read(id).keyframe_ids.size();
    const auto raw = submaps.ReadCloud(id);
    const auto corrected = submaps.ReadCloud(id, version);
    valid &= raw.cloud.size() == corrected.cloud.size() && !raw.cloud.empty();
    valid &= std::abs(corrected.parent_from_submap.translation.x() -
                      raw.parent_from_submap.translation.x() -
                      (online ? 20 : 10)) < 1e-9;
  }
  valid &= grouped == dataset.ids().size();
  std::filesystem::remove_all(directory);
  return valid && lidar_updates > 2 &&
         (!visual || (visual_updates > 2 && tracked));
}

}  // namespace livo_atlas::test_data

#endif  // LIVO_ATLAS_TESTS_INSTALLED_CONSUMER_RUNTIME_SCENARIO_H_
