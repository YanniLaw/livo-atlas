#ifndef LIVO_ATLAS_TESTS_INSTALLED_CONSUMER_EXPORT_SCENARIO_H_
#define LIVO_ATLAS_TESTS_INSTALLED_CONSUMER_EXPORT_SCENARIO_H_

#include <chrono>
#include <filesystem>
#include <fstream>
#include <opencv2/core/persistence.hpp>
#include <opencv2/imgcodecs.hpp>
#include <string>

#include "livo_atlas/common/logging.h"
#include "livo_atlas/io/colmap_exporter.h"
#include "livo_atlas/io/keyframe_store.h"
#include "livo_atlas/io/reconstruction_view.h"

namespace livo_atlas::test_data {
inline bool RunColmapScenario() {
  namespace fs = std::filesystem;
  const auto root =
      fs::temp_directory_path() /
      ("installed-colmap-" +
       std::to_string(
           std::chrono::steady_clock::now().time_since_epoch().count()));
  try {
    Calibration calibration;
    calibration.camera = {"Pinhole", 8, 6, 1, 4, 4, 3.5, 2.5, {}};
    Keyframe frame;
    frame.timestamp = frame.cloud_timestamp = 100;
    frame.image_timestamp = 100;
    frame.odom_from_body.translation << 1, 2, 3;
    frame.camera_from_odom = RigidTransform{Eigen::Matrix3d::Identity(),
                                            Eigen::Vector3d(-1, -2, -3)};
    frame.image = cv::Mat(6, 8, CV_8UC3, cv::Scalar(5, 17, 29));
    PointCloud::Ptr cloud(new PointCloud);
    Point point{};
    point.z = 4;
    point.intensity = 7;
    cloud->push_back(point);
    frame.cloud = cloud;
    const auto dataset = root / "dataset";
    KeyframeStore writer(dataset, calibration);
    writer.Append(frame);
    RigidTransform optimized;
    optimized.translation << 10, 0, 0;
    writer.WritePoses(1, {{0, optimized}});

    auto input = frame;
    input.image = frame.image.clone();
    PoseSnapshot raw_poses;
    raw_poses.frame_count = 1;
    raw_poses.poses.push_back({0, frame.odom_from_body});
    const auto raw_view =
        MakeReconstructionView(calibration.camera, input, raw_poses, {48, 1});
    auto map_poses = raw_poses;
    map_poses.version = 1;
    map_poses.optimized_frame_count = 1;
    map_poses.parent_frame = PoseFrame::kMap;
    map_poses.poses[0].map_from_body = optimized;
    const auto map_view =
        MakeReconstructionView(calibration.camera, input, map_poses, {48, 1});
    input.image.setTo(cv::Scalar(0, 0, 0));
    if (raw_view.parent_frame != PoseFrame::kOdom ||
        map_view.parent_frame != PoseFrame::kMap ||
        map_view.pose_version != 1 || raw_view.camera.cx != 3.5 ||
        raw_view.camera.cy != 2.5 ||
        !map_view.camera_from_parent.translation.isApprox(
            Eigen::Vector3d(-10, 0, 0)) ||
        raw_view.depth_mask.at<unsigned char>(3, 4) != 1 ||
        map_view.depth_mask.at<unsigned char>(3, 4) != 1 ||
        raw_view.depth_m.at<float>(3, 4) != 4 ||
        map_view.depth_m.at<float>(3, 4) != 4 ||
        raw_view.valid_depth_pixels != 1 ||
        raw_view.image_bgr.at<cv::Vec3b>(3, 4) != cv::Vec3b(5, 17, 29))
      return false;

    for (std::uint64_t version : {0, 1}) {
      const auto output = root / std::to_string(version);
      const auto result = ExportColmap(dataset, output, {version, 1});
      if (result.images != 1 || result.points != 1) return false;
      cv::FileStorage metadata((output / "export.yml").string(),
                               cv::FileStorage::READ);
      if (!metadata.isOpened() ||
          static_cast<std::string>(metadata["pose_version"]) !=
              std::to_string(version))
        return false;
      const auto image = cv::imread((output / "images/0.png").string());
      if (image.size() != frame.image.size() ||
          cv::norm(image, frame.image, cv::NORM_INF) != 0)
        return false;
      std::ifstream points(output / "sparse/0/points3D.txt");
      std::string comment;
      while (points.peek() == '#') std::getline(points, comment);
      std::uint64_t id, image_id, index;
      double x, y, z, error;
      int red, green, blue;
      points >> id >> x >> y >> z >> red >> green >> blue >> error >>
          image_id >> index;
      if (!points || id != 1 || image_id != 1 || index != 0 || red != 29 ||
          green != 17 || blue != 5 || error != 0)
        return false;
      const Eigen::Vector3d expected =
          version ? Eigen::Vector3d(10, 0, 4) : Eigen::Vector3d(1, 2, 7);
      if (!Eigen::Vector3d(x, y, z).isApprox(expected)) return false;
    }
    KeyframeStore original(dataset);
    if (original.Read(0).cloud->front().z != 4 ||
        original.Read(0).odom_from_body.translation !=
            frame.odom_from_body.translation)
      return false;
    fs::remove_all(root);
    LOG(INFO) << "Installed COLMAP export: raw and optimized data passed";
    return true;
  } catch (const std::exception& error) {
    LOG(ERROR) << root << ": " << error.what();
    return false;
  }
}
}  // namespace livo_atlas::test_data

#endif  // LIVO_ATLAS_TESTS_INSTALLED_CONSUMER_EXPORT_SCENARIO_H_
