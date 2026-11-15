#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>

#include <opencv2/core.hpp>

#include "livo_atlas/io/keyframe_store.h"

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Expected a new dataset directory\n";
    return 2;
  }
  try {
    livo_atlas::Calibration calibration;
    calibration.camera = {"Pinhole", 8, 6, 1, 4, 4, 3.5, 2.5, {}};
    livo_atlas::KeyframeStore store(std::filesystem::path(argv[1]),
                                    calibration);
    livo_atlas::Keyframe frame;
    livo_atlas::RigidTransform camera_from_odom;
    camera_from_odom.rotation << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    camera_from_odom.translation << 1, 2, 3;
    frame.camera_from_odom = camera_from_odom;
    frame.image = cv::Mat(6, 8, CV_8UC3, cv::Scalar(10, 20, 30));
    livo_atlas::PointCloud::Ptr cloud(new livo_atlas::PointCloud);
    livo_atlas::Point point{};
    point.x = -1.5;
    point.y = 0.5;
    point.z = 1;
    cloud->push_back(point);
    frame.cloud = cloud;
    for (std::uint64_t id = 0; id < 2; ++id) {
      frame.id = id;
      frame.timestamp = 100 + id;
      frame.cloud_timestamp = frame.timestamp - 0.01;
      frame.image_timestamp = frame.timestamp + 0.01;
      store.Append(frame);
    }
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
