#ifndef LIVO_ATLAS_ROS2_NODE_OPTIONS_H_
#define LIVO_ATLAS_ROS2_NODE_OPTIONS_H_

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "livo_atlas/core/livo_config.h"

namespace livo_atlas::ros2 {

struct NodeOptions {
  LivoConfig livo;
  std::string reliability = "best_effort";
  int imu_depth = 2000;
  int lidar_depth = 100;
  int image_depth = 20;
};

NodeOptions ReadNodeOptions(rclcpp::Node& node);

}  // namespace livo_atlas::ros2
#endif  // LIVO_ATLAS_ROS2_NODE_OPTIONS_H_
