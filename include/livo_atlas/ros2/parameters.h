#ifndef LIVO_ATLAS_ROS2_PARAMETERS_H_
#define LIVO_ATLAS_ROS2_PARAMETERS_H_

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "livo_atlas/runtime/mapping_options.h"

namespace livo_atlas::ros2 {

struct NodeParameters {
  MappingOptions mapping;
  std::string reliability = "best_effort";
  int imu_depth = 2000;
  int lidar_depth = 100;
  int image_depth = 20;
};

NodeParameters ReadParameters(rclcpp::Node& node);

}  // namespace livo_atlas::ros2
#endif  // LIVO_ATLAS_ROS2_PARAMETERS_H_
