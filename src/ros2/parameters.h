#ifndef LIVO_ATLAS_ROS2_PARAMETERS_H_
#define LIVO_ATLAS_ROS2_PARAMETERS_H_
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "livo_atlas/runtime/mapping_session.h"
namespace livo_atlas::ros2 {
struct NodeParameters {
  SessionOptions session;
  std::string lidar_topic = "/livox/lidar";
  std::string imu_topic = "/livox/imu";
  std::string image_topic = "/left_camera/image";
  double image_time_offset = 0;
  double lidar_time_offset = 0;
  std::string reliability = "best_effort";
  int imu_depth = 2000;
  int lidar_depth = 100;
  int image_depth = 20;
};
NodeParameters ReadParameters(rclcpp::Node& node);
}  // namespace livo_atlas::ros2
#endif  // LIVO_ATLAS_ROS2_PARAMETERS_H_
