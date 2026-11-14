#ifndef LIVO_ATLAS_ROS2_KEYFRAME_TRAJECTORY_PUBLISHER_H_
#define LIVO_ATLAS_ROS2_KEYFRAME_TRAJECTORY_PUBLISHER_H_

#include <livo_atlas/msg/keyframe_trajectory.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/context.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>

#include <string>

#include "livo_atlas/mapping/trajectory_snapshot.h"

namespace livo_atlas::ros2 {

class KeyframeTrajectoryPublisher {
 public:
  KeyframeTrajectoryPublisher(rclcpp::Node& node, std::string map_frame);
  // Called on the trajectory worker. Invalid ready payloads clear both topics
  // before throwing, so the controller can stop this derived output.
  void Publish(const TrajectorySnapshot& snapshot) const;

 private:
  std::string map_frame_;
  rclcpp::Context::SharedPtr context_;
  rclcpp::Publisher<livo_atlas::msg::KeyframeTrajectory>::SharedPtr trajectory_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_;
};

}  // namespace livo_atlas::ros2

#endif  // LIVO_ATLAS_ROS2_KEYFRAME_TRAJECTORY_PUBLISHER_H_
