#include "livo_atlas/ros2/keyframe_trajectory_publisher.h"

#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

#include <cstdint>
#include <stdexcept>
#include <utility>

#include "livo_atlas/core/global_output_options.h"
#include "ros/trajectory_messages.h"

namespace livo_atlas::ros2 {

KeyframeTrajectoryPublisher::KeyframeTrajectoryPublisher(rclcpp::Node& node,
                                                         std::string map_frame)
    : map_frame_(std::move(map_frame)),
      context_(node.get_node_base_interface()->get_context()) {
  GlobalOutputOptions options;
  options.enabled = true;
  options.map_frame = map_frame_;
  ValidateGlobalOutputOptions(options);
  const auto qos = rclcpp::QoS(1).reliable().transient_local();
  trajectory_ = node.create_publisher<livo_atlas::msg::KeyframeTrajectory>(
      "/keyframe_trajectory", qos);
  path_ = node.create_publisher<nav_msgs::msg::Path>("/keyframe_path", qos);
}

void KeyframeTrajectoryPublisher::Publish(
    const TrajectorySnapshot& snapshot) const {
  if (!rclcpp::ok(context_)) return;
  auto [message, path] =
      ros_detail::MakeTrajectoryMessages<livo_atlas::msg::KeyframeTrajectory,
                                         nav_msgs::msg::Path>(
          snapshot, map_frame_, [](std::int64_t ns) {
            builtin_interfaces::msg::Time stamp;
            stamp.sec = static_cast<std::int32_t>(ns / 1000000000LL);
            stamp.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);
            return stamp;
          });
  trajectory_->publish(message);
  path_->publish(path);
  if (rclcpp::ok(context_) && snapshot.status == TrajectoryStatus::kReady &&
      message.status == livo_atlas::msg::KeyframeTrajectory::ERROR)
    throw std::invalid_argument(message.error);
}

}  // namespace livo_atlas::ros2
