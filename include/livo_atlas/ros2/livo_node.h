#ifndef LIVO_ATLAS_ROS2_LIVO_NODE_H_
#define LIVO_ATLAS_ROS2_LIVO_NODE_H_

#include <chrono>
#include <functional>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/ros2/livo_publisher.h"
#include "livo_atlas/ros2/node_options.h"
#include "livo_atlas/ros_common/sensor_input.h"

namespace livo_atlas::ros2 {

class LivoNode {
 public:
  // The processing group owns the system while the executor is running.
  LivoNode(rclcpp::Node& node, const NodeOptions& parameters,
           LivoSystem& system,
           std::function<void()> poll_background_outputs = {});
  ~LivoNode();
  LivoNode(const LivoNode&) = delete;
  LivoNode& operator=(const LivoNode&) = delete;
  SyncStatus ProcessNext();  // Processing group or stopped executor only.
  void RethrowCallbackFailure();
  // Call once the executor has stopped.
  void DrainInput();
  void ReportInputs();

 private:
  NodeOptions parameters_;
  LivoSystem& system_;
  std::function<void()> poll_background_outputs_;
  LivoPublisher publisher_;
  rclcpp::Context::SharedPtr context_;
  ros_detail::SensorInput input_;
  std::chrono::steady_clock::time_point next_propagation_;
  std::chrono::steady_clock::time_point last_propagation_;
  // Destroy callback sources before the state they access.
  rclcpp::CallbackGroup::SharedPtr lidar_group_;
  rclcpp::CallbackGroup::SharedPtr imu_group_;
  rclcpp::CallbackGroup::SharedPtr image_group_;
  rclcpp::CallbackGroup::SharedPtr processing_group_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_;
  rclcpp::TimerBase::SharedPtr processing_;
};

}  // namespace livo_atlas::ros2

#endif  // LIVO_ATLAS_ROS2_LIVO_NODE_H_
