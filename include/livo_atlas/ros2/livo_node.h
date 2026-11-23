#ifndef LIVO_ATLAS_ROS2_LIVO_NODE_H_
#define LIVO_ATLAS_ROS2_LIVO_NODE_H_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <functional>
#include <mutex>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/ros2/livo_publisher.h"
#include "livo_atlas/ros2/node_options.h"

namespace livo_atlas::ros2 {

class LivoNode {
 public:
  // Stop the executor before using system directly.
  LivoNode(rclcpp::Node& node, const NodeOptions& parameters,
           LivoSystem& system,
           std::function<void()> poll_background_outputs = {});
  ~LivoNode();
  LivoNode(const LivoNode&) = delete;
  LivoNode& operator=(const LivoNode&) = delete;
  SyncStatus ProcessNext();
  void RethrowCallbackFailure();
  void ReportInputs();

 private:
  template <typename Callback>
  void RunCallback(Callback callback);

  NodeOptions parameters_;
  LivoSystem& system_;
  std::function<void()> poll_background_outputs_;
  LivoPublisher publisher_;
  rclcpp::Context::SharedPtr context_;
  // Serialize system access and background output polling across callbacks.
  std::mutex system_mutex_;
  std::exception_ptr callback_failure_;
  std::chrono::steady_clock::time_point last_propagation_;
  std::atomic<std::uint64_t> lidar_count_{0};
  std::atomic<std::uint64_t> imu_count_{0};
  std::atomic<std::uint64_t> image_count_{0};
  // Destroy callback sources before the state they access.
  rclcpp::CallbackGroup::SharedPtr lidar_group_;
  rclcpp::CallbackGroup::SharedPtr imu_group_;
  rclcpp::CallbackGroup::SharedPtr image_group_;
  rclcpp::CallbackGroup::SharedPtr processing_group_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_;
  rclcpp::TimerBase::SharedPtr processing_, propagation_;
};

}  // namespace livo_atlas::ros2

#endif  // LIVO_ATLAS_ROS2_LIVO_NODE_H_
