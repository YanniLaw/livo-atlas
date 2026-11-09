#ifndef LIVO_ATLAS_ROS2_LIVO_NODE_H_
#define LIVO_ATLAS_ROS2_LIVO_NODE_H_

#include <cstdint>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/ros2/livo_publisher.h"
#include "livo_atlas/ros2/node_options.h"

namespace livo_atlas::ros2 {

class LivoNode {
 public:
  LivoNode(rclcpp::Node& node, const NodeOptions& parameters,
           LivoSystem& system);
  ~LivoNode();
  LivoNode(const LivoNode&) = delete;
  LivoNode& operator=(const LivoNode&) = delete;
  SyncStatus ProcessNext();
  void ReportInputs();

 private:
  void Warn(const std::string& error);

  std::uint64_t lidar_count_ = 0, imu_count_ = 0, image_count_ = 0;
  rclcpp::Node& node_;
  NodeOptions parameters_;
  LivoSystem& system_;
  LivoPublisher publisher_;
  // Destroy callback sources before the state they access.
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_;
  rclcpp::TimerBase::SharedPtr processing_, propagation_;
};

}  // namespace livo_atlas::ros2

#endif  // LIVO_ATLAS_ROS2_LIVO_NODE_H_
