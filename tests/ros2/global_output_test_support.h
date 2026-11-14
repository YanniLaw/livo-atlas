#ifndef LIVO_ATLAS_TESTS_ROS2_GLOBAL_OUTPUT_TEST_SUPPORT_H_
#define LIVO_ATLAS_TESTS_ROS2_GLOBAL_OUTPUT_TEST_SUPPORT_H_

#include <gtest/gtest.h>
#include <tf2_msgs/msg/tf_message.hpp>

#include <Eigen/Geometry>
#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "livo_atlas/core/global_output_options.h"
#include "livo_atlas/mapping/map_correction.h"
#include "livo_atlas/ros2/livo_publisher.h"

namespace livo_atlas::ros2::test_support {

inline FrontendResult LidarResult() {
  FrontendResult result;
  result.updated = true;
  result.kind = UpdateKind::kLidarInertial;
  result.estimate.pos_end << 4, 5, 6;
  // The old Euler publisher sees identity; projecting this shear does not.
  result.estimate.rot_end(0, 1) = 0.000045;
  return result;
}

inline MapCorrection Correction() {
  MapCorrection correction;
  correction.pose_version = 1;
  correction.frame_count = 3;
  correction.optimized_frame_count = 2;
  correction.map_from_odom.rotation << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  correction.map_from_odom.translation << 1, 2, 3;
  return correction;
}

inline Eigen::Matrix3d Rotation(const geometry_msgs::msg::Quaternion& q) {
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
}

class GlobalOutputTest : public ::testing::Test {
 protected:
  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("global_output_test");
    executor_.add_node(node_);
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/aft_mapped_to_init", 20,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr p) {
          odometry_.push_back(*p);
        });
    tf_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", 20, [this](tf2_msgs::msg::TFMessage::ConstSharedPtr p) {
          transforms_.push_back(*p);
        });
    global_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/global_pose", 20,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr p) {
          global_.push_back(*p);
        });
    image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "/rgb_img", 20,
        [this](sensor_msgs::msg::Image::ConstSharedPtr) { ++images_; });
    propagation_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/LIVO2/imu_propagate", 20,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr) { ++propagations_; });
  }

  bool Wait(const std::function<bool()>& condition) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < end) {
      executor_.spin_some();
      if (condition()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

  void Drain() {
    const auto end =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(150);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < end) {
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  void Open(GlobalOutputOptions options = {true, true, "map"}) {
    publisher_ = std::make_unique<LivoPublisher>(
        *node_, FrontendMode::kLidarInertialVisual, 1, options);
    ASSERT_TRUE(Wait([&] {
      return odom_sub_->get_publisher_count() &&
             tf_sub_->get_publisher_count() &&
             image_sub_->get_publisher_count() &&
             propagation_sub_->get_publisher_count() &&
             (!options.enabled || global_sub_->get_publisher_count());
    }));
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::vector<nav_msgs::msg::Odometry> odometry_;
  std::vector<tf2_msgs::msg::TFMessage> transforms_;
  std::vector<geometry_msgs::msg::PoseStamped> global_;
  int images_ = 0;
  int propagations_ = 0;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_,
      propagation_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr global_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  std::unique_ptr<LivoPublisher> publisher_;
};

}  // namespace livo_atlas::ros2::test_support

#endif  // LIVO_ATLAS_TESTS_ROS2_GLOBAL_OUTPUT_TEST_SUPPORT_H_
