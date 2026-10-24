#include <gtest/gtest.h>

#include <chrono>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <thread>

#include "livo_atlas/ros2/measurement_conversions.h"
#include "ros2/fixtures.h"
#include "ros2/mapper_node.h"
namespace livo_atlas::ros2 {
TEST(Ros2NodeTest, ActualDdsPublishesLidarPoseCloudAndTransform) {
  auto params = test::Parameters("/tmp/livo_atlas_p4/tests/dds");
  params.emplace_back("common.img_en", 0);
  params.emplace_back("common.imu_topic", "/p4_test/imu");
  params.emplace_back("common.lid_topic", "/p4_test/lidar");
  params.emplace_back("preprocess.point_filter_num", 1);
  auto mapper = std::make_shared<MapperNode>(
      rclcpp::NodeOptions().parameter_overrides(params));
  auto peer = std::make_shared<rclcpp::Node>("p4_dds_peer");
  int poses = 0, clouds = 0, transforms = 0;
  auto odom = peer->create_subscription<nav_msgs::msg::Odometry>(
      "/aft_mapped_to_init", 10,
      [&](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        EXPECT_EQ(message->header.frame_id, "camera_init");
        EXPECT_EQ(message->child_frame_id, "aft_mapped");
        EXPECT_TRUE(std::isfinite(message->pose.pose.position.x));
        ++poses;
      });
  auto cloud = peer->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud_registered", 10,
      [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        EXPECT_EQ(message->header.frame_id, "camera_init");
        EXPECT_FALSE(message->fields.empty());
        ++clouds;
      });
  auto tf = peer->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", 10, [&](tf2_msgs::msg::TFMessage::ConstSharedPtr message) {
        if (!message->transforms.empty()) {
          EXPECT_EQ(message->transforms.front().child_frame_id, "aft_mapped");
          ++transforms;
        }
      });
  auto pub = peer->create_publisher<livox_ros_driver2::msg::CustomMsg>(
      "/p4_test/lidar", rclcpp::SensorDataQoS());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(peer);
  executor.add_node(mapper);
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(4);
  while (pub->get_subscription_count() == 0 &&
         std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(pub->get_subscription_count(), 1u);
  for (int frame = 0; frame < 3; ++frame) {
    livox_ros_driver2::msg::CustomMsg message;
    message.header.stamp = FromSeconds(100 + frame * .25);
    const auto scan = test::Scan(100);
    for (const auto& p : std::get<std::vector<LivoxPoint>>(scan.points)) {
      livox_ros_driver2::msg::CustomPoint q;
      q.x = p.x;
      q.y = p.y;
      q.z = p.z;
      q.line = p.line;
      q.tag = p.tag;
      q.offset_time = p.offset_time;
      message.points.push_back(q);
    }
    message.point_num = message.points.size();
    pub->publish(message);
    for (int i = 0; i < 20; ++i) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  EXPECT_EQ(poses, 3);
  EXPECT_EQ(clouds, 3);
  EXPECT_EQ(transforms, 3);
  testing::internal::CaptureStdout();
  mapper->Finish();
  const auto output = testing::internal::GetCapturedStdout();
  EXPECT_NE(output.find("lidar=3 imu=0 image=0"), std::string::npos);
}
}  // namespace livo_atlas::ros2
