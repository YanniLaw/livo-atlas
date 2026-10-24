#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <thread>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "livo_atlas/ros2/measurement_conversions.h"
#include "livo_atlas/ros2/result_publisher.h"
namespace livo_atlas::ros2 {
namespace {

bool WaitFor(rclcpp::Executor& executor,
             const std::function<bool()>& condition) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (condition()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}
TEST(RosOutputTest, PublishesOrdinarySnapshotsWithLegacySchemasAndPoses) {
  auto node = std::make_shared<rclcpp::Node>("p4_publisher_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::vector<sensor_msgs::msg::PointCloud2> clouds;
  std::vector<sensor_msgs::msg::Image> images;
  std::vector<nav_msgs::msg::Odometry> odometry;
  std::vector<nav_msgs::msg::Odometry> propagation;
  std::vector<nav_msgs::msg::Path> paths;
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  std::vector<visualization_msgs::msg::MarkerArray> planes;
  auto cloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud_registered", 10,
      [&](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
        clouds.push_back(*msg);
      });
  auto image_sub = node->create_subscription<sensor_msgs::msg::Image>(
      "/rgb_img", 10, [&](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        images.push_back(*msg);
      });
  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
      "/aft_mapped_to_init", 10,
      [&](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
        odometry.push_back(*msg);
      });
  auto imu_sub = node->create_subscription<nav_msgs::msg::Odometry>(
      "/LIVO2/imu_propagate", 10,
      [&](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
        propagation.push_back(*msg);
      });
  auto path_sub = node->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, [&](const nav_msgs::msg::Path::ConstSharedPtr& msg) {
        paths.push_back(*msg);
      });
  auto pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/vision_pose/pose", 10,
      [&](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
        poses.push_back(*msg);
      });
  auto plane_sub =
      node->create_subscription<visualization_msgs::msg::MarkerArray>(
          "/planes", 10,
          [&](const visualization_msgs::msg::MarkerArray::ConstSharedPtr& msg) {
            planes.push_back(*msg);
          });
  ResultPublisher publisher(*node, FrontendMode::kLidarInertialVisual);
  ASSERT_TRUE(WaitFor(executor, [&] {
    return cloud_sub->get_publisher_count() &&
           image_sub->get_publisher_count() &&
           odom_sub->get_publisher_count() && imu_sub->get_publisher_count() &&
           path_sub->get_publisher_count() && pose_sub->get_publisher_count() &&
           plane_sub->get_publisher_count();
  }));
  FrontendResult result;
  result.updated = true;
  result.kind = UpdateKind::kLidarInertial;
  result.estimate.pos_end << 1, 2, 3;
  result.estimate.vel_end << 4, 5, 6;
  result.estimate.rot_end << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  result.has_plane_visualization = true;
  PlaneVisualization plane;
  plane.id = 7;
  plane.center << 7, 8, 9;
  plane.x_axis << 0, 1, 0;
  plane.y_axis << -1, 0, 0;
  plane.eigenvalues << 0.01, 0.04, 0.09;
  plane.valid = true;
  result.planes.push_back(plane);
  PreparedOutput prepared;
  prepared.has_cloud = true;
  ColoredCloud::Ptr colored(new ColoredCloud);
  pcl::PointXYZRGB point(std::uint8_t{11}, std::uint8_t{22}, std::uint8_t{33});
  point.x = 1;
  point.y = 2;
  point.z = 3;
  colored->push_back(point);
  prepared.colored_cloud = colored;
  publisher.Publish(result, prepared);
  result.kind = UpdateKind::kVisual;
  result.tracking_image = cv::Mat(2, 3, CV_8UC3, cv::Scalar(44, 55, 66));
  publisher.Publish(result, prepared);
  publisher.PublishImuPropagation(result.estimate, 123.000000456);
  ASSERT_TRUE(WaitFor(executor, [&] {
    return clouds.size() == 2 && images.size() == 1 && odometry.size() == 1 &&
           paths.size() == 1 && poses.size() == 1 && planes.size() == 1 &&
           propagation.size() == 1;
  }));
  EXPECT_TRUE(clouds[0].fields.empty());
  EXPECT_TRUE(clouds[0].data.empty());
  EXPECT_EQ(clouds[1].header.frame_id, "camera_init");
  ColoredCloud decoded;
  pcl::fromROSMsg(clouds[1], decoded);
  ASSERT_EQ(decoded.size(), 1);
  EXPECT_EQ(decoded[0].r, 11);
  EXPECT_FLOAT_EQ(decoded[0].z, 3);
  EXPECT_EQ(images[0].encoding, "bgr8");
  EXPECT_EQ(images[0].data[0], 44);
  EXPECT_EQ(odometry[0].child_frame_id, "aft_mapped");
  EXPECT_DOUBLE_EQ(odometry[0].pose.pose.position.y, 2);
  EXPECT_NEAR(odometry[0].pose.pose.orientation.z, std::sqrt(0.5), 1e-15);
  ASSERT_EQ(paths[0].poses.size(), 1);
  EXPECT_DOUBLE_EQ(paths[0].poses[0].pose.position.z, 3);
  EXPECT_DOUBLE_EQ(poses[0].pose.position.x, 1);
  EXPECT_EQ(propagation[0].header.frame_id, "world");
  EXPECT_DOUBLE_EQ(ToSeconds(propagation[0].header.stamp), 123.000000456);
  EXPECT_DOUBLE_EQ(propagation[0].twist.twist.linear.x, 4);
  ASSERT_EQ(planes[0].markers.size(), 1);
  const auto& marker = planes[0].markers[0];
  EXPECT_EQ(marker.id, 7);
  EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::CYLINDER);
  EXPECT_EQ(marker.header.stamp.sec, 0);
  EXPECT_EQ(marker.header.stamp.nanosec, 0u);
  EXPECT_DOUBLE_EQ(marker.pose.position.z, 9);
  EXPECT_NEAR(marker.pose.orientation.z, std::sqrt(0.5), 1e-15);
  EXPECT_NEAR(marker.scale.x, 0.9, 1e-15);
  EXPECT_NEAR(marker.scale.y, 0.6, 1e-15);
  EXPECT_NEAR(marker.scale.z, 0.2, 1e-15);
  EXPECT_FLOAT_EQ(marker.color.a, 0.8);
  EXPECT_FLOAT_EQ(marker.color.b, 0.5);
}

}  // namespace
}  // namespace livo_atlas::ros2
