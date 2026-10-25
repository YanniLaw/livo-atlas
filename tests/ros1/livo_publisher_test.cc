#include "livo_atlas/ros1/livo_publisher.h"

#include <geometry_msgs/PoseStamped.h>
#include <gtest/gtest.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <functional>
#include <opencv2/core.hpp>
#include <vector>

namespace livo_atlas {
namespace {

bool WaitFor(const std::function<bool()>& condition) {
  const auto deadline = ros::WallTime::now() + ros::WallDuration(3);
  while (ros::ok() && ros::WallTime::now() < deadline) {
    ros::spinOnce();
    if (condition()) return true;
    ros::WallDuration(0.01).sleep();
  }
  return false;
}
TEST(RosOutputTest, PublishesOrdinarySnapshotsWithLegacySchemasAndPoses) {
  ros::NodeHandle node;
  std::vector<sensor_msgs::PointCloud2> clouds;
  std::vector<sensor_msgs::Image> images;
  std::vector<nav_msgs::Odometry> odometry;
  std::vector<nav_msgs::Odometry> propagation;
  std::vector<nav_msgs::Path> paths;
  std::vector<geometry_msgs::PoseStamped> poses;
  std::vector<visualization_msgs::MarkerArray> planes;
  auto cloud_sub = node.subscribe<sensor_msgs::PointCloud2>(
      "/cloud_registered", 10,
      [&](const sensor_msgs::PointCloud2::ConstPtr& msg) {
        clouds.push_back(*msg);
      });
  auto image_sub = node.subscribe<sensor_msgs::Image>(
      "/rgb_img", 10,
      [&](const sensor_msgs::Image::ConstPtr& msg) { images.push_back(*msg); });
  auto odom_sub = node.subscribe<nav_msgs::Odometry>(
      "/aft_mapped_to_init", 10, [&](const nav_msgs::Odometry::ConstPtr& msg) {
        odometry.push_back(*msg);
      });
  auto imu_sub = node.subscribe<nav_msgs::Odometry>(
      "/LIVO2/imu_propagate", 10, [&](const nav_msgs::Odometry::ConstPtr& msg) {
        propagation.push_back(*msg);
      });
  auto path_sub = node.subscribe<nav_msgs::Path>(
      "/path", 10,
      [&](const nav_msgs::Path::ConstPtr& msg) { paths.push_back(*msg); });
  auto pose_sub = node.subscribe<geometry_msgs::PoseStamped>(
      "/mavros/vision_pose/pose", 10,
      [&](const geometry_msgs::PoseStamped::ConstPtr& msg) {
        poses.push_back(*msg);
      });
  auto plane_sub = node.subscribe<visualization_msgs::MarkerArray>(
      "/planes", 10, [&](const visualization_msgs::MarkerArray::ConstPtr& msg) {
        planes.push_back(*msg);
      });
  image_transport::ImageTransport transport(node);
  ros1::LivoPublisher publisher(FrontendMode::kLidarInertialVisual);
  publisher.Initialize(node, transport);
  ASSERT_TRUE(WaitFor([&] {
    return cloud_sub.getNumPublishers() && image_sub.getNumPublishers() &&
           odom_sub.getNumPublishers() && imu_sub.getNumPublishers() &&
           path_sub.getNumPublishers() && pose_sub.getNumPublishers() &&
           plane_sub.getNumPublishers();
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
  ColoredCloud::Ptr colored(new ColoredCloud);
  pcl::PointXYZRGB point(11, 22, 33);
  point.x = 1;
  point.y = 2;
  point.z = 3;
  colored->push_back(point);
  publisher.Publish(result, colored);
  result.kind = UpdateKind::kVisual;
  result.tracking_image = cv::Mat(2, 3, CV_8UC3, cv::Scalar(44, 55, 66));
  publisher.Publish(result, colored);
  publisher.PublishImuPropagation(result.estimate, ros::Time(123, 456));
  ASSERT_TRUE(WaitFor([&] {
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
  EXPECT_EQ(propagation[0].header.stamp, ros::Time(123, 456));
  EXPECT_DOUBLE_EQ(propagation[0].twist.twist.linear.x, 4);
  ASSERT_EQ(planes[0].markers.size(), 1);
  const auto& marker = planes[0].markers[0];
  EXPECT_EQ(marker.id, 7);
  EXPECT_EQ(marker.type, visualization_msgs::Marker::CYLINDER);
  EXPECT_TRUE(marker.header.stamp.isZero());
  EXPECT_DOUBLE_EQ(marker.pose.position.z, 9);
  EXPECT_NEAR(marker.pose.orientation.z, std::sqrt(0.5), 1e-15);
  EXPECT_NEAR(marker.scale.x, 0.9, 1e-15);
  EXPECT_NEAR(marker.scale.y, 0.6, 1e-15);
  EXPECT_NEAR(marker.scale.z, 0.2, 1e-15);
  EXPECT_FLOAT_EQ(marker.color.a, 0.8);
  EXPECT_FLOAT_EQ(marker.color.b, 0.5);
}

}  // namespace
}  // namespace livo_atlas
