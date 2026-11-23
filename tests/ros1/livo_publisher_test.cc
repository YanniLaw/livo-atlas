#include "livo_atlas/ros1/livo_publisher.h"

#include "ros1/global_output_test_support.h"
#include <Eigen/SVD>
#include <limits>

#include <geometry_msgs/PoseStamped.h>
#include <gtest/gtest.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <ros/serialization.h>
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

TEST(RosOutputTest, PathKeepsNewestPosesAndBoundedMessageSize) {
  ros::NodeHandle node;
  nav_msgs::Path received;
  auto subscription = node.subscribe<nav_msgs::Path>(
      "/path", 10,
      [&](const nav_msgs::Path::ConstPtr& msg) { received = *msg; });
  image_transport::ImageTransport transport(node);
  ros1::LivoPublisher publisher(FrontendMode::kLidarInertial, 1, {}, 3);
  publisher.Initialize(node, transport);
  ASSERT_TRUE(WaitFor([&] { return subscription.getNumPublishers() != 0; }));
  FrontendResult result;
  result.updated = true;
  result.kind = UpdateKind::kLidarInertial;
  const auto send = [&](int frame) {
    result.estimate.pos_end.x() = frame;
    publisher.Publish(result, nullptr);
  };
  const auto received_frame = [&](int frame) {
    return !received.poses.empty() &&
           received.poses.back().pose.position.x == frame;
  };
  send(1);
  ASSERT_TRUE(WaitFor([&] { return received_frame(1); }));
  const auto path_stamp = received.header.stamp;
  send(2);
  send(3);
  ASSERT_TRUE(WaitFor([&] { return received_frame(3); }));
  const auto third_stamp = received.poses.back().header.stamp;
  send(4);
  send(5);
  ASSERT_TRUE(WaitFor([&] { return received_frame(5); }));
  ASSERT_EQ(received.poses.size(), 3u);
  EXPECT_EQ(received.header.frame_id, "camera_init");
  EXPECT_EQ(received.header.stamp, path_stamp);
  EXPECT_EQ(received.poses.front().header.stamp, third_stamp);
  for (int index = 0; index < 3; ++index) {
    EXPECT_DOUBLE_EQ(received.poses[index].pose.position.x, 3 + index);
    EXPECT_EQ(received.poses[index].header.frame_id, "camera_init");
    EXPECT_GE(received.poses[index].header.stamp, third_stamp);
  }
  const auto bytes = ros::serialization::serializationLength(received);
  for (int frame = 6; frame <= 256; ++frame) send(frame);
  ASSERT_TRUE(WaitFor([&] { return received_frame(256); }));
  ASSERT_EQ(received.poses.size(), 3u);
  EXPECT_DOUBLE_EQ(received.poses.front().pose.position.x, 254);
  EXPECT_EQ(ros::serialization::serializationLength(received), bytes);
}

}  // namespace
}  // namespace livo_atlas

namespace livo_atlas::ros1::test_support {

TEST_F(GlobalOutputTest, GlobalPoseUsesActualLegacyQuaternionAndPairedTf) {
  Open({true, true, "atlas/map_2"});
  const auto result = LidarResult();
  const auto correction = Correction();
  publisher_->Publish(result, nullptr, correction);
  ASSERT_TRUE(Wait([&] {
    return odometry_.size() == 1 && transforms_.size() == 1 &&
           global_.size() == 1;
  }));
  const auto& raw = odometry_[0];
  const auto& pose = global_[0];
  ASSERT_EQ(transforms_[0].transforms.size(), 2);
  const auto& map_tf = transforms_[0].transforms[0];
  const auto& raw_tf = transforms_[0].transforms[1];
  EXPECT_EQ(map_tf.header.frame_id, "atlas/map_2");
  EXPECT_EQ(map_tf.child_frame_id, "camera_init");
  EXPECT_EQ(raw_tf.header.frame_id, "camera_init");
  EXPECT_EQ(raw_tf.child_frame_id, "aft_mapped");
  EXPECT_EQ(map_tf.header.stamp, raw.header.stamp);
  EXPECT_EQ(raw_tf.header.stamp, raw.header.stamp);
  EXPECT_EQ(pose.header.stamp, raw.header.stamp);
  EXPECT_EQ(pose.header.frame_id, "atlas/map_2");
  EXPECT_TRUE(Rotation(raw.pose.pose.orientation).isIdentity(1e-15));
  EXPECT_TRUE(Rotation(raw_tf.transform.rotation).isIdentity(1e-15));
  EXPECT_DOUBLE_EQ(raw_tf.transform.translation.x, 4);
  EXPECT_DOUBLE_EQ(raw.pose.pose.position.y, 5);
  EXPECT_DOUBLE_EQ(raw.pose.pose.position.z, 6);
  EXPECT_TRUE(Rotation(map_tf.transform.rotation)
                  .isApprox(correction.map_from_odom.rotation, 1e-14));
  EXPECT_DOUBLE_EQ(map_tf.transform.translation.x, 1);
  EXPECT_DOUBLE_EQ(map_tf.transform.translation.y, 2);
  EXPECT_DOUBLE_EQ(map_tf.transform.translation.z, 3);
  // Literal C * O_pub; independently check the full TF chain.
  EXPECT_DOUBLE_EQ(pose.pose.position.x, -4);
  EXPECT_DOUBLE_EQ(pose.pose.position.y, 6);
  EXPECT_DOUBLE_EQ(pose.pose.position.z, 9);
  const Eigen::Matrix3d expected_rotation =
      Rotation(map_tf.transform.rotation) * Rotation(raw_tf.transform.rotation);
  EXPECT_TRUE(
      Rotation(pose.pose.orientation).isApprox(expected_rotation, 1e-14));
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      result.estimate.rot_end, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Matrix3d projected = svd.matrixU() * svd.matrixV().transpose();
  EXPECT_GT((Rotation(pose.pose.orientation) -
             correction.map_from_odom.rotation * projected)
                .norm(),
            1e-5);
  EXPECT_DOUBLE_EQ(result.estimate.rot_end(0, 1), 0.000045);
}

TEST_F(GlobalOutputTest, DisabledDoesNotAdvertiseGlobalPose) {
  Open({});
  publisher_->Publish(LidarResult(), nullptr);
  ASSERT_TRUE(
      Wait([&] { return odometry_.size() == 1 && transforms_.size() == 1; }));
  Drain();
  EXPECT_TRUE(global_.empty());
  EXPECT_EQ(global_sub_.getNumPublishers(), 0U);
  ASSERT_EQ(transforms_[0].transforms.size(), 1);
  EXPECT_EQ(transforms_[0].transforms[0].child_frame_id, "aft_mapped");
}

TEST_F(GlobalOutputTest, PoseOnlyModeKeepsSingleTf) {
  Open({true, false, "map"});
  publisher_->Publish(LidarResult(), nullptr, MapCorrection{});
  ASSERT_TRUE(
      Wait([&] { return global_.size() == 1 && transforms_.size() == 1; }));
  ASSERT_EQ(transforms_[0].transforms.size(), 1);
  EXPECT_EQ(transforms_[0].transforms[0].header.frame_id, "camera_init");
  EXPECT_DOUBLE_EQ(global_[0].pose.position.x, 4);
  EXPECT_TRUE(Rotation(global_[0].pose.orientation).isIdentity(1e-15));
}

TEST_F(GlobalOutputTest, MissingOrInvalidCorrectionFailsBeforePublication) {
  Open();
  auto result = LidarResult();
  EXPECT_THROW(publisher_->Publish(result, nullptr), std::invalid_argument);
  auto correction = Correction();
  correction.optimized_frame_count = 0;
  EXPECT_THROW(publisher_->Publish(result, nullptr, correction),
               std::invalid_argument);
  correction = Correction();
  correction.map_from_odom.translation.x() =
      std::numeric_limits<double>::infinity();
  EXPECT_THROW(publisher_->Publish(result, nullptr, correction),
               std::invalid_argument);
  correction = Correction();
  result.estimate.pos_end.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(publisher_->Publish(result, nullptr, correction),
               std::invalid_argument);
  Drain();
  EXPECT_TRUE(global_.empty());
  EXPECT_TRUE(transforms_.empty());
  EXPECT_TRUE(odometry_.empty());
}

TEST_F(GlobalOutputTest, DirectOptionsRejectConflictingFrame) {
  EXPECT_THROW(Open({true, true, "world"}), std::invalid_argument);
}

}  // namespace livo_atlas::ros1::test_support
