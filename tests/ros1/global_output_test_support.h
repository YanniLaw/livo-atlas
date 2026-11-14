#ifndef LIVO_ATLAS_TESTS_ROS1_GLOBAL_OUTPUT_TEST_SUPPORT_H_
#define LIVO_ATLAS_TESTS_ROS1_GLOBAL_OUTPUT_TEST_SUPPORT_H_

#include <geometry_msgs/PoseStamped.h>
#include <gtest/gtest.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <tf2_msgs/TFMessage.h>

#include <Eigen/Geometry>
#include <functional>
#include <memory>
#include <vector>

#include "livo_atlas/core/global_output_options.h"
#include "livo_atlas/mapping/map_correction.h"
#include "livo_atlas/ros1/livo_publisher.h"

namespace livo_atlas::ros1::test_support {

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

inline Eigen::Matrix3d Rotation(const geometry_msgs::Quaternion& q) {
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
}

class GlobalOutputTest : public ::testing::Test {
 protected:
  void SetUp() override {
    odom_sub_ = node_.subscribe<nav_msgs::Odometry>(
        "/aft_mapped_to_init", 20,
        [this](const nav_msgs::Odometry::ConstPtr& p) {
          odometry_.push_back(*p);
        });
    tf_sub_ = node_.subscribe<tf2_msgs::TFMessage>(
        "/tf", 20, [this](const tf2_msgs::TFMessage::ConstPtr& p) {
          transforms_.push_back(*p);
        });
    global_sub_ = node_.subscribe<geometry_msgs::PoseStamped>(
        "/global_pose", 20,
        [this](const geometry_msgs::PoseStamped::ConstPtr& p) {
          global_.push_back(*p);
        });
    image_sub_ = node_.subscribe<sensor_msgs::Image>(
        "/rgb_img", 20,
        [this](const sensor_msgs::Image::ConstPtr&) { ++images_; });
    propagation_sub_ = node_.subscribe<nav_msgs::Odometry>(
        "/LIVO2/imu_propagate", 20,
        [this](const nav_msgs::Odometry::ConstPtr&) { ++propagations_; });
  }

  bool Wait(const std::function<bool()>& condition) {
    const auto end = ros::WallTime::now() + ros::WallDuration(3);
    while (ros::ok() && ros::WallTime::now() < end) {
      ros::spinOnce();
      if (condition()) return true;
      ros::WallDuration(0.01).sleep();
    }
    return false;
  }

  void Drain() {
    const auto end = ros::WallTime::now() + ros::WallDuration(0.15);
    while (ros::ok() && ros::WallTime::now() < end) {
      ros::spinOnce();
      ros::WallDuration(0.005).sleep();
    }
  }

  void Open(GlobalOutputOptions options = {true, true, "map"}) {
    publisher_ = std::make_unique<LivoPublisher>(
        FrontendMode::kLidarInertialVisual, 1, options);
    publisher_->Initialize(node_, transport_);
    ASSERT_TRUE(Wait([&] {
      return odom_sub_.getNumPublishers() && tf_sub_.getNumPublishers() &&
             image_sub_.getNumPublishers() &&
             propagation_sub_.getNumPublishers() &&
             (!options.enabled || global_sub_.getNumPublishers());
    }));
  }

  ros::NodeHandle node_;
  image_transport::ImageTransport transport_{node_};
  std::vector<nav_msgs::Odometry> odometry_;
  std::vector<tf2_msgs::TFMessage> transforms_;
  std::vector<geometry_msgs::PoseStamped> global_;
  int images_ = 0;
  int propagations_ = 0;
  ros::Subscriber odom_sub_, tf_sub_, global_sub_, image_sub_, propagation_sub_;
  std::unique_ptr<LivoPublisher> publisher_;
};

}  // namespace livo_atlas::ros1::test_support

#endif  // LIVO_ATLAS_TESTS_ROS1_GLOBAL_OUTPUT_TEST_SUPPORT_H_
