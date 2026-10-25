#include "livo_atlas/ros2/livo_publisher.h"

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "livo_atlas/common/so3_math.h"
#include "livo_atlas/ros2/measurement_conversions.h"

namespace livo_atlas::ros2 {
namespace {

Eigen::Vector3d PlaneColor(const Eigen::Vector3d& variance) {
  double value = std::min(variance.sum(), 0.25);
  value = std::pow(value * (1.0 / 0.25), 0.2);
  double red, green, blue;
  if (value < 0.1242) {
    blue = 0.504 + ((1.0 - 0.504) / 0.1242) * value;
    green = red = 0;
  } else if (value < 0.3747) {
    blue = 1;
    red = 0;
    green = (value - 0.1242) * (1.0 / (0.3747 - 0.1242));
  } else if (value < 0.6253) {
    blue = (0.6253 - value) * (1.0 / (0.6253 - 0.3747));
    green = 1;
    red = (value - 0.3747) * (1.0 / (0.6253 - 0.3747));
  } else if (value < 0.8758) {
    blue = 0;
    red = 1;
    green = (0.8758 - value) * (1.0 / (0.8758 - 0.6253));
  } else {
    blue = green = 0;
    red = 1.0 - (value - 0.8758) * ((1.0 - 0.504) / (1.0 - 0.8758));
  }
  return {static_cast<std::uint8_t>(255 * red) / 256.0,
          static_cast<std::uint8_t>(255 * green) / 256.0,
          static_cast<std::uint8_t>(255 * blue) / 256.0};
}

visualization_msgs::msg::MarkerArray PlaneMarkers(
    const std::vector<PlaneVisualization>& planes) {
  visualization_msgs::msg::MarkerArray markers;
  markers.markers.reserve(planes.size());
  for (const auto& plane : planes) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "camera_init";
    marker.ns = "plane";
    marker.id = plane.id;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = plane.center.x();
    marker.pose.position.y = plane.center.y();
    marker.pose.position.z = plane.center.z();
    Eigen::Matrix3d rotation;
    rotation.col(0) = plane.x_axis;
    rotation.col(1) = plane.y_axis;
    rotation.col(2) = plane.normal;
    const Eigen::Quaterniond orientation(rotation);
    marker.pose.orientation.w = orientation.w();
    marker.pose.orientation.x = orientation.x();
    marker.pose.orientation.y = orientation.y();
    marker.pose.orientation.z = orientation.z();
    marker.scale.x = 3 * std::sqrt(plane.eigenvalues.z());
    marker.scale.y = 3 * std::sqrt(plane.eigenvalues.y());
    marker.scale.z = 2 * std::sqrt(plane.eigenvalues.x());
    const auto color = PlaneColor(plane.variance_diagonal);
    marker.color.a = plane.valid ? 0.8F : 0.0F;
    marker.color.r = color.x();
    marker.color.g = color.y();
    marker.color.b = color.z();
    markers.markers.push_back(marker);
  }
  return markers;
}

geometry_msgs::msg::Pose Pose(
    const FilterState& state,
    const geometry_msgs::msg::Quaternion& orientation) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = state.pos_end.x();
  pose.position.y = state.pos_end.y();
  pose.position.z = state.pos_end.z();
  pose.orientation = orientation;
  return pose;
}

}  // namespace

class LivoPublisher::Impl {
 public:
  Impl(rclcpp::Node& node, FrontendMode mode)
      : node_(node), mode_(mode), broadcaster_(node) {
    cloud_ = node.create_publisher<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered", 100);
    effect_ = node.create_publisher<sensor_msgs::msg::PointCloud2>(
        "/cloud_effected", 100);
    odometry_ = node.create_publisher<nav_msgs::msg::Odometry>(
        "/aft_mapped_to_init", 10);
    path_publisher_ = node.create_publisher<nav_msgs::msg::Path>("/path", 10);
    mavros_ = node.create_publisher<geometry_msgs::msg::PoseStamped>(
        "/mavros/vision_pose/pose", 10);
    propagation_ = node.create_publisher<nav_msgs::msg::Odometry>(
        "/LIVO2/imu_propagate", 10000);
    planes_ = node.create_publisher<visualization_msgs::msg::MarkerArray>(
        "/planes", 10000);
    image_ = node.create_publisher<sensor_msgs::msg::Image>("/rgb_img", 1);
    path_.header.stamp = node_.now();
    path_.header.frame_id = "camera_init";
  }

  void PublishCloud(const FrontendResult& result,
                    const PreparedOutput& prepared) {
    if (!prepared.has_cloud) return;
    sensor_msgs::msg::PointCloud2 message;
    if (mode_ == FrontendMode::kLidarInertialVisual) {
      if (result.kind == UpdateKind::kVisual) {
        pcl::toROSMsg(*prepared.colored_cloud, message);
      }
    } else {
      pcl::toROSMsg(*result.world_cloud, message);
    }
    message.header.stamp = node_.now();
    message.header.frame_id = "camera_init";
    cloud_->publish(message);
  }

  void Publish(const FrontendResult& result, const PreparedOutput& prepared) {
    if (!result.updated) return;
    if (result.kind == UpdateKind::kVisual) {
      PublishCloud(result, prepared);
      cv_bridge::CvImage image;
      image.header.stamp = node_.now();
      image.encoding = sensor_msgs::image_encodings::BGR8;
      image.image = result.tracking_image;
      image_->publish(*image.toImageMsg());
      return;
    }
    const Eigen::Vector3d euler =
        RotationMatrixToEuler(result.estimate.rot_end);
    tf2::Quaternion quaternion;
    quaternion.setRPY(euler.x(), euler.y(), euler.z());
    geometry_msgs::msg::Quaternion orientation;
    orientation.x = quaternion.x();
    orientation.y = quaternion.y();
    orientation.z = quaternion.z();
    orientation.w = quaternion.w();
    nav_msgs::msg::Odometry odometry;
    odometry.header.frame_id = "camera_init";
    odometry.child_frame_id = "aft_mapped";
    odometry.header.stamp = node_.now();
    odometry.pose.pose = Pose(result.estimate, orientation);
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odometry.header;
    transform.child_frame_id = odometry.child_frame_id;
    transform.transform.translation.x = result.estimate.pos_end.x();
    transform.transform.translation.y = result.estimate.pos_end.y();
    transform.transform.translation.z = result.estimate.pos_end.z();
    transform.transform.rotation = orientation;
    broadcaster_.sendTransform(transform);
    odometry_->publish(odometry);
    PublishCloud(result, prepared);
    if (result.effective_cloud) {
      sensor_msgs::msg::PointCloud2 message;
      pcl::toROSMsg(*result.effective_cloud, message);
      message.header.stamp = node_.now();
      message.header.frame_id = "camera_init";
      effect_->publish(message);
    }
    if (result.has_plane_visualization) {
      planes_->publish(PlaneMarkers(result.planes));
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.pose = odometry.pose.pose;
    pose.header.stamp = node_.now();
    pose.header.frame_id = "camera_init";
    path_.poses.push_back(pose);
    path_publisher_->publish(path_);
    pose.header.stamp = node_.now();
    mavros_->publish(pose);
  }

  void PublishImuPropagation(const FilterState& state, double timestamp) {
    nav_msgs::msg::Odometry message;
    message.header.frame_id = "world";
    message.header.stamp = FromSeconds(timestamp);
    geometry_msgs::msg::Quaternion orientation;
    const Eigen::Quaterniond quaternion(state.rot_end);
    orientation.w = quaternion.w();
    orientation.x = quaternion.x();
    orientation.y = quaternion.y();
    orientation.z = quaternion.z();
    message.pose.pose = Pose(state, orientation);
    message.twist.twist.linear.x = state.vel_end.x();
    message.twist.twist.linear.y = state.vel_end.y();
    message.twist.twist.linear.z = state.vel_end.z();
    propagation_->publish(message);
  }

  rclcpp::Node& node_;
  FrontendMode mode_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_, effect_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_, propagation_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mavros_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr planes_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_;
  tf2_ros::TransformBroadcaster broadcaster_;
  nav_msgs::msg::Path path_;
};

LivoPublisher::LivoPublisher(rclcpp::Node& node, FrontendMode mode)
    : impl_(std::make_unique<Impl>(node, mode)) {}
LivoPublisher::~LivoPublisher() = default;
void LivoPublisher::Publish(const FrontendResult& result,
                            const PreparedOutput& prepared) {
  impl_->Publish(result, prepared);
}
void LivoPublisher::PublishImuPropagation(const FilterState& state,
                                          double timestamp) {
  impl_->PublishImuPropagation(state, timestamp);
}
}  // namespace livo_atlas::ros2
