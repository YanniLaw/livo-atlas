#include "livo_atlas/ros1/result_publisher.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>

#include "livo_atlas/core/so3_math.h"

namespace livo_atlas {
namespace ros1 {
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

visualization_msgs::MarkerArray PlaneMarkers(
    const std::vector<PlaneVisualization>& planes) {
  visualization_msgs::MarkerArray markers;
  markers.markers.reserve(planes.size());
  for (const auto& plane : planes) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_init";
    marker.ns = "plane";
    marker.id = plane.id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
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

geometry_msgs::Pose Pose(const State& state,
                         const geometry_msgs::Quaternion& orientation) {
  geometry_msgs::Pose pose;
  pose.position.x = state.pos_end.x();
  pose.position.y = state.pos_end.y();
  pose.position.z = state.pos_end.z();
  pose.orientation = orientation;
  return pose;
}

}  // namespace

class ResultPublisher::Impl {
 public:
  explicit Impl(FrontendMode mode) : mode_(mode) {}

  void Initialize(ros::NodeHandle& node,
                  image_transport::ImageTransport& transport) {
    cloud_ = node.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);
    effect_ = node.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100);
    odometry_ = node.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
    path_publisher_ = node.advertise<nav_msgs::Path>("/path", 10);
    mavros_ = node.advertise<geometry_msgs::PoseStamped>(
        "/mavros/vision_pose/pose", 10);
    propagation_ = node.advertise<nav_msgs::Odometry>(
        "/LIVO2/imu_propagate", 10000);
    planes_ = node.advertise<visualization_msgs::MarkerArray>("/planes", 10000);
    image_ = transport.advertise("/rgb_img", 1);
    compatibility_ = {
        node.advertise<visualization_msgs::MarkerArray>("visualization_marker",
                                                       100),
        node.advertise<sensor_msgs::PointCloud2>("/cloud_visual_sub_map_before",
                                               100),
        node.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100),
        node.advertise<visualization_msgs::Marker>("/planner_normal", 1),
        node.advertise<visualization_msgs::MarkerArray>("/voxels", 1),
        node.advertise<sensor_msgs::PointCloud2>("/dyn_obj", 100),
        node.advertise<sensor_msgs::PointCloud2>("/dyn_obj_removed", 100),
        node.advertise<sensor_msgs::PointCloud2>("/dyn_obj_dbg_hist", 100)};
    path_.header.stamp = ros::Time::now();
    path_.header.frame_id = "camera_init";
  }

  void PublishCloud(const FrontendResult& result,
                     const PreparedOutput& prepared) {
    if (!prepared.has_cloud) return;
    sensor_msgs::PointCloud2 message;
    if (mode_ == FrontendMode::kLidarInertialVisual) {
      if (result.kind == UpdateKind::kVisual) {
        pcl::toROSMsg(*prepared.colored_cloud, message);
      }
    } else {
      pcl::toROSMsg(*result.world_cloud, message);
    }
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "camera_init";
    cloud_.publish(message);
  }

  void Publish(const FrontendResult& result, const PreparedOutput& prepared) {
    if (!result.updated) return;
    if (result.kind == UpdateKind::kVisual) {
      PublishCloud(result, prepared);
      cv_bridge::CvImage image;
      image.header.stamp = ros::Time::now();
      image.encoding = sensor_msgs::image_encodings::BGR8;
      image.image = result.tracking_image;
      image_.publish(image.toImageMsg());
      return;
    }
    const Eigen::Vector3d euler =
        RotationMatrixToEuler(result.estimate.rot_end);
    const auto orientation = tf::createQuaternionMsgFromRollPitchYaw(
        euler.x(), euler.y(), euler.z());
    nav_msgs::Odometry odometry;
    odometry.header.frame_id = "camera_init";
    odometry.child_frame_id = "aft_mapped";
    odometry.header.stamp = ros::Time::now();
    odometry.pose.pose = Pose(result.estimate, orientation);
    tf::Transform transform;
    const auto& position = result.estimate.pos_end;
    transform.setOrigin(tf::Vector3(position.x(), position.y(), position.z()));
    transform.setRotation(tf::Quaternion(orientation.x, orientation.y,
                                         orientation.z, orientation.w));
    broadcaster_.sendTransform(tf::StampedTransform(
        transform, odometry.header.stamp, "camera_init", "aft_mapped"));
    odometry_.publish(odometry);
    PublishCloud(result, prepared);
    if (result.effective_cloud) {
      sensor_msgs::PointCloud2 message;
      pcl::toROSMsg(*result.effective_cloud, message);
      message.header.stamp = ros::Time::now();
      message.header.frame_id = "camera_init";
      effect_.publish(message);
    }
    if (result.has_plane_visualization) {
      planes_.publish(PlaneMarkers(result.planes));
    }
    geometry_msgs::PoseStamped pose;
    pose.pose = odometry.pose.pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "camera_init";
    path_.poses.push_back(pose);
    path_publisher_.publish(path_);
    pose.header.stamp = ros::Time::now();
    mavros_.publish(pose);
  }

  void PublishImuPropagation(const State& state, const ros::Time& timestamp) {
    nav_msgs::Odometry message;
    message.header.frame_id = "world";
    message.header.stamp = timestamp;
    geometry_msgs::Quaternion orientation;
    const Eigen::Quaterniond quaternion(state.rot_end);
    orientation.w = quaternion.w();
    orientation.x = quaternion.x();
    orientation.y = quaternion.y();
    orientation.z = quaternion.z();
    message.pose.pose = Pose(state, orientation);
    message.twist.twist.linear.x = state.vel_end.x();
    message.twist.twist.linear.y = state.vel_end.y();
    message.twist.twist.linear.z = state.vel_end.z();
    propagation_.publish(message);
  }

  FrontendMode mode_;
  ros::Publisher cloud_, effect_, odometry_, path_publisher_, mavros_;
  ros::Publisher propagation_, planes_;
  image_transport::Publisher image_;
  std::vector<ros::Publisher> compatibility_;
  tf::TransformBroadcaster broadcaster_;
  nav_msgs::Path path_;
};

ResultPublisher::ResultPublisher(FrontendMode mode)
    : impl_(std::make_unique<Impl>(mode)) {}
ResultPublisher::~ResultPublisher() = default;
void ResultPublisher::Initialize(ros::NodeHandle& node,
                                 image_transport::ImageTransport& transport) {
  impl_->Initialize(node, transport);
}
void ResultPublisher::Publish(const FrontendResult& result,
                              const PreparedOutput& prepared) {
  impl_->Publish(result, prepared);
}
void ResultPublisher::PublishImuPropagation(const State& state,
                                            const ros::Time& timestamp) {
  impl_->PublishImuPropagation(state, timestamp);
}

}  // namespace ros1
}  // namespace livo_atlas
