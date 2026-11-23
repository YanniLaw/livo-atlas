#include "livo_atlas/ros2/livo_publisher.h"

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <stdexcept>
#include <utility>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "livo_atlas/common/so3_math.h"
#include "ros_common/publication_timing.h"
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

LivoPublisher::LivoPublisher(rclcpp::Node& node, FrontendMode mode,
                             int image_queue_size,
                             GlobalOutputOptions global_output,
                             int path_max_poses, bool stage_timing)
    : node_(node),
      mode_(mode),
      path_max_poses_(path_max_poses),
      stage_timing_(stage_timing),
      global_output_(std::move(global_output)),
      broadcaster_(node) {
  ValidateGlobalOutputOptions(global_output_);
  if (image_queue_size <= 0)
    throw std::invalid_argument("Image queue size must be positive");
  if (path_max_poses <= 0) {
    throw std::invalid_argument("Path pose limit must be positive");
  }
  if (global_output_.enabled)
    global_pose_ = node.create_publisher<geometry_msgs::msg::PoseStamped>(
        "/global_pose", rclcpp::QoS(1).reliable().durability_volatile());
  cloud_ = node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cloud_registered", 100);
  effect_ = node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cloud_effected", 100);
  odometry_ =
      node.create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", 10);
  path_publisher_ = node.create_publisher<nav_msgs::msg::Path>("/path", 10);
  mavros_ = node.create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/vision_pose/pose", 10);
  propagation_ = node.create_publisher<nav_msgs::msg::Odometry>(
      "/LIVO2/imu_propagate", 10000);
  planes_ = node.create_publisher<visualization_msgs::msg::MarkerArray>(
      "/planes", 10000);
  image_ = node.create_publisher<sensor_msgs::msg::Image>("/rgb_img",
                                                          image_queue_size);
  path_.header.stamp = node_.now();
  path_.header.frame_id = "camera_init";
}

void LivoPublisher::PublishCloud(const FrontendResult& result,
                                 const ColoredCloud::ConstPtr& colored_cloud,
                                 ros_detail::PublicationTiming& timing) {
  if (!colored_cloud) return;
  sensor_msgs::msg::PointCloud2 message;
  if (mode_ == FrontendMode::kLidarInertialVisual) {
    if (result.kind == UpdateKind::kVisual) {
      pcl::toROSMsg(*colored_cloud, message);
    }
  } else {
    pcl::toROSMsg(*result.world_cloud, message);
  }
  message.header.stamp = node_.now();
  message.header.frame_id = "camera_init";
  timing.Send([&] { cloud_->publish(message); });
}

void LivoPublisher::Publish(const FrontendResult& result,
                            const ColoredCloud::ConstPtr& colored_cloud,
                            const std::optional<MapCorrection>& correction) {
  if (!result.updated) return;
  ros_detail::PublicationTiming timing(stage_timing_, result.kind);
  if (result.kind == UpdateKind::kVisual) {
    PublishCloud(result, colored_cloud, timing);
    cv_bridge::CvImage image;
    image.header.stamp = node_.now();
    image.encoding = sensor_msgs::image_encodings::BGR8;
    image.image = result.tracking_image;
    const auto message = image.toImageMsg();
    timing.Send([&] { image_->publish(*message); });
    timing.Report();
    return;
  }
  const Eigen::Vector3d euler = RotationMatrixToEuler(result.estimate.rot_end);
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
  geometry_msgs::msg::PoseStamped global_pose;
  const std::int64_t stamp_ns =
      rclcpp::Time(odometry.header.stamp).nanoseconds();
  // Preserve raw output during clock repeats/rollbacks; gate only new output.
  const bool publish_global = global_output_.enabled && stamp_ns > 0 &&
                              stamp_ns > last_global_stamp_ns_;
  if (global_output_.enabled) {
    if (!correction)
      throw std::invalid_argument("Global output requires a map correction");
    // Compose the actual legacy quaternion, not a projection of filter
    // rotation.
    const RigidTransform published{
        Eigen::Quaterniond(orientation.w, orientation.x, orientation.y,
                           orientation.z)
            .toRotationMatrix(),
        result.estimate.pos_end};
    const auto map_from_body = ApplyMapCorrection(*correction, published);
    global_pose.header = odometry.header;
    global_pose.header.frame_id = global_output_.map_frame;
    global_pose.pose.position.x = map_from_body.translation.x();
    global_pose.pose.position.y = map_from_body.translation.y();
    global_pose.pose.position.z = map_from_body.translation.z();
    const Eigen::Quaterniond rotation(map_from_body.rotation);
    global_pose.pose.orientation.w = rotation.w();
    global_pose.pose.orientation.x = rotation.x();
    global_pose.pose.orientation.y = rotation.y();
    global_pose.pose.orientation.z = rotation.z();
  }
  geometry_msgs::msg::TransformStamped transform;
  transform.header = odometry.header;
  transform.child_frame_id = odometry.child_frame_id;
  transform.transform.translation.x = result.estimate.pos_end.x();
  transform.transform.translation.y = result.estimate.pos_end.y();
  transform.transform.translation.z = result.estimate.pos_end.z();
  transform.transform.rotation = orientation;
  if (publish_global && global_output_.publish_tf) {
    const auto& map_from_odom = correction->map_from_odom;
    geometry_msgs::msg::TransformStamped global_transform;
    global_transform.header = odometry.header;
    global_transform.header.frame_id = global_output_.map_frame;
    global_transform.child_frame_id = "camera_init";
    global_transform.transform.translation.x = map_from_odom.translation.x();
    global_transform.transform.translation.y = map_from_odom.translation.y();
    global_transform.transform.translation.z = map_from_odom.translation.z();
    const Eigen::Quaterniond rotation(map_from_odom.rotation);
    global_transform.transform.rotation.w = rotation.w();
    global_transform.transform.rotation.x = rotation.x();
    global_transform.transform.rotation.y = rotation.y();
    global_transform.transform.rotation.z = rotation.z();
    // Both edges carry the same sample time in one TFMessage.
    const std::vector<geometry_msgs::msg::TransformStamped> transforms{
        global_transform, transform};
    timing.Send([&] { broadcaster_.sendTransform(transforms); });
  } else {
    timing.Send([&] { broadcaster_.sendTransform(transform); });
  }
  timing.Send([&] { odometry_->publish(odometry); });
  if (publish_global) {
    timing.Send([&] { global_pose_->publish(global_pose); });
    last_global_stamp_ns_ = stamp_ns;
  }
  PublishCloud(result, colored_cloud, timing);
  if (result.effective_cloud) {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*result.effective_cloud, message);
    message.header.stamp = node_.now();
    message.header.frame_id = "camera_init";
    timing.Send([&] { effect_->publish(message); });
  }
  if (result.has_plane_visualization) {
    const auto markers = PlaneMarkers(result.planes);
    timing.Send([&] { planes_->publish(markers); });
  }
  geometry_msgs::msg::PoseStamped pose;
  pose.pose = odometry.pose.pose;
  pose.header.stamp = node_.now();
  pose.header.frame_id = "camera_init";
  if (path_.poses.size() == static_cast<std::size_t>(path_max_poses_)) {
    path_.poses.erase(path_.poses.begin());
  }
  path_.poses.push_back(pose);
  timing.Send([&] { path_publisher_->publish(path_); });
  pose.header.stamp = node_.now();
  timing.Send([&] { mavros_->publish(pose); });
  timing.Report();
}

void LivoPublisher::PublishImuPropagation(const FilterState& state,
                                          double timestamp) {
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

LivoPublisher::~LivoPublisher() = default;

}  // namespace livo_atlas::ros2
