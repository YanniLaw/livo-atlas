#include "livo_atlas/ros1/livo_publisher.h"

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

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

#include "livo_atlas/common/so3_math.h"
#include "ros_common/publication_timing.h"

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

geometry_msgs::Pose Pose(const FilterState& state,
                         const geometry_msgs::Quaternion& orientation) {
  geometry_msgs::Pose pose;
  pose.position.x = state.pos_end.x();
  pose.position.y = state.pos_end.y();
  pose.position.z = state.pos_end.z();
  pose.orientation = orientation;
  return pose;
}

}  // namespace

LivoPublisher::LivoPublisher(FrontendMode mode, int image_queue_size,
                             GlobalOutputOptions global_output)
    : mode_(mode),
      image_queue_size_(image_queue_size),
      global_output_(std::move(global_output)) {
  ValidateGlobalOutputOptions(global_output_);
  if (image_queue_size <= 0)
    throw std::invalid_argument("Image queue size must be positive");
}

void LivoPublisher::Initialize(ros::NodeHandle& node,
                               image_transport::ImageTransport& transport) {
  if (global_output_.enabled)
    global_pose_ =
        node.advertise<geometry_msgs::PoseStamped>("/global_pose", 1, false);
  cloud_ = node.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);
  effect_ = node.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100);
  odometry_ = node.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
  path_publisher_ = node.advertise<nav_msgs::Path>("/path", 10);
  mavros_ = node.advertise<geometry_msgs::PoseStamped>(
      "/mavros/vision_pose/pose", 10);
  propagation_ =
      node.advertise<nav_msgs::Odometry>("/LIVO2/imu_propagate", 10000);
  planes_ = node.advertise<visualization_msgs::MarkerArray>("/planes", 10000);
  image_ = transport.advertise("/rgb_img", image_queue_size_);
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

void LivoPublisher::PublishCloud(const FrontendResult& result,
                                 const ColoredCloud::ConstPtr& colored_cloud,
                                 ros_detail::PublicationTiming& timing) {
  if (!colored_cloud) return;
  sensor_msgs::PointCloud2 message;
  if (mode_ == FrontendMode::kLidarInertialVisual) {
    if (result.kind == UpdateKind::kVisual) {
      pcl::toROSMsg(*colored_cloud, message);
    }
  } else {
    pcl::toROSMsg(*result.world_cloud, message);
  }
  message.header.stamp = ros::Time::now();
  message.header.frame_id = "camera_init";
  timing.Send([&] { cloud_.publish(message); });
}

void LivoPublisher::Publish(const FrontendResult& result,
                            const ColoredCloud::ConstPtr& colored_cloud,
                            const std::optional<MapCorrection>& correction,
                            bool stage_timing) {
  if (!result.updated) return;
  ros_detail::PublicationTiming timing(stage_timing, result.kind);
  if (result.kind == UpdateKind::kVisual) {
    PublishCloud(result, colored_cloud, timing);
    cv_bridge::CvImage image;
    image.header.stamp = ros::Time::now();
    image.encoding = sensor_msgs::image_encodings::BGR8;
    image.image = result.tracking_image;
    const auto message = image.toImageMsg();
    timing.Send([&] { image_.publish(message); });
    timing.Report();
    return;
  }
  const Eigen::Vector3d euler = RotationMatrixToEuler(result.estimate.rot_end);
  const auto orientation =
      tf::createQuaternionMsgFromRollPitchYaw(euler.x(), euler.y(), euler.z());
  nav_msgs::Odometry odometry;
  odometry.header.frame_id = "camera_init";
  odometry.child_frame_id = "aft_mapped";
  odometry.header.stamp = ros::Time::now();
  odometry.pose.pose = Pose(result.estimate, orientation);
  geometry_msgs::PoseStamped global_pose;
  const std::int64_t stamp_ns = odometry.header.stamp.toNSec();
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
  tf::Transform transform;
  const auto& position = result.estimate.pos_end;
  transform.setOrigin(tf::Vector3(position.x(), position.y(), position.z()));
  transform.setRotation(tf::Quaternion(orientation.x, orientation.y,
                                       orientation.z, orientation.w));
  const tf::StampedTransform raw_tf(transform, odometry.header.stamp,
                                    "camera_init", "aft_mapped");
  if (publish_global && global_output_.publish_tf) {
    const auto& map_from_odom = correction->map_from_odom;
    const Eigen::Quaterniond rotation(map_from_odom.rotation);
    tf::Transform global_transform;
    global_transform.setOrigin(tf::Vector3(map_from_odom.translation.x(),
                                           map_from_odom.translation.y(),
                                           map_from_odom.translation.z()));
    global_transform.setRotation(
        tf::Quaternion(rotation.x(), rotation.y(), rotation.z(), rotation.w()));
    // Both edges carry the same sample time in one TFMessage.
    const std::vector<tf::StampedTransform> transforms{
        {global_transform, odometry.header.stamp, global_output_.map_frame,
         "camera_init"},
        raw_tf};
    timing.Send([&] { broadcaster_.sendTransform(transforms); });
  } else {
    timing.Send([&] { broadcaster_.sendTransform(raw_tf); });
  }
  timing.Send([&] { odometry_.publish(odometry); });
  if (publish_global) {
    timing.Send([&] { global_pose_.publish(global_pose); });
    last_global_stamp_ns_ = stamp_ns;
  }
  PublishCloud(result, colored_cloud, timing);
  if (result.effective_cloud) {
    sensor_msgs::PointCloud2 message;
    pcl::toROSMsg(*result.effective_cloud, message);
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "camera_init";
    timing.Send([&] { effect_.publish(message); });
  }
  if (result.has_plane_visualization) {
    const auto markers = PlaneMarkers(result.planes);
    timing.Send([&] { planes_.publish(markers); });
  }
  geometry_msgs::PoseStamped pose;
  pose.pose = odometry.pose.pose;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "camera_init";
  path_.poses.push_back(pose);
  timing.Send([&] { path_publisher_.publish(path_); });
  pose.header.stamp = ros::Time::now();
  timing.Send([&] { mavros_.publish(pose); });
  timing.Report();
}

void LivoPublisher::PublishImuPropagation(const FilterState& state,
                                          const ros::Time& timestamp) {
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

LivoPublisher::~LivoPublisher() = default;

}  // namespace ros1
}  // namespace livo_atlas
