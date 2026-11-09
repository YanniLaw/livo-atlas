#ifndef LIVO_ATLAS_ROS2_LIVO_PUBLISHER_H_
#define LIVO_ATLAS_ROS2_LIVO_PUBLISHER_H_

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "livo_atlas/common/filter_state.h"
#include "livo_atlas/frontend/frontend_result.h"
#include "livo_atlas/frontend/synchronized_measurements.h"
#include "livo_atlas/sensor/point_types.h"

namespace livo_atlas::ros2 {

class LivoPublisher {
 public:
  LivoPublisher(rclcpp::Node& node, FrontendMode mode,
                int image_queue_size = 1);
  ~LivoPublisher();
  LivoPublisher(const LivoPublisher&) = delete;
  LivoPublisher& operator=(const LivoPublisher&) = delete;
  void Publish(const FrontendResult& result,
               const ColoredCloud::ConstPtr& colored_cloud);
  void PublishImuPropagation(const FilterState& state, double timestamp);

 private:
  void PublishCloud(const FrontendResult& result,
                    const ColoredCloud::ConstPtr& colored_cloud);

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

}  // namespace livo_atlas::ros2

#endif  // LIVO_ATLAS_ROS2_LIVO_PUBLISHER_H_
