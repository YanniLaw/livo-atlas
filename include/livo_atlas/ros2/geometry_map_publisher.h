#ifndef LIVO_ATLAS_ROS2_GEOMETRY_MAP_PUBLISHER_H_
#define LIVO_ATLAS_ROS2_GEOMETRY_MAP_PUBLISHER_H_

#include <string>

#include <livo_atlas/msg/geometry_map.hpp>
#include <rclcpp/context.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "livo_atlas/mapping/geometry_map_snapshot.h"

namespace livo_atlas::ros2 {

class GeometryMapPublisher {
 public:
  GeometryMapPublisher(rclcpp::Node& node, std::string map_frame);
  // Called serially on the geometry worker. A malformed ready payload with a
  // valid identity publishes an empty error before reporting failure.
  void Publish(const GeometryMapSnapshot& snapshot) const;

 private:
  std::string map_frame_;
  rclcpp::Context::SharedPtr context_;
  rclcpp::Publisher<livo_atlas::msg::GeometryMap>::SharedPtr geometry_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_;
};

}  // namespace livo_atlas::ros2

#endif  // LIVO_ATLAS_ROS2_GEOMETRY_MAP_PUBLISHER_H_
