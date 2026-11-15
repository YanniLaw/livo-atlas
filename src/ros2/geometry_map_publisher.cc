#include "livo_atlas/ros2/geometry_map_publisher.h"

#include <cstdint>
#include <stdexcept>
#include <utility>

#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

#include "livo_atlas/core/global_output_options.h"
#include "ros/geometry_messages.h"

namespace livo_atlas::ros2 {

GeometryMapPublisher::GeometryMapPublisher(rclcpp::Node& node,
                                           std::string map_frame)
    : map_frame_(std::move(map_frame)),
      context_(node.get_node_base_interface()->get_context()) {
  GlobalOutputOptions options;
  options.enabled = options.geometry.enabled = true;
  options.map_frame = map_frame_;
  ValidateGlobalOutputOptions(options);
  const auto qos = rclcpp::QoS(1).reliable().transient_local();
  geometry_map_ =
      node.create_publisher<livo_atlas::msg::GeometryMap>("/geometry_map", qos);
  global_map_ =
      node.create_publisher<sensor_msgs::msg::PointCloud2>("/global_map", qos);
}

void GeometryMapPublisher::Publish(const GeometryMapSnapshot& snapshot) const {
  if (!rclcpp::ok(context_)) return;
  auto [message, cloud] =
      ros_detail::MakeGeometryMessages<livo_atlas::msg::GeometryMap,
                                       sensor_msgs::msg::PointCloud2>(
          snapshot, map_frame_, [](std::int64_t ns) {
            builtin_interfaces::msg::Time result;
            result.sec = static_cast<std::int32_t>(ns / 1000000000LL);
            result.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);
            return result;
          });
  geometry_map_->publish(message);
  global_map_->publish(cloud);
  if (rclcpp::ok(context_) && snapshot.status == GeometryMapStatus::kReady &&
      message.status == livo_atlas::msg::GeometryMap::ERROR)
    throw std::invalid_argument(message.error);
}

}  // namespace livo_atlas::ros2
