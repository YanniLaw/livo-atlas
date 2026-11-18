#ifndef LIVO_ATLAS_ROS2_GAUSSIAN_RENDER_PUBLISHER_H_
#define LIVO_ATLAS_ROS2_GAUSSIAN_RENDER_PUBLISHER_H_

#include <livo_atlas/msg/gaussian_render_frame.hpp>
#include <rclcpp/context.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

#include "livo_atlas/gaussian/gaussian_render_snapshot.h"

namespace livo_atlas::ros2 {

class GaussianRenderPublisher {
 public:
  GaussianRenderPublisher(rclcpp::Node& node, std::string optical_frame,
                          std::string map_frame);
  // Serial worker only. true means native calls completed, not delivery.
  bool Publish(const GaussianRenderSnapshot& source) const;

 private:
  std::string optical_frame_;
  std::string map_frame_;
  rclcpp::Context::SharedPtr context_;
  rclcpp::Publisher<livo_atlas::msg::GaussianRenderFrame>::SharedPtr frame_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_;
};

}  // namespace livo_atlas::ros2
#endif  // LIVO_ATLAS_ROS2_GAUSSIAN_RENDER_PUBLISHER_H_
