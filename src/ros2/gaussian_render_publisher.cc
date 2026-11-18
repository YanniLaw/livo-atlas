#include "livo_atlas/ros2/gaussian_render_publisher.h"

#include <cstdint>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <stdexcept>
#include <utility>

#include "livo_atlas/core/gaussian_render_output_options.h"
#include "ros/gaussian_render_messages.h"

namespace livo_atlas::ros2 {

GaussianRenderPublisher::GaussianRenderPublisher(rclcpp::Node& node,
                                                 std::string optical_frame,
                                                 std::string map_frame)
    : optical_frame_(std::move(optical_frame)),
      map_frame_(std::move(map_frame)),
      context_(node.get_node_base_interface()->get_context()) {
  GaussianRenderOutputOptions options;
  options.enabled = true;
  options.optical_frame = optical_frame_;
  ValidateGaussianRenderOutputOptions(options, map_frame_);
  const auto qos = rclcpp::QoS(1).reliable().transient_local();
  frame_ = node.create_publisher<livo_atlas::msg::GaussianRenderFrame>(
      "gaussian_render", qos);
  image_ = node.create_publisher<sensor_msgs::msg::Image>(
      "gaussian_render/image", qos);
}

bool GaussianRenderPublisher::Publish(
    const GaussianRenderSnapshot& source) const {
  if (!rclcpp::ok(context_)) return false;
  auto message = ros_detail::MakeGaussianRenderMessage<
      livo_atlas::msg::GaussianRenderFrame>(
      source, optical_frame_, map_frame_, [](std::int64_t ns) {
        builtin_interfaces::msg::Time result;
        result.sec = static_cast<std::int32_t>(ns / 1000000000LL);
        result.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);
        return result;
      });
  frame_->publish(message);
  image_->publish(message.image);
  if (source.status == GaussianRenderStatus::kReady &&
      message.status == livo_atlas::msg::GaussianRenderFrame::ERROR)
    throw std::invalid_argument(message.reason);
  return rclcpp::ok(context_);
}

}  // namespace livo_atlas::ros2
