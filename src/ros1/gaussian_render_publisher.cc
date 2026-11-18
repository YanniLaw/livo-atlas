#include "livo_atlas/ros1/gaussian_render_publisher.h"

#include <livo_atlas/GaussianRenderFrame.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <cstdint>
#include <stdexcept>
#include <utility>

#include "livo_atlas/core/gaussian_render_output_options.h"
#include "ros/gaussian_render_messages.h"

namespace livo_atlas::ros1 {

GaussianRenderPublisher::GaussianRenderPublisher(ros::NodeHandle& node,
                                                 std::string optical_frame,
                                                 std::string map_frame)
    : optical_frame_(std::move(optical_frame)),
      map_frame_(std::move(map_frame)) {
  GaussianRenderOutputOptions options;
  options.enabled = true;
  options.optical_frame = optical_frame_;
  ValidateGaussianRenderOutputOptions(options, map_frame_);
  frame_ = node.advertise<GaussianRenderFrame>("gaussian_render", 1, true);
  image_ = node.advertise<sensor_msgs::Image>("gaussian_render/image", 1, true);
}

bool GaussianRenderPublisher::Publish(
    const GaussianRenderSnapshot& source) const {
  if (!ros::ok()) return false;
  auto message = ros_detail::MakeGaussianRenderMessage<GaussianRenderFrame>(
      source, optical_frame_, map_frame_, [](std::int64_t ns) {
        return ros::Time(static_cast<std::uint32_t>(ns / 1000000000LL),
                         static_cast<std::uint32_t>(ns % 1000000000LL));
      });
  // ROS1 rewrites the outer Header seq; keep nested/mirrored headers aligned.
  message.header.seq = sequence_;
  message.image.header.seq = sequence_++;
  frame_.publish(message);
  image_.publish(message.image);
  if (source.status == GaussianRenderStatus::kReady &&
      message.status == GaussianRenderFrame::ERROR)
    throw std::invalid_argument(message.reason);
  return ros::ok();
}

}  // namespace livo_atlas::ros1
