#include "livo_atlas/ros1/keyframe_trajectory_publisher.h"

#include <livo_atlas/KeyframeTrajectory.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <cstdint>
#include <stdexcept>
#include <utility>

#include "livo_atlas/core/global_output_options.h"
#include "ros_common/trajectory_messages.h"

namespace livo_atlas::ros1 {

KeyframeTrajectoryPublisher::KeyframeTrajectoryPublisher(ros::NodeHandle& node,
                                                         std::string map_frame)
    : map_frame_(std::move(map_frame)) {
  GlobalOutputOptions options;
  options.enabled = true;
  options.map_frame = map_frame_;
  ValidateGlobalOutputOptions(options);
  trajectory_ = node.advertise<livo_atlas::KeyframeTrajectory>(
      "/keyframe_trajectory", 1, true);
  path_ = node.advertise<nav_msgs::Path>("/keyframe_path", 1, true);
}

void KeyframeTrajectoryPublisher::Publish(
    const TrajectorySnapshot& snapshot) const {
  if (!ros::ok()) return;
  auto [message, path] =
      ros_detail::MakeTrajectoryMessages<livo_atlas::KeyframeTrajectory,
                                         nav_msgs::Path>(
          snapshot, map_frame_, [](std::int64_t ns) {
            return ros::Time(static_cast<std::uint32_t>(ns / 1000000000LL),
                             static_cast<std::uint32_t>(ns % 1000000000LL));
          });
  trajectory_.publish(message);
  path_.publish(path);
  if (ros::ok() && snapshot.status == TrajectoryStatus::kReady &&
      message.status == livo_atlas::KeyframeTrajectory::ERROR)
    throw std::invalid_argument(message.error);
}

}  // namespace livo_atlas::ros1
