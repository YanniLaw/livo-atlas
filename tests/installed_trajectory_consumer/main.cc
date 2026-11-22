#if LIVO_ATLAS_CONSUMER_ROS_VERSION == 1
#include <livo_atlas/KeyframeTrajectory.h>
#include <ros/serialization.h>
using Message = livo_atlas::KeyframeTrajectory;
#else
#include <livo_atlas/msg/keyframe_trajectory.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
using Message = livo_atlas::msg::KeyframeTrajectory;
#endif

#include <cstdint>
#include <string>
#include <vector>

#include "livo_atlas/common/logging.h"

int main(int, char** argv) {
  livo_atlas::InitializeLogging(argv[0]);
  Message message;
  message.producer_id.assign(32, 'b');
  message.revision = (std::uint64_t{1} << 40) + 3;
  message.pose_version = (std::uint64_t{1} << 36) + 1;
  message.frame_count = message.optimized_frame_count = 1;
  message.header.frame_id = "atlas/map";
  message.status = Message::READY;
  message.keyframes.resize(1);
  message.keyframes[0].timestamp = 123.125;
  message.keyframes[0].pose.position.x = 7;
  message.keyframes[0].pose.orientation.w = 1;
  Message restored;
  std::size_t bytes = 0;
#if LIVO_ATLAS_CONSUMER_ROS_VERSION == 1
  bytes = ros::serialization::serializationLength(message);
  std::vector<std::uint8_t> buffer(bytes);
  ros::serialization::OStream output(buffer.data(), buffer.size());
  ros::serialization::serialize(output, message);
  ros::serialization::IStream input(buffer.data(), buffer.size());
  ros::serialization::deserialize(input, restored);
#else
  rclcpp::Serialization<Message> serializer;
  rclcpp::SerializedMessage serialized;
  serializer.serialize_message(&message, &serialized);
  serializer.deserialize_message(&serialized, &restored);
  bytes = serialized.size();
#endif
  if (restored.producer_id != std::string(32, 'b') ||
      restored.revision != 1099511627779ULL ||
      restored.pose_version != 68719476737ULL ||
      restored.keyframes.size() != 1 ||
      restored.keyframes[0].timestamp != 123.125 ||
      restored.keyframes[0].pose.position.x != 7)
    return 1;
  LOG(INFO) << "Installed trajectory message round trip: " << bytes
            << " bytes";
  return 0;
}
