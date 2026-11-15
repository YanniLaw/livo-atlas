#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#if LIVO_ATLAS_CONSUMER_ROS_VERSION == 1
#include <livo_atlas/GeometryMap.h>
#include <ros/serialization.h>
using Message = livo_atlas::GeometryMap;
#else
#include <livo_atlas/msg/geometry_map.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
using Message = livo_atlas::msg::GeometryMap;
#endif

int main() {
  Message message;
  message.producer_id.assign(32, 'b');
  message.revision = (std::uint64_t{1} << 40) + 3;
  message.pose_version = (std::uint64_t{1} << 36) + 1;
  message.frame_count = message.optimized_frame_count = 1;
  message.submap_count = 1;
  message.header.frame_id = "atlas/map";
  message.status = Message::READY;
  message.representation = "voxel_moments_v1";
  message.sources.resize(1);
  message.sources[0].representation_revision = (std::uint64_t{1} << 38) + 5;
  message.sources[0].built_pose_version = message.pose_version;
  message.sources[0].point_count = 1;
  message.cloud.header = message.header;
  message.cloud.height = 1;
  message.cloud.width = 1;
  message.cloud.point_step = 16;
  message.cloud.row_step = 16;
  message.cloud.data.resize(16);
  message.cloud.data[2] = 0x80;
  message.cloud.data[3] = 0x3f;
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
      restored.pose_version != 68719476737ULL || restored.sources.size() != 1 ||
      restored.sources[0].representation_revision != 274877906949ULL ||
      restored.sources[0].built_pose_version != restored.pose_version ||
      restored.cloud.header.frame_id != "atlas/map" ||
      restored.cloud.data.size() != 16 || restored.cloud.data[2] != 0x80 ||
      restored.cloud.data[3] != 0x3f)
    return 1;
  std::cout << "Installed geometry message round trip: " << bytes << " bytes\n";
  return 0;
}
