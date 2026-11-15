#include "livo_atlas/ros1/geometry_map_publisher.h"

#include <cstdint>
#include <stdexcept>
#include <utility>

#include <livo_atlas/GeometryMap.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "livo_atlas/core/global_output_options.h"
#include "ros/geometry_messages.h"

namespace livo_atlas::ros1 {

GeometryMapPublisher::GeometryMapPublisher(ros::NodeHandle& node,
                                           std::string map_frame)
    : map_frame_(std::move(map_frame)) {
  GlobalOutputOptions options;
  options.enabled = options.geometry.enabled = true;
  options.map_frame = map_frame_;
  ValidateGlobalOutputOptions(options);
  geometry_map_ =
      node.advertise<livo_atlas::GeometryMap>("/geometry_map", 1, true);
  global_map_ =
      node.advertise<sensor_msgs::PointCloud2>("/global_map", 1, true);
}

void GeometryMapPublisher::Publish(const GeometryMapSnapshot& snapshot) const {
  if (!ros::ok()) return;
  auto [message, cloud] =
      ros_detail::MakeGeometryMessages<livo_atlas::GeometryMap,
                                       sensor_msgs::PointCloud2>(
          snapshot, map_frame_, [](std::int64_t ns) {
            return ros::Time(static_cast<std::uint32_t>(ns / 1000000000LL),
                             static_cast<std::uint32_t>(ns % 1000000000LL));
          });
  message.header.seq = sequence_;
  message.cloud.header.seq = sequence_;
  cloud.header.seq = sequence_++;
  geometry_map_.publish(message);
  global_map_.publish(cloud);
  if (ros::ok() && snapshot.status == GeometryMapStatus::kReady &&
      message.status == livo_atlas::GeometryMap::ERROR)
    throw std::invalid_argument(message.error);
}

}  // namespace livo_atlas::ros1
