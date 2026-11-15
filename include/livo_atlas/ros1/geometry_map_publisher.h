#ifndef LIVO_ATLAS_ROS1_GEOMETRY_MAP_PUBLISHER_H_
#define LIVO_ATLAS_ROS1_GEOMETRY_MAP_PUBLISHER_H_

#include <cstdint>
#include <string>

#include <ros/publisher.h>

#include "livo_atlas/mapping/geometry_map_snapshot.h"

namespace ros {
class NodeHandle;
}

namespace livo_atlas::ros1 {

class GeometryMapPublisher {
 public:
  GeometryMapPublisher(ros::NodeHandle& node, std::string map_frame);
  // Called serially on the geometry worker. A malformed ready payload with a
  // valid identity publishes an empty error before reporting failure.
  void Publish(const GeometryMapSnapshot& snapshot) const;

 private:
  std::string map_frame_;
  ros::Publisher geometry_map_;
  ros::Publisher global_map_;
  // ROS1 rewrites only the outer Header sequence during serialization.
  mutable std::uint32_t sequence_ = 0;
};

}  // namespace livo_atlas::ros1

#endif  // LIVO_ATLAS_ROS1_GEOMETRY_MAP_PUBLISHER_H_
