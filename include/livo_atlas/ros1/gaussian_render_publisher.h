#ifndef LIVO_ATLAS_ROS1_GAUSSIAN_RENDER_PUBLISHER_H_
#define LIVO_ATLAS_ROS1_GAUSSIAN_RENDER_PUBLISHER_H_

#include <ros/publisher.h>

#include <cstdint>
#include <string>

#include "livo_atlas/gaussian/gaussian_render_snapshot.h"

namespace ros {
class NodeHandle;
}

namespace livo_atlas::ros1 {

class GaussianRenderPublisher {
 public:
  GaussianRenderPublisher(ros::NodeHandle& node, std::string optical_frame,
                          std::string map_frame);
  // Serial worker only. true means native calls completed, not delivery.
  bool Publish(const GaussianRenderSnapshot& source) const;

 private:
  std::string optical_frame_;
  std::string map_frame_;
  ros::Publisher frame_;
  ros::Publisher image_;
  mutable std::uint32_t sequence_ = 0;
};

}  // namespace livo_atlas::ros1
#endif  // LIVO_ATLAS_ROS1_GAUSSIAN_RENDER_PUBLISHER_H_
