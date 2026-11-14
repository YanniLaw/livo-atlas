#ifndef LIVO_ATLAS_ROS1_KEYFRAME_TRAJECTORY_PUBLISHER_H_
#define LIVO_ATLAS_ROS1_KEYFRAME_TRAJECTORY_PUBLISHER_H_

#include <ros/publisher.h>

#include <string>

#include "livo_atlas/mapping/trajectory_snapshot.h"

namespace ros {
class NodeHandle;
}

namespace livo_atlas::ros1 {

class KeyframeTrajectoryPublisher {
 public:
  KeyframeTrajectoryPublisher(ros::NodeHandle& node, std::string map_frame);
  // Called on the trajectory worker. Invalid ready payloads clear both topics
  // before throwing, so the controller can stop this derived output.
  void Publish(const TrajectorySnapshot& snapshot) const;

 private:
  std::string map_frame_;
  ros::Publisher trajectory_;
  ros::Publisher path_;
};

}  // namespace livo_atlas::ros1

#endif  // LIVO_ATLAS_ROS1_KEYFRAME_TRAJECTORY_PUBLISHER_H_
