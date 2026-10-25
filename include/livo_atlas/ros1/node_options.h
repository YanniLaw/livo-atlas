#ifndef LIVO_ATLAS_ROS1_NODE_OPTIONS_H_
#define LIVO_ATLAS_ROS1_NODE_OPTIONS_H_

#include <ros/ros.h>

#include "livo_atlas/core/livo_config.h"

namespace livo_atlas::ros1 {

struct NodeOptions {
  LivoConfig livo;
};

NodeOptions ReadNodeOptions(ros::NodeHandle& node);

}  // namespace livo_atlas::ros1
#endif  // LIVO_ATLAS_ROS1_NODE_OPTIONS_H_
