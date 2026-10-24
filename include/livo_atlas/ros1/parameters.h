#ifndef LIVO_ATLAS_ROS1_PARAMETERS_H_
#define LIVO_ATLAS_ROS1_PARAMETERS_H_

#include <ros/ros.h>

#include "livo_atlas/runtime/mapping_options.h"

namespace livo_atlas::ros1 {

struct NodeParameters {
  MappingOptions mapping;
};

NodeParameters ReadParameters(ros::NodeHandle& node);

}  // namespace livo_atlas::ros1
#endif  // LIVO_ATLAS_ROS1_PARAMETERS_H_
