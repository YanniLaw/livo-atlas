#ifndef LIVO_ATLAS_INCLUDE_LIVO_ATLAS_ROS_SENSOR_EXTRINSICS_H_
#define LIVO_ATLAS_INCLUDE_LIVO_ATLAS_ROS_SENSOR_EXTRINSICS_H_

#include <string>

#include <ros/ros.h>

#include "livo_atlas/core/sensor_extrinsics.h"

namespace livo_atlas {

// Load and validate the four required legacy extrinsic parameters. On failure,
// output is left unchanged and error identifies the invalid parameter.
bool LoadSensorExtrinsics(const ros::NodeHandle& nh, SensorExtrinsics* output,
                          std::string* error);

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_INCLUDE_LIVO_ATLAS_ROS_SENSOR_EXTRINSICS_H_
