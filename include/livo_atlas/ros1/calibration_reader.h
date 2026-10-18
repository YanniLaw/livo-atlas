#ifndef LIVO_ATLAS_ROS1_CALIBRATION_READER_H_
#define LIVO_ATLAS_ROS1_CALIBRATION_READER_H_

#include <string>

#include <ros/node_handle.h>

#include "livo_atlas/frontend/calibration.h"

namespace livo_atlas::ros1 {

// Read extrinsics from node and camera parameters from camera_node.
// Returns an empty error string on success. On failure, result is unchanged.
std::string ReadCalibration(const ros::NodeHandle& node,
                           const ros::NodeHandle& camera_node,
                           Calibration* result);

}  // namespace livo_atlas::ros1

#endif  // LIVO_ATLAS_ROS1_CALIBRATION_READER_H_
