#ifndef LIVO_ATLAS_INCLUDE_LIVO_ATLAS_ROS_CAMERA_CONFIG_H_
#define LIVO_ATLAS_INCLUDE_LIVO_ATLAS_ROS_CAMERA_CONFIG_H_

#include <memory>
#include <string>

#include <ros/node_handle.h>
#include <vikit/abstract_camera.h>

#include "livo_atlas/core/camera_config.h"

namespace livo_atlas {
namespace ros {

// Read the legacy single-camera parameters below namespace_prefix, validate
// them, and leave output unchanged if any parameter is missing or invalid.
bool LoadCameraConfig(const ::ros::NodeHandle& nh,
                      const std::string& namespace_prefix,
                      CameraConfig* output, std::string* error);

// Construct the matching Vikit model. The returned pointer owns the model.
std::unique_ptr<vk::AbstractCamera> MakeVikitCameraModel(
    const CameraConfig& config);

}  // namespace ros
}  // namespace livo_atlas

#endif  // LIVO_ATLAS_INCLUDE_LIVO_ATLAS_ROS_CAMERA_CONFIG_H_
