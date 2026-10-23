#ifndef LIVO_ATLAS_ROS1_VOXEL_OPTIONS_READER_H_
#define LIVO_ATLAS_ROS1_VOXEL_OPTIONS_READER_H_

#include <ros/node_handle.h>

#include "livo_atlas/mapping/voxel_map_options.h"

namespace livo_atlas {
namespace ros1 {

VoxelMapOptions ReadVoxelOptions(const ros::NodeHandle& node);

}  // namespace ros1
}  // namespace livo_atlas

#endif  // LIVO_ATLAS_ROS1_VOXEL_OPTIONS_READER_H_
