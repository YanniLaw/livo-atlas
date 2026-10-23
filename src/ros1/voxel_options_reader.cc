#include "livo_atlas/ros1/voxel_options_reader.h"

namespace livo_atlas {
namespace ros1 {

VoxelMapOptions ReadVoxelOptions(const ros::NodeHandle& node) {
  VoxelMapOptions options;
  node.param("publish/pub_plane_en", options.is_pub_plane_map_, false);
  node.param("lio/max_layer", options.max_layer_, 1);
  node.param("lio/voxel_size", options.max_voxel_size_, 0.5);
  node.param("lio/min_eigen_value", options.planner_threshold_, 0.01);
  node.param("lio/sigma_num", options.sigma_num_, 3.0);
  node.param("lio/beam_err", options.beam_err_, 0.02);
  node.param("lio/dept_err", options.dept_err_, 0.05);
  node.param("lio/layer_init_num", options.layer_init_num_,
             std::vector<int>{5, 5, 5, 5, 5});
  node.param("lio/max_points_num", options.max_points_num_, 50);
  node.param("lio/max_iterations", options.max_iterations_, 5);
  node.param("local_map/map_sliding_en", options.map_sliding_en, false);
  node.param("local_map/half_map_size", options.half_map_size, 100);
  node.param("local_map/sliding_thresh", options.sliding_thresh, 8.0);
  return options;
}

}  // namespace ros1
}  // namespace livo_atlas
