#ifndef LIVO_ATLAS_MAPPING_VOXEL_MAP_OPTIONS_H_
#define LIVO_ATLAS_MAPPING_VOXEL_MAP_OPTIONS_H_

#include <vector>

namespace livo_atlas {

// Field names/order remain compatible with the existing map implementation.
struct VoxelMapOptions {
  double max_voxel_size_ = 0.5;
  int max_layer_ = 1;
  int max_iterations_ = 5;
  std::vector<int> layer_init_num_{5, 5, 5, 5, 5};
  int max_points_num_ = 50;
  double planner_threshold_ = 0.01;
  double beam_err_ = 0.02;
  double dept_err_ = 0.05;
  double sigma_num_ = 3;
  bool is_pub_plane_map_ = false;
  double sliding_thresh = 8;
  bool map_sliding_en = false;
  int half_map_size = 100;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_MAPPING_VOXEL_MAP_OPTIONS_H_
