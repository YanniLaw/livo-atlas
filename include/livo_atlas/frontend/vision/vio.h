/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIVO_ATLAS_FRONTEND_VISION_VIO_H_
#define LIVO_ATLAS_FRONTEND_VISION_VIO_H_

#include <opencv2/imgproc/imgproc_c.h>
#include <pcl/filters/voxel_grid.h>
#include <sophus/se3.h>

#include <opencv2/core/mat.hpp>
#include <set>
#include <unordered_map>
#include <vector>

#include "livo_atlas/camera/pinhole_camera.h"
#include "livo_atlas/common/filter_state.h"
#include "livo_atlas/common/math_types.h"
#include "livo_atlas/frontend/diagnostics.h"
#include "livo_atlas/frontend/vision/feature.h"
#include "livo_atlas/frontend/vision/frame.h"
#include "livo_atlas/mapping/voxel_map.h"
#include "livo_atlas/sensor/extrinsics.h"

namespace livo_atlas {

inline constexpr int kVisualBufferCapacity = 500;
inline constexpr int kVoxelAppendCapacity = 100;

struct SubSparseMap {
  std::vector<float> propa_errors;
  std::vector<float> errors;
  std::vector<std::vector<float>> warp_patch;
  std::vector<int> search_levels;
  std::vector<VisualPoint *> voxel_points;
  std::vector<double> inv_expo_list;
  std::vector<PointWithCovariance> add_from_voxel_map;

  SubSparseMap() {
    propa_errors.reserve(kVisualBufferCapacity);
    errors.reserve(kVisualBufferCapacity);
    warp_patch.reserve(kVisualBufferCapacity);
    search_levels.reserve(kVisualBufferCapacity);
    voxel_points.reserve(kVisualBufferCapacity);
    inv_expo_list.reserve(kVisualBufferCapacity);
    add_from_voxel_map.reserve(kVoxelAppendCapacity);
  };

  void reset() {
    propa_errors.clear();
    errors.clear();
    warp_patch.clear();
    search_levels.clear();
    voxel_points.clear();
    inv_expo_list.clear();
    add_from_voxel_map.clear();
  }
};

class PatchWarp {
 public:
  Eigen::Matrix2d A_cur_ref;
  int search_level;
  PatchWarp(int level, Eigen::Matrix2d warp_matrix)
      : search_level(level), A_cur_ref(warp_matrix) {}
  ~PatchWarp() {}
};

class VisualVoxel {
 public:
  std::vector<VisualPoint *> voxel_points;
  int count;
  VisualVoxel(int num) : count(num) {}
  ~VisualVoxel() {
    for (VisualPoint *vp : voxel_points) {
      if (vp != nullptr) {
        delete vp;
        vp = nullptr;
      }
    }
  }
};

class VIOManager {
 public:
  int grid_size;
  livo_atlas::Camera *cam;
  FilterState *state;
  FilterState *state_propagat;
  Matrix3d Rli, Rci, Rcl, Rcw, Jdphi_dR, Jdp_dt, Jdp_dR;
  Vector3d Pli, Pci, Pcl, Pcw;
  std::vector<int> grid_num;
  std::vector<int> map_index;
  std::vector<int> border_flag;
  std::vector<int> update_flag;
  std::vector<float> map_dist;
  std::vector<float> scan_value;
  std::vector<float> patch_buffer;
  bool normal_en, inverse_composition_en, exposure_estimate_en, raycast_en,
      has_ref_patch_cache;
  bool ncc_en = false;

  int width, height, grid_n_width, grid_n_height, length;
  double image_resize_factor;
  double fx, fy, cx, cy;
  int patch_pyramid_level, patch_size, patch_size_total, patch_size_half,
      border, warp_len;
  int max_iterations, total_points;

  double img_point_cov, outlier_threshold, ncc_thre;

  SubSparseMap *visual_submap;
  std::vector<std::vector<Vector3d>> rays_with_sample_points;

  bool plot_flag;

  Eigen::Matrix<double, kStateDimension, kStateDimension> G, H_T_H;
  Eigen::MatrixXd K, H_sub_inv;

  std::vector<livo_atlas::DebugImage> debug_images_;
  std::unordered_map<VOXEL_LOCATION, VisualVoxel *> feat_map;
  std::unordered_map<VOXEL_LOCATION, int> sub_feat_map;
  std::unordered_map<int, PatchWarp *> warp_map;
  std::vector<VisualPoint *> retrieve_voxel_points;
  std::vector<PointWithCovariance> append_voxel_points;
  FramePtr new_frame_;
  cv::Mat img_cp, img_rgb, img_test;

  enum CellType { TYPE_MAP = 1, TYPE_POINTCLOUD, TYPE_UNKNOWN };

  VIOManager();
  ~VIOManager();
  void UpdateStateInverse(cv::Mat img, int level);
  void UpdateState(cv::Mat img, int level);
  void ProcessFrame(
      cv::Mat &img, std::vector<PointWithCovariance> &pg,
      const std::unordered_map<VOXEL_LOCATION, VoxelOctreeNode *> &feat_map,
      double img_time);
  void RetrieveFromVisualSparseMap(
      cv::Mat img, std::vector<PointWithCovariance> &pg,
      const std::unordered_map<VOXEL_LOCATION, VoxelOctreeNode *> &plane_map);
  void GenerateVisualMapPoints(cv::Mat img,
                               std::vector<PointWithCovariance> &pg);
  void SetImuToLidarExtrinsic(const Vector3d &transl, const Matrix3d &rot);
  void SetLidarToCameraExtrinsic(const livo_atlas::RigidTransform &transform);
  void InitializeVio();
  void GetImagePatch(cv::Mat img, Vector2d pc, float *patch_tmp, int level);
  void ComputeProjectionJacobian(Vector3d p,
                                 Eigen::Matrix<double, (2), (3)> &J);
  void ComputeJacobianAndUpdateEkf(cv::Mat img);
  void ResetGrid();
  void UpdateVisualMapPoints(cv::Mat img);
  void GetAffineWarpMatrix(const livo_atlas::Camera &cam,
                           const Vector2d &px_ref, const Vector3d &f_ref,
                           const double depth_ref, const Sophus::SE3 &T_cur_ref,
                           const int level_ref, const int pyramid_level,
                           const int halfpatch_size,
                           Eigen::Matrix2d &A_cur_ref);
  void GetHomographyWarpMatrix(const livo_atlas::Camera &cam,
                               const Vector2d &px_ref, const Vector3d &xyz_ref,
                               const Vector3d &normal_ref,
                               const Sophus::SE3 &T_cur_ref,
                               const int level_ref, Eigen::Matrix2d &A_cur_ref);
  void WarpPatch(const Eigen::Matrix2d &A_cur_ref, const cv::Mat &img_ref,
                 const Vector2d &px_ref, const int level_ref,
                 const int search_level, const int pyramid_level,
                 const int halfpatch_size, float *patch);
  void InsertPointIntoVoxelMap(VisualPoint *pt_new);
  void DrawTrackedPoints();
  void UpdateFrameState(FilterState state);
  void ProjectReferencePatches(
      const std::unordered_map<VOXEL_LOCATION, VoxelOctreeNode *> &plane_map);
  void UpdateReferencePatches(
      const std::unordered_map<VOXEL_LOCATION, VoxelOctreeNode *> &plane_map);
  void PrecomputeReferencePatches(int level);
  double ComputeNcc(float *ref_patch, float *cur_patch, int patch_size);
  int SelectSearchLevel(const Eigen::Matrix2d &A_cur_ref, const int max_level);

  // void resetRvizDisplay();
  // deque<VisualPoint *> map_cur_frame;
  // deque<VisualPoint *> sub_map_ray;
  // deque<VisualPoint *> sub_map_ray_fov;
  // deque<VisualPoint *> visual_sub_map_cur;
  // deque<VisualPoint *> visual_converged_point;
  // std::vector<std::vector<V3D>> sample_points;

  // PointCloudXYZI::Ptr pg_down;
  // pcl::VoxelGrid<PointType> downSizeFilter;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_FRONTEND_VISION_VIO_H_
