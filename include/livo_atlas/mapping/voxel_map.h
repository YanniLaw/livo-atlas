/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIVO_ATLAS_MAPPING_VOXEL_MAP_H_
#define LIVO_ATLAS_MAPPING_VOXEL_MAP_H_

#include <Eigen/Core>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "livo_atlas/common/filter_state.h"
#include "livo_atlas/common/math_types.h"
#include "livo_atlas/frontend/diagnostics.h"
#include "livo_atlas/mapping/point_with_covariance.h"
#include "livo_atlas/mapping/voxel_map_options.h"
#include "livo_atlas/sensor/point_types.h"

namespace livo_atlas {

inline constexpr std::int64_t kVoxelHashPrime = 116101;
inline constexpr std::int64_t kVoxelHashModulus = 10000000000;

static int voxel_plane_id = 0;

typedef struct PointPlaneMatch {
  Eigen::Vector3d point_b_;
  Eigen::Vector3d point_w_;
  Eigen::Vector3d normal_;
  Eigen::Vector3d center_;
  Eigen::Matrix<double, 6, 6> plane_var_;
  Matrix3d body_cov_;
  int layer_;
  double d_;
  double eigen_value_;
  bool is_valid_;
  float dis_to_plane_;
} PointPlaneMatch;

typedef struct VoxelPlane {
  Eigen::Vector3d center_;
  Eigen::Vector3d normal_;
  Eigen::Vector3d y_normal_;
  Eigen::Vector3d x_normal_;
  Eigen::Matrix3d covariance_;
  Eigen::Matrix<double, 6, 6> plane_var_;
  float radius_ = 0;
  float min_eigen_value_ = 1;
  float mid_eigen_value_ = 1;
  float max_eigen_value_ = 1;
  float d_ = 0;
  int points_size_ = 0;
  bool is_plane_ = false;
  bool is_init_ = false;
  int id_ = 0;
  bool is_update_ = false;
  VoxelPlane() {
    plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
    covariance_ = Eigen::Matrix3d::Zero();
    center_ = Eigen::Vector3d::Zero();
    normal_ = Eigen::Vector3d::Zero();
  }
} VoxelPlane;

class VOXEL_LOCATION {
 public:
  std::int64_t x, y, z;

  VOXEL_LOCATION(std::int64_t vx = 0, std::int64_t vy = 0, std::int64_t vz = 0)
      : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOCATION &other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

// Hash value
}  // namespace livo_atlas

namespace std {
template <>
struct hash<livo_atlas::VOXEL_LOCATION> {
  std::int64_t operator()(const livo_atlas::VOXEL_LOCATION &s) const {
    return ((((s.z) * livo_atlas::kVoxelHashPrime) %
                 livo_atlas::kVoxelHashModulus +
             (s.y)) *
            livo_atlas::kVoxelHashPrime) %
               livo_atlas::kVoxelHashModulus +
           (s.x);
  }
};
}  // namespace std

namespace livo_atlas {

void ComputeBodyCovariance(Eigen::Vector3d &pb, const float range_inc,
                           const float degree_inc, Eigen::Matrix3d &cov);

class VoxelOctreeNode {
 public:
  VoxelOctreeNode() = default;
  std::vector<PointWithCovariance> temp_points_;
  VoxelPlane *plane_ptr_;
  int layer_;
  int octo_state_;  // 0 is end of tree, 1 is not
  VoxelOctreeNode *leaves_[8];
  double voxel_center_[3];  // x, y, z
  std::vector<int> layer_init_num_;
  float quater_length_;
  float planer_threshold_;
  int points_size_threshold_;
  int update_size_threshold_;
  int max_points_num_;
  int max_layer_;
  int new_points_;
  bool init_octo_;
  bool update_enable_;

  VoxelOctreeNode(int max_layer, int layer, int points_size_threshold,
                  int max_points_num, float planer_threshold)
      : max_layer_(max_layer),
        layer_(layer),
        points_size_threshold_(points_size_threshold),
        max_points_num_(max_points_num),
        planer_threshold_(planer_threshold) {
    temp_points_.clear();
    octo_state_ = 0;
    new_points_ = 0;
    update_size_threshold_ = 5;
    init_octo_ = false;
    update_enable_ = true;
    for (int i = 0; i < 8; i++) {
      leaves_[i] = nullptr;
    }
    plane_ptr_ = new VoxelPlane;
  }

  ~VoxelOctreeNode() {
    for (int i = 0; i < 8; i++) {
      delete leaves_[i];
    }
    delete plane_ptr_;
  }
  void InitializePlane(const std::vector<PointWithCovariance> &points,
                       VoxelPlane *plane);
  void InitializeOctree();
  void SplitOctreeNode();
  void Update(const PointWithCovariance &pv);

  VoxelOctreeNode *FindCorrespondence(Eigen::Vector3d pw);
  VoxelOctreeNode *Insert(const PointWithCovariance &pv);
};

class VoxelMapManager {
 public:
  VoxelMapManager() = default;
  VoxelMapOptions config_setting_;
  int current_frame_id_ = 0;
  std::unordered_map<VOXEL_LOCATION, VoxelOctreeNode *> voxel_map_;

  PointCloud::Ptr feats_undistort_;
  PointCloud::Ptr feats_down_body_;
  PointCloud::Ptr feats_down_world_;

  Matrix3d extR_;
  Vector3d extT_;
  int scan_count = 0;
  FilterState state_;
  Vector3d position_last_;

  Vector3d last_slide_position = {0, 0, 0};

  int feats_down_size_;
  int effct_feat_num_;
  std::vector<Matrix3d> cross_mat_list_;
  std::vector<Matrix3d> body_cov_list_;
  std::vector<PointWithCovariance> pv_list_;
  std::vector<PointPlaneMatch> ptpl_list_;

  VoxelMapManager(
      VoxelMapOptions &config_setting,
      std::unordered_map<VOXEL_LOCATION, VoxelOctreeNode *> &voxel_map)
      : config_setting_(config_setting), voxel_map_(voxel_map) {
    current_frame_id_ = 0;
    feats_undistort_.reset(new PointCloud());
    feats_down_body_.reset(new PointCloud());
    feats_down_world_.reset(new PointCloud());
  };

  void EstimateState(FilterState &state_propagat);
  void TransformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t,
                      const PointCloud::Ptr &input_cloud,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr &trans_cloud);

  void BuildVoxelMap();
  Eigen::Vector3f RGBFromVoxel(const Vector3d &input_point);

  void UpdateVoxelMap(const std::vector<PointWithCovariance> &input_points);

  void BuildResidualListOMP(std::vector<PointWithCovariance> &pv_list,
                            std::vector<PointPlaneMatch> &ptpl_list);

  void BuildSingleResidual(PointWithCovariance &pv,
                           const VoxelOctreeNode *current_octo,
                           const int current_layer, bool &is_sucess,
                           double &prob, PointPlaneMatch &single_ptpl);

  std::vector<livo_atlas::PlaneVisualization> GetVisualization() const;

  void SlideMap();
  void RemoveOutsideBounds(const int &x_max, const int &x_min, const int &y_max,
                           const int &y_min, const int &z_max,
                           const int &z_min);

 private:
  void GetUpdatePlane(const VoxelOctreeNode *current_octo,
                      int pub_max_voxel_layer,
                      std::vector<VoxelPlane> &plane_list) const;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_MAPPING_VOXEL_MAP_H_
