/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIVO_ATLAS_SENSOR_LIDAR_PREPROCESSOR_H_
#define LIVO_ATLAS_SENSOR_LIDAR_PREPROCESSOR_H_

#include <memory>
#include <set>
#include <vector>

#include "livo_atlas/common/math_types.h"
#include "livo_atlas/sensor/lidar_scan.h"
#include "livo_atlas/sensor/point_types.h"

namespace livo_atlas {

enum LidarFeatureType {
  kUnclassified,
  kPotentialPlane,
  kPlane,
  kJumpEdge,
  kPlaneEdge,
  kWire,
  kInvalidPoint
};
enum NeighborSide { kPreviousNeighbor, kNextNeighbor };
enum NeighborJumpType { kNormal, kNearZero, kOpposite, kInfinite, kBlind };

struct LidarPointAttributes {
  double range;
  double dista;
  double angle[2];
  double intersect;
  NeighborJumpType edj[2];
  LidarFeatureType ftype;
  LidarPointAttributes() {
    range = 0;
    edj[kPreviousNeighbor] = kNormal;
    edj[kNextNeighbor] = kNormal;
    ftype = kUnclassified;
    intersect = 2;
  }
};

class LidarPreprocessor {
 public:
  //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LidarPreprocessor();
  ~LidarPreprocessor();

  void Process(const livo_atlas::LidarScan &scan, PointCloud *output);

  void Configure(bool feat_en, int lid_type, double bld, int pfilt_num);

  PointCloud pl_full, pl_corn, pl_surf;
  PointCloud pl_buff[128];                        // maximum 128 line lidar
  std::vector<LidarPointAttributes> typess[128];  // maximum 128 line lidar
  int lidar_type, point_filter_num, N_SCANS;

  double blind, blind_sqr;
  bool feature_enabled, given_offset_time;

 private:
  void ProcessPoints(const std::vector<livo_atlas::LivoxPoint> &points);
  void ProcessPoints(const pcl::PointCloud<livo_atlas::OusterPoint> &pl_orig);
  void ProcessPoints(const pcl::PointCloud<livo_atlas::VelodynePoint> &pl_orig);
  void ProcessPoints(const pcl::PointCloud<livo_atlas::Xt32Point> &pl_orig);
  void ProcessPoints(
      const pcl::PointCloud<livo_atlas::Pandar128Point> &pl_orig);
  void ProcessPoints(
      const pcl::PointCloud<livo_atlas::RobosensePoint> &pl_orig);
  void ProcessPoints(const pcl::PointCloud<pcl::PointXYZRGB> &pl_orig);
  void ClassifyFeatures(PointCloud &pl,
                        std::vector<LidarPointAttributes> &types);
  int ClassifyPlane(const PointCloud &pl,
                    std::vector<LidarPointAttributes> &types, uint i,
                    uint &i_nex, Eigen::Vector3d &curr_direct);
  bool ClassifySmallPlane(const PointCloud &pl,
                          std::vector<LidarPointAttributes> &types, uint i_cur,
                          uint &i_nex, Eigen::Vector3d &curr_direct);
  bool ClassifyJumpEdge(const PointCloud &pl,
                        std::vector<LidarPointAttributes> &types, uint i,
                        NeighborSide nor_dir);

  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_SENSOR_LIDAR_PREPROCESSOR_H_
