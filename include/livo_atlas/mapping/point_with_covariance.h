#ifndef LIVO_ATLAS_MAPPING_POINT_WITH_COVARIANCE_H_
#define LIVO_ATLAS_MAPPING_POINT_WITH_COVARIANCE_H_

#include <Eigen/Core>

namespace livo_atlas {

struct PointWithCovariance {
  Eigen::Vector3d point_b;      // point in the lidar body frame
  Eigen::Vector3d point_i;      // point in the imu body frame
  Eigen::Vector3d point_w;      // point in the world frame
  Eigen::Matrix3d var_nostate;  // the var removed the state covarience
  Eigen::Matrix3d body_var;
  Eigen::Matrix3d var;
  Eigen::Matrix3d point_crossmat;
  Eigen::Vector3d normal;
  PointWithCovariance() {
    var_nostate = Eigen::Matrix3d::Zero();
    var = Eigen::Matrix3d::Zero();
    body_var = Eigen::Matrix3d::Zero();
    point_crossmat = Eigen::Matrix3d::Zero();
    point_b = Eigen::Vector3d::Zero();
    point_i = Eigen::Vector3d::Zero();
    point_w = Eigen::Vector3d::Zero();
    normal = Eigen::Vector3d::Zero();
  };
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_MAPPING_POINT_WITH_COVARIANCE_H_
