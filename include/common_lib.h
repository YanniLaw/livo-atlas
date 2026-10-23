/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <utils/so3_math.h>
#include <utils/types.h>
#include <utils/color.h>
#include <opencv2/opencv.hpp>
#include <sophus/se3.h>

#include <deque>

#include "livo_atlas/core/state.h"
#include "livo_atlas/frontend/synchronized_measurements.h"
#include "livo_atlas/sensor/measurements.h"

using namespace std;
using namespace Eigen;
using namespace Sophus;

#define print_line std::cout << __FILE__ << ", " << __LINE__ << std::endl;
#define G_m_s2 (livo_atlas::kGravityMetersPerSecondSquared)
#define DIM_STATE (livo_atlas::kStateDimension)
#define INIT_COV (livo_atlas::kInitialCovariance)
#define SIZE_LARGE (500)
#define SIZE_SMALL (100)
#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]
#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "Log/" + name))

enum LID_TYPE
{
  AVIA = livo_atlas::kLivox,
  VELO16 = livo_atlas::kVelodyne,
  OUST64 = livo_atlas::kOuster,
  L515 = livo_atlas::kDepthCamera,
  XT32 = livo_atlas::kXt32,
  PANDAR128 = livo_atlas::kPandar128,
  ROBOSENSE = livo_atlas::kRobosense
};
// Compatibility names for the existing estimator; the core owns the types.
using SLAM_MODE = livo_atlas::FrontendMode;
constexpr SLAM_MODE ONLY_LO = SLAM_MODE::kLidarOnly;
constexpr SLAM_MODE ONLY_LIO = SLAM_MODE::kLidarInertial;
constexpr SLAM_MODE LIVO = SLAM_MODE::kLidarInertialVisual;
using EKF_STATE = livo_atlas::UpdateKind;
constexpr EKF_STATE WAIT = EKF_STATE::kWaiting;
constexpr EKF_STATE VIO = EKF_STATE::kVisual;
constexpr EKF_STATE LIO = EKF_STATE::kLidarInertial;
constexpr EKF_STATE LO = EKF_STATE::kLidarOnly;
using MeasureGroup = livo_atlas::MeasurementGroup;
using LidarMeasureGroup = livo_atlas::SynchronizedMeasurements;

typedef struct pointWithVar
{
  Eigen::Vector3d point_b;     // point in the lidar body frame
  Eigen::Vector3d point_i;     // point in the imu body frame
  Eigen::Vector3d point_w;     // point in the world frame
  Eigen::Matrix3d var_nostate; // the var removed the state covarience
  Eigen::Matrix3d body_var;
  Eigen::Matrix3d var;
  Eigen::Matrix3d point_crossmat;
  Eigen::Vector3d normal;
  pointWithVar()
  {
    var_nostate = Eigen::Matrix3d::Zero();
    var = Eigen::Matrix3d::Zero();
    body_var = Eigen::Matrix3d::Zero();
    point_crossmat = Eigen::Matrix3d::Zero();
    point_b = Eigen::Vector3d::Zero();
    point_i = Eigen::Vector3d::Zero();
    point_w = Eigen::Vector3d::Zero();
    normal = Eigen::Vector3d::Zero();
  };
} pointWithVar;


using StatesGroup = livo_atlas::State;

template <typename T>
auto set_pose6d(const double t, const Matrix<T, 3, 1> &a, const Matrix<T, 3, 1> &g, const Matrix<T, 3, 1> &v, const Matrix<T, 3, 1> &p,
                const Matrix<T, 3, 3> &R)
{
  Pose6D rot_kp;
  rot_kp.offset_time = t;
  for (int i = 0; i < 3; i++)
  {
    rot_kp.acc[i] = a(i);
    rot_kp.gyr[i] = g(i);
    rot_kp.vel[i] = v(i);
    rot_kp.pos[i] = p(i);
    for (int j = 0; j < 3; j++)
      rot_kp.rot[i * 3 + j] = R(i, j);
  }
  // Map<M3D>(rot_kp.rot, 3,3) = R;
  return move(rot_kp);
}

#endif
