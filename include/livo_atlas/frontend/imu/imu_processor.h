/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIVO_ATLAS_FRONTEND_IMU_IMU_PROCESSOR_H_
#define LIVO_ATLAS_FRONTEND_IMU_IMU_PROCESSOR_H_

#include <Eigen/Eigen>
#include <vector>

#include "livo_atlas/common/filter_state.h"
#include "livo_atlas/common/math_types.h"
#include "livo_atlas/common/so3_math.h"
#include "livo_atlas/frontend/diagnostics.h"
#include "livo_atlas/frontend/imu/pose6d.h"
#include "livo_atlas/frontend/synchronized_measurements.h"
#include "livo_atlas/sensor/point_types.h"

namespace livo_atlas {

/// *************IMU Process and undistortion
class ImuProcessor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcessor();
  ~ImuProcessor();

  void Reset();
  void SetExtrinsic(const Vector3d &transl, const Matrix3d &rot);
  void SetExtrinsic(const Vector3d &transl);
  void SetExtrinsic(const Eigen::Matrix<double, (4), (4)> &T);
  void SetGyroscopeCovariance(const Vector3d &scaler);
  void SetAccelerometerCovariance(const Vector3d &scaler);
  void SetGyroscopeBiasCovariance(const Vector3d &b_g);
  void SetAccelerometerBiasCovariance(const Vector3d &b_a);
  void SetInverseExposureCovariance(const double &inv_expo);
  void SetInitializationFrames(const int &num);
  void DisableImu();
  void DisableGravityEstimation();
  void DisableBiasEstimation();
  void DisableExposureEstimation();
  void Process(SynchronizedMeasurements &lidar_meas, FilterState &stat,
               PointCloud::Ptr cur_pcl_un_);
  void UndistortCloud(SynchronizedMeasurements &lidar_meas,
                      FilterState &state_inout, PointCloud &pcl_out);

  std::vector<livo_atlas::ImuDiagnostic> diagnostics_;
  double IMU_mean_acc_norm;
  Vector3d unbiased_gyr;

  Vector3d cov_acc;
  Vector3d cov_gyr;
  Vector3d cov_bias_gyr;
  Vector3d cov_bias_acc;
  double cov_inv_expo;
  double first_lidar_time;
  bool imu_time_init = false;
  bool imu_need_init = true;
  Matrix3d Eye3d;
  Vector3d Zero3d;
  int lidar_type = kLivox;

 private:
  void InitializeImu(const UpdateMeasurements &meas, FilterState &state,
                     int &N);
  void PredictWithoutImu(SynchronizedMeasurements &meas,
                         FilterState &state_inout, PointCloud &pcl_out);
  PointCloud pcl_wait_proc;
  livo_atlas::Imu last_imu;
  PointCloud::Ptr cur_pcl_un_;
  std::vector<Pose6D> IMUpose;
  Matrix3d Lid_rot_to_IMU;
  Vector3d Lid_offset_to_IMU;
  Vector3d mean_acc;
  Vector3d mean_gyr;
  Vector3d angvel_last;
  Vector3d acc_s_last;
  double last_prop_end_time = 0.0;
  double time_last_scan;
  int init_iter_num = 1, MAX_INI_COUNT = 20;
  bool b_first_frame = true;
  bool imu_en = true;
  bool gravity_est_en = true;
  bool ba_bg_est_en = true;
  bool exposure_estimate_en = true;
};

}  // namespace livo_atlas

#endif
