/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "livo_atlas/frontend/imu/imu_processor.h"

#include <algorithm>
#include <cassert>
#include <ostream>

namespace livo_atlas {

namespace {
bool TimeLess(const Point &first, const Point &second) {
  return first.curvature < second.curvature;
}
}  // namespace

ImuProcessor::ImuProcessor()
    : Eye3d(Matrix3d::Identity()),
      Zero3d(0, 0, 0),
      b_first_frame(true),
      imu_need_init(true) {
  init_iter_num = 1;
  cov_acc = Vector3d(0.1, 0.1, 0.1);
  cov_gyr = Vector3d(0.1, 0.1, 0.1);
  cov_bias_gyr = Vector3d(0.1, 0.1, 0.1);
  cov_bias_acc = Vector3d(0.1, 0.1, 0.1);
  cov_inv_expo = 0.2;
  mean_acc = Vector3d(0, 0, -1.0);
  mean_gyr = Vector3d(0, 0, 0);
  angvel_last = Zero3d;
  acc_s_last = Zero3d;
  Lid_offset_to_IMU = Zero3d;
  Lid_rot_to_IMU = Eye3d;
  last_imu = {};
  cur_pcl_un_.reset(new PointCloud());
}

ImuProcessor::~ImuProcessor() {}

void ImuProcessor::Reset() {
  mean_acc = Vector3d(0, 0, -1.0);
  mean_gyr = Vector3d(0, 0, 0);
  angvel_last = Zero3d;
  imu_need_init = true;
  init_iter_num = 1;
  IMUpose.clear();
  last_imu = {};
  last_prop_end_time = 0.0;
  cur_pcl_un_.reset(new PointCloud());
}

void ImuProcessor::DisableImu() {
  imu_en = false;
  imu_need_init = false;
}

void ImuProcessor::DisableGravityEstimation() { gravity_est_en = false; }

void ImuProcessor::DisableBiasEstimation() { ba_bg_est_en = false; }

void ImuProcessor::DisableExposureEstimation() { exposure_estimate_en = false; }

void ImuProcessor::SetExtrinsic(const Eigen::Matrix<double, (4), (4)> &T) {
  Lid_offset_to_IMU = T.block<3, 1>(0, 3);
  Lid_rot_to_IMU = T.block<3, 3>(0, 0);
}

void ImuProcessor::SetExtrinsic(const Vector3d &transl) {
  Lid_offset_to_IMU = transl;
  Lid_rot_to_IMU.setIdentity();
}

void ImuProcessor::SetExtrinsic(const Vector3d &transl, const Matrix3d &rot) {
  Lid_offset_to_IMU = transl;
  Lid_rot_to_IMU = rot;
}

void ImuProcessor::SetGyroscopeCovariance(const Vector3d &scaler) {
  cov_gyr = scaler;
}

void ImuProcessor::SetAccelerometerCovariance(const Vector3d &scaler) {
  cov_acc = scaler;
}

void ImuProcessor::SetGyroscopeBiasCovariance(const Vector3d &b_g) {
  cov_bias_gyr = b_g;
}

void ImuProcessor::SetInverseExposureCovariance(const double &inv_expo) {
  cov_inv_expo = inv_expo;
}

void ImuProcessor::SetAccelerometerBiasCovariance(const Vector3d &b_a) {
  cov_bias_acc = b_a;
}

void ImuProcessor::SetInitializationFrames(const int &num) {
  MAX_INI_COUNT = num;
}

void ImuProcessor::InitializeImu(const UpdateMeasurements &meas,
                                 FilterState &state_inout, int &N) {
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  Vector3d cur_acc, cur_gyr;

  if (b_first_frame) {
    Reset();
    N = 1;
    b_first_frame = false;
    const auto &imu_acc = meas.imu.front().linear_acceleration;
    const auto &gyr_acc = meas.imu.front().angular_velocity;
    mean_acc << imu_acc.x(), imu_acc.y(), imu_acc.z();
    mean_gyr << gyr_acc.x(), gyr_acc.y(), gyr_acc.z();
    // first_lidar_time = meas.lidar_frame_beg_time;
  }

  for (const auto &imu : meas.imu) {
    const auto &imu_acc = imu.linear_acceleration;
    const auto &gyr_acc = imu.angular_velocity;
    cur_acc << imu_acc.x(), imu_acc.y(), imu_acc.z();
    cur_gyr << gyr_acc.x(), gyr_acc.y(), gyr_acc.z();

    mean_acc += (cur_acc - mean_acc) / N;
    mean_gyr += (cur_gyr - mean_gyr) / N;

    // cov_acc = cov_acc * (N - 1.0) / N + (cur_acc -
    // mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N); cov_gyr
    // = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr -
    // mean_gyr) * (N - 1.0) / (N * N);

    N++;
  }
  IMU_mean_acc_norm = mean_acc.norm();
  state_inout.gravity =
      -mean_acc / mean_acc.norm() * kGravityMetersPerSecondSquared;
  state_inout.rot_end =
      Eye3d;  // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
  state_inout.bias_g = Zero3d;  // mean_gyr;

  last_imu = meas.imu.back();
}

void ImuProcessor::PredictWithoutImu(SynchronizedMeasurements &meas,
                                     FilterState &state_inout,
                                     PointCloud &pcl_out) {
  pcl_out = *(meas.lidar_cloud);
  /*** sort point clouds by offset time ***/
  const double &pcl_beg_time = meas.scan_start_time;
  std::sort(pcl_out.points.begin(), pcl_out.points.end(), TimeLess);
  const double &pcl_end_time =
      pcl_beg_time + pcl_out.points.back().curvature / double(1000);
  meas.last_lidar_update_time = pcl_end_time;
  const double &pcl_end_offset_time =
      pcl_out.points.back().curvature / double(1000);

  Eigen::Matrix<double, (kStateDimension), (kStateDimension)> F_x, cov_w;
  double dt = 0;

  if (b_first_frame) {
    dt = 0.1;
    b_first_frame = false;
  } else {
    dt = pcl_beg_time - time_last_scan;
  }

  time_last_scan = pcl_beg_time;
  // for (size_t i = 0; i < pcl_out->points.size(); i++) {
  //   if (dt < pcl_out->points[i].curvature) {
  //     dt = pcl_out->points[i].curvature;
  //   }
  // }
  // dt = dt / (double)1000;
  // double dt = pcl_out->points.back().curvature / double(1000);

  /* covariance propagation */
  // M3D acc_avr_skew;
  Matrix3d Exp_f = Exp(state_inout.bias_g, dt);

  F_x.setIdentity();
  cov_w.setZero();

  F_x.block<3, 3>(0, 0) = Exp(state_inout.bias_g, -dt);
  F_x.block<3, 3>(0, 10) = Eye3d * dt;
  F_x.block<3, 3>(3, 7) = Eye3d * dt;
  // F_x.block<3, 3>(6, 0)  = - R_imu * acc_avr_skew * dt;
  // F_x.block<3, 3>(6, 12) = - R_imu * dt;
  // F_x.block<3, 3>(6, 15) = Eye3d * dt;

  cov_w.block<3, 3>(10, 10).diagonal() =
      cov_gyr * dt * dt;  // for omega in constant model
  cov_w.block<3, 3>(7, 7).diagonal() =
      cov_acc * dt * dt;  // for velocity in constant model
  // cov_w.block<3, 3>(6, 6) =
  //     R_imu * cov_acc.asDiagonal() * R_imu.transpose() * dt * dt;
  // cov_w.block<3, 3>(9, 9).diagonal() =
  //     cov_bias_gyr * dt * dt; // bias gyro covariance
  // cov_w.block<3, 3>(12, 12).diagonal() =
  //     cov_bias_acc * dt * dt; // bias acc covariance

  //           << std::endl;
  state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;
  //           << std::endl;
  state_inout.rot_end = state_inout.rot_end * Exp_f;
  state_inout.pos_end = state_inout.pos_end + state_inout.vel_end * dt;

  if (lidar_type != kDepthCamera) {
    auto it_pcl = pcl_out.points.end() - 1;
    double dt_j = 0.0;
    for (; it_pcl != pcl_out.points.begin(); it_pcl--) {
      dt_j = pcl_end_offset_time - it_pcl->curvature / double(1000);
      Matrix3d R_jk(Exp(state_inout.bias_g, -dt_j));
      Vector3d P_j(it_pcl->x, it_pcl->y, it_pcl->z);
      // Using rotation and translation to un-distort points
      Vector3d p_jk;
      p_jk = -state_inout.rot_end.transpose() * state_inout.vel_end * dt_j;

      Vector3d P_compensate = R_jk * P_j + p_jk;

      /// save Undistorted points and their rotation
      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);
    }
  }
}

void ImuProcessor::UndistortCloud(SynchronizedMeasurements &lidar_meas,
                                  FilterState &state_inout,
                                  PointCloud &pcl_out) {
  pcl_out.clear();
  /*** add the imu of the last frame-tail to the of current frame-head ***/
  UpdateMeasurements &meas = lidar_meas.updates.back();
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu);
  const double &imu_beg_time = v_imu.front().timestamp;
  const double &imu_end_time = v_imu.back().timestamp;
  const double prop_beg_time = last_prop_end_time;

  const double prop_end_time =
      lidar_meas.update_kind == UpdateKind::kLidarInertial
          ? meas.lidar_update_time
          : meas.visual_update_time;

  /*** cut lidar point based on the propagation-start time and required
   * propagation-end time ***/
  // const double pcl_offset_time = (prop_end_time -
  // lidar_meas.lidar_frame_beg_time) * 1000.; // the offset time w.r.t scan
  // start time auto pcl_it = lidar_meas.pcl_proc_cur->points.begin() +
  // lidar_meas.lidar_scan_index_now; auto pcl_it_end =
  // pcl_offset_time: %lf \n", pcl_it->curvature, pcl_offset_time); while
  // (pcl_it != pcl_it_end && pcl_it->curvature <= pcl_offset_time)
  // {
  //   pcl_wait_proc.push_back(*pcl_it);
  //   pcl_it++;
  //   lidar_meas.lidar_scan_index_now++;
  // }

  // "<<pcl_it->curvature<<endl;

  if (lidar_meas.update_kind == UpdateKind::kLidarInertial) {
    pcl_wait_proc.resize(lidar_meas.current_cloud->points.size());
    pcl_wait_proc = *(lidar_meas.current_cloud);
    lidar_meas.scan_cursor = 0;
    IMUpose.push_back(MakePose6D(0.0, acc_s_last, angvel_last,
                                 state_inout.vel_end, state_inout.pos_end,
                                 state_inout.rot_end));
  }

  // sort(pcl_out.points.begin(), pcl_out.points.end(), TimeLess);
  // lidar_meas.debug_show();
  // "<<prop_end_time<<", " \
  //          <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to
  //          "<<imu_end_time<<endl;

  /*** Initialize IMU pose ***/
  // IMUpose.clear();

  /*** forward propagation at each imu point ***/
  Vector3d acc_imu(acc_s_last), angvel_avr(angvel_last), acc_avr,
      vel_imu(state_inout.vel_end), pos_imu(state_inout.pos_end);
  Matrix3d R_imu(state_inout.rot_end);
  Eigen::Matrix<double, (kStateDimension), (kStateDimension)> F_x, cov_w;
  double dt, dt_all = 0.0;
  double offs_t;
  // double imu_time;
  double tau;
  if (!imu_time_init) {
    // imu_time = v_imu.front().timestamp - first_lidar_time;
    // tau = 1.0 / (0.25 * sin(2 * CV_PI * 0.5 * imu_time) + 0.75);
    tau = 1.0;
    imu_time_init = true;
  } else {
    tau = state_inout.inv_expo_time;
  }
  // state_inout.cov(6, 6) = 0.01;

  switch (lidar_meas.update_kind) {
    case UpdateKind::kLidarInertial:
    case UpdateKind::kVisual:
      dt = 0;
      for (int i = 0; i < v_imu.size() - 1; i++) {
        auto head = v_imu[i];
        auto tail = v_imu[i + 1];

        if (tail.timestamp < prop_beg_time) continue;

        angvel_avr << 0.5 * (head.angular_velocity.x() +
                             tail.angular_velocity.x()),
            0.5 * (head.angular_velocity.y() + tail.angular_velocity.y()),
            0.5 * (head.angular_velocity.z() + tail.angular_velocity.z());

        // angvel_avr<<tail.angular_velocity.x(), tail.angular_velocity.y(),
        // tail.angular_velocity.z();

        acc_avr << 0.5 * (head.linear_acceleration.x() +
                          tail.linear_acceleration.x()),
            0.5 * (head.linear_acceleration.y() + tail.linear_acceleration.y()),
            0.5 * (head.linear_acceleration.z() + tail.linear_acceleration.z());

        // #ifdef DEBUG_PRINT
        diagnostics_.push_back(
            {head.timestamp - first_lidar_time, angvel_avr, acc_avr});
        // #endif

        // imu_time = head.timestamp - first_lidar_time;

        angvel_avr -= state_inout.bias_g;
        acc_avr = acc_avr * kGravityMetersPerSecondSquared / mean_acc.norm() -
                  state_inout.bias_a;

        if (head.timestamp < prop_beg_time) {
          dt = tail.timestamp - last_prop_end_time;
          offs_t = tail.timestamp - prop_beg_time;
        } else if (i != v_imu.size() - 2) {
          dt = tail.timestamp - head.timestamp;
          offs_t = tail.timestamp - prop_beg_time;
        } else {
          dt = prop_end_time - head.timestamp;
          offs_t = prop_end_time - prop_beg_time;
        }

        dt_all += dt;

        /* covariance propagation */
        Matrix3d acc_avr_skew;
        Matrix3d Exp_f = Exp(angvel_avr, dt);
        acc_avr_skew << 0.0, -acc_avr[2], acc_avr[1], acc_avr[2], 0.0,
            -acc_avr[0], -acc_avr[1], acc_avr[0], 0.0;

        F_x.setIdentity();
        cov_w.setZero();

        F_x.block<3, 3>(0, 0) = Exp(angvel_avr, -dt);
        if (ba_bg_est_en) F_x.block<3, 3>(0, 10) = -Eye3d * dt;
        // F_x.block<3,3>(3,0)  = R_imu * off_vel_skew * dt;
        F_x.block<3, 3>(3, 7) = Eye3d * dt;
        F_x.block<3, 3>(7, 0) = -R_imu * acc_avr_skew * dt;
        if (ba_bg_est_en) F_x.block<3, 3>(7, 13) = -R_imu * dt;
        if (gravity_est_en) F_x.block<3, 3>(7, 16) = Eye3d * dt;

        // tau = 1.0 / (0.25 * sin(2 * CV_PI * 0.5 * imu_time) + 0.75);
        // F_x(6,6) = 0.25 * 2 * CV_PI * 0.5 * cos(2 * CV_PI * 0.5 * imu_time) *
        // (-tau*tau); F_x(18,18) = 0.00001;
        if (exposure_estimate_en) cov_w(6, 6) = cov_inv_expo * dt * dt;
        cov_w.block<3, 3>(0, 0).diagonal() = cov_gyr * dt * dt;
        cov_w.block<3, 3>(7, 7) =
            R_imu * cov_acc.asDiagonal() * R_imu.transpose() * dt * dt;
        cov_w.block<3, 3>(10, 10).diagonal() =
            cov_bias_gyr * dt * dt;  // bias gyro covariance
        cov_w.block<3, 3>(13, 13).diagonal() =
            cov_bias_acc * dt * dt;  // bias acc covariance

        state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;
        // state_inout.cov.block<18,18>(0,0) = F_x.block<18,18>(0,0) *
        // state_inout.cov.block<18,18>(0,0) * F_x.block<18,18>(0,0).transpose()
        // + cov_w.block<18,18>(0,0);

        // tau = tau + 0.25 * 2 * CV_PI * 0.5 * cos(2 * CV_PI * 0.5 * imu_time)
        // *
        // (-tau*tau) * dt;

        // tau = 1.0 / (0.25 * sin(2 * CV_PI * 0.5 * imu_time) + 0.75);

        /* propogation of IMU attitude */
        R_imu = R_imu * Exp_f;

        /* Specific acceleration (global frame) of IMU */
        acc_imu = R_imu * acc_avr + state_inout.gravity;

        /* propogation of IMU */
        pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;

        /* velocity of IMU */
        vel_imu = vel_imu + acc_imu * dt;

        /* save the poses at each IMU measurements */
        angvel_last = angvel_avr;
        acc_s_last = acc_imu;

        // offs_t: %lf \n", offs_t);
        IMUpose.push_back(
            MakePose6D(offs_t, acc_imu, angvel_avr, vel_imu, pos_imu, R_imu));
      }

      // unbiased_gyr = V3D(IMUpose.back().gyr[0], IMUpose.back().gyr[1],
      // IMUpose.back().gyr[2]);
      lidar_meas.last_lidar_update_time = prop_end_time;
      // dt = prop_end_time - imu_end_time;
      break;
  }

  state_inout.vel_end = vel_imu;
  state_inout.rot_end = R_imu;
  state_inout.pos_end = pos_imu;
  state_inout.inv_expo_time = tau;

  /*** calculated the pos and attitude prediction at the frame-end ***/
  // if (imu_end_time>prop_beg_time)
  // {
  //   double note = prop_end_time > imu_end_time ? 1.0 : -1.0;
  //   dt = note * (prop_end_time - imu_end_time);
  //   state_inout.vel_end = vel_imu + note * acc_imu * dt;
  //   state_inout.rot_end = R_imu * Exp(V3D(note * angvel_avr), dt);
  //   state_inout.pos_end = pos_imu + note * vel_imu * dt + note * 0.5 *
  //   acc_imu * dt * dt;
  // }
  // else
  // {
  //   double note = prop_end_time > prop_beg_time ? 1.0 : -1.0;
  //   dt = note * (prop_end_time - prop_beg_time);
  //   state_inout.vel_end = vel_imu + note * acc_imu * dt;
  //   state_inout.rot_end = R_imu * Exp(V3D(note * angvel_avr), dt);
  //   state_inout.pos_end = pos_imu + note * vel_imu * dt + note * 0.5 *
  //   acc_imu * dt * dt;
  // }

  // state_inout.pos_end.transpose()<<endl;

  last_imu = v_imu.back();
  last_prop_end_time = prop_end_time;

  // auto pos_liD_e = state_inout.pos_end + state_inout.rot_end *
  // Lid_offset_to_IMU; auto R_liD_e   = state_inout.rot_end * Lidar_R_to_IMU;

  // "<<state_inout.pos_end.transpose()<<"
  // ba"<<state_inout.bias_a.transpose()<<" bg
  // "<<state_inout.cov.diagonal().transpose()<<endl;

  //   for (auto it = IMUpose.begin(); it != IMUpose.end(); ++it) {
  //   }
  //          <<"lidar_meas.size:
  //          "<<lidar_meas.lidar_cloud->points.size()<<endl;
  if (pcl_wait_proc.points.size() < 1) return;

  /*** undistort each lidar point (backward propagation), ONLY working for LIO
   * update ***/
  if (lidar_meas.update_kind == UpdateKind::kLidarInertial) {
    auto it_pcl = pcl_wait_proc.points.end() - 1;
    Matrix3d extR_Ri(Lid_rot_to_IMU.transpose() *
                     state_inout.rot_end.transpose());
    Vector3d exrR_extT(Lid_rot_to_IMU.transpose() * Lid_offset_to_IMU);
    for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--) {
      auto head = it_kp - 1;
      auto tail = it_kp;
      R_imu << head->rot[0], head->rot[1], head->rot[2], head->rot[3],
          head->rot[4], head->rot[5], head->rot[6], head->rot[7], head->rot[8];
      acc_imu << head->acc[0], head->acc[1], head->acc[2];
      vel_imu << head->vel[0], head->vel[1], head->vel[2];
      pos_imu << head->pos[0], head->pos[1], head->pos[2];
      angvel_avr << head->gyr[0], head->gyr[1], head->gyr[2];

      // it_pcl->curvature / double(1000) - head->offset_time);

      for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--) {
        dt = it_pcl->curvature / double(1000) - head->offset_time;

        /* Transform to the 'end' frame */
        Matrix3d R_i(R_imu * Exp(angvel_avr, dt));
        Vector3d T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt -
                      state_inout.pos_end);

        Vector3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
        // V3D P_compensate = Lid_rot_to_IMU.transpose() *
        // (state_inout.rot_end.transpose() * (R_i * (Lid_rot_to_IMU * P_i +
        // Lid_offset_to_IMU) + T_ei) - Lid_offset_to_IMU);
        Vector3d P_compensate =
            (extR_Ri *
                 (R_i * (Lid_rot_to_IMU * P_i + Lid_offset_to_IMU) + T_ei) -
             exrR_extT);

        /// save Undistorted points and their rotation
        it_pcl->x = P_compensate(0);
        it_pcl->y = P_compensate(1);
        it_pcl->z = P_compensate(2);

        if (it_pcl == pcl_wait_proc.points.begin()) break;
      }
    }
    pcl_out = pcl_wait_proc;
    pcl_wait_proc.clear();
    IMUpose.clear();
  }
}

void ImuProcessor::Process(SynchronizedMeasurements &lidar_meas,
                           FilterState &stat, PointCloud::Ptr cur_pcl_un_) {
  diagnostics_.clear();
  assert(lidar_meas.lidar_cloud != nullptr);
  if (!imu_en) {
    PredictWithoutImu(lidar_meas, stat, *cur_pcl_un_);
    return;
  }

  UpdateMeasurements meas = lidar_meas.updates.back();

  if (imu_need_init) {
    double pcl_end_time = lidar_meas.update_kind == UpdateKind::kLidarInertial
                              ? meas.lidar_update_time
                              : meas.visual_update_time;
    // lidar_meas.last_lio_update_time = pcl_end_time;

    if (meas.imu.empty()) {
      return;
    };
    /// The very first lidar frame
    InitializeImu(meas, stat, init_iter_num);

    imu_need_init = true;

    last_imu = meas.imu.back();

    if (init_iter_num > MAX_INI_COUNT) {
      // cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
      imu_need_init = false;
    }

    return;
  }

  UndistortCloud(lidar_meas, stat, *cur_pcl_un_);
}

}  // namespace livo_atlas
