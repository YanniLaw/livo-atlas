#ifndef LIVO_ATLAS_FRONTEND_IMU_POSE6D_H_
#define LIVO_ATLAS_FRONTEND_IMU_POSE6D_H_

#include <Eigen/Core>
#include <utility>

namespace livo_atlas {

struct Pose6D {
  /*** the preintegrated Lidar states at the time of IMU measurements in a frame
   * ***/
  double offset_time;  // the offset time of IMU measurement w.r.t the first
                       // lidar point
  double acc[3];  // the preintegrated total acceleration (global frame) at the
                  // Lidar origin
  double
      gyr[3];  // the unbiased angular velocity (body frame) at the Lidar origin
  double
      vel[3];  // the preintegrated velocity (global frame) at the Lidar origin
  double
      pos[3];  // the preintegrated position (global frame) at the Lidar origin
  double
      rot[9];  // the preintegrated rotation (global frame) at the Lidar origin
};

template <typename T>
auto MakePose6D(const double t, const Eigen::Matrix<T, 3, 1> &a,
                const Eigen::Matrix<T, 3, 1> &g,
                const Eigen::Matrix<T, 3, 1> &v,
                const Eigen::Matrix<T, 3, 1> &p,
                const Eigen::Matrix<T, 3, 3> &R) {
  Pose6D rot_kp;
  rot_kp.offset_time = t;
  for (int i = 0; i < 3; i++) {
    rot_kp.acc[i] = a(i);
    rot_kp.gyr[i] = g(i);
    rot_kp.vel[i] = v(i);
    rot_kp.pos[i] = p(i);
    for (int j = 0; j < 3; j++) rot_kp.rot[i * 3 + j] = R(i, j);
  }
  // Map<M3D>(rot_kp.rot, 3,3) = R;
  return std::move(rot_kp);
}

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_FRONTEND_IMU_POSE6D_H_
