#include "livo_atlas/frontend/imu_propagation.h"

namespace livo_atlas {
void PropagateImu(FilterState* state, double dt, Eigen::Vector3d acceleration,
                  Eigen::Vector3d angular_velocity,
                  double mean_acceleration_norm) {
  acceleration =
      acceleration * kGravityMetersPerSecondSquared / mean_acceleration_norm -
      state->bias_a;
  angular_velocity -= state->bias_g;
  const Eigen::Matrix3d increment = Exp(angular_velocity, dt);
  state->rot_end = state->rot_end * increment;
  const Eigen::Vector3d world_acceleration =
      state->rot_end * acceleration +
      Eigen::Vector3d(state->gravity[0], state->gravity[1], state->gravity[2]);
  state->pos_end =
      state->pos_end + state->vel_end * dt + 0.5 * world_acceleration * dt * dt;
  state->vel_end = state->vel_end + world_acceleration * dt;
}
}  // namespace livo_atlas
