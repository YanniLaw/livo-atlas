#ifndef LIVO_ATLAS_FRONTEND_IMU_PROPAGATION_H_
#define LIVO_ATLAS_FRONTEND_IMU_PROPAGATION_H_

#include "livo_atlas/core/state.h"

namespace livo_atlas {
// The caller supplies a nonnull state, finite dt and positive initialized norm.
void PropagateImu(State* state, double dt, Eigen::Vector3d acceleration,
                  Eigen::Vector3d angular_velocity,
                  double mean_acceleration_norm);
}  // namespace livo_atlas

#endif  // LIVO_ATLAS_FRONTEND_IMU_PROPAGATION_H_
