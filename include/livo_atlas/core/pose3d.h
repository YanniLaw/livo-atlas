#ifndef LIVO_ATLAS_INCLUDE_LIVO_ATLAS_CORE_POSE3D_H_
#define LIVO_ATLAS_INCLUDE_LIVO_ATLAS_CORE_POSE3D_H_

#include <Eigen/Geometry>

namespace livo_atlas {

// An SE(3) transform represented by Eigen. A pose named T_A_B maps points
// from frame B into frame A, with translation in meters and rotation in
// radians. T_A_B * T_B_C composes to T_A_C, and T_A_B.inverse() is T_B_A.
// Eigen does not initialize the transform to identity; use Pose3d::Identity()
// when an identity pose is required.
using Pose3d = Eigen::Isometry3d;

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_INCLUDE_LIVO_ATLAS_CORE_POSE3D_H_
