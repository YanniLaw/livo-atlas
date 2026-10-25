#ifndef LIVO_ATLAS_FRONTEND_CALIBRATION_H_
#define LIVO_ATLAS_FRONTEND_CALIBRATION_H_

#include "livo_atlas/camera/camera_config.h"
#include "livo_atlas/sensor/extrinsics.h"

namespace livo_atlas {

struct Calibration {
  RigidTransform imu_from_lidar;     // T_I_L: extrinsic_R / extrinsic_T.
  RigidTransform camera_from_lidar;  // T_C_L: Rcl / Pcl.
  CameraConfig camera;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_FRONTEND_CALIBRATION_H_
