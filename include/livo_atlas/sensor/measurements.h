#ifndef LIVO_ATLAS_SENSOR_MEASUREMENTS_H_
#define LIVO_ATLAS_SENSOR_MEASUREMENTS_H_

#include <Eigen/Core>
#include <opencv2/core/mat.hpp>

#include "livo_atlas/sensor/point_types.h"

namespace livo_atlas {

// Sensor header time in seconds after adapter-specific time correction.
// Acceleration retains the input units; IMU initialization normalizes it.
struct Imu {
  double timestamp = 0.0;
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
};

// Owns BGR8 pixels independently of the incoming message. Mat copies share
// storage; the sequential frontend is the only writer after enqueueing.
struct TimedImage {
  double timestamp = 0.0;
  cv::Mat bgr;
};

// The adapter transfers a separate cloud to the sequential frontend. Curvature
// stores the offset from timestamp in milliseconds, not surface curvature.
struct TimedPointCloud {
  double timestamp = 0.0;
  PointCloud::Ptr cloud;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_SENSOR_MEASUREMENTS_H_
