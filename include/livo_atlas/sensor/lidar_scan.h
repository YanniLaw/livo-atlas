#ifndef LIVO_ATLAS_SENSOR_LIDAR_SCAN_H_
#define LIVO_ATLAS_SENSOR_LIDAR_SCAN_H_

#include <cstdint>
#include <variant>
#include <vector>

#include "livo_atlas/sensor/point_types.h"

namespace livo_atlas {

struct LivoxPoint {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  std::uint32_t offset_time = 0;  // Nanoseconds from the sensor header time.
  std::uint8_t reflectivity = 0;
  std::uint8_t tag = 0;
  std::uint8_t line = 0;
};

// Preserve driver field precision and point order. Unit conversion, filtering
// and sorting belong to preprocessing, not message decoding.
using LidarPoints = std::variant<
    std::vector<LivoxPoint>, pcl::PointCloud<VelodynePoint>,
    pcl::PointCloud<OusterPoint>, pcl::PointCloud<pcl::PointXYZRGB>,
    pcl::PointCloud<Xt32Point>, pcl::PointCloud<Pandar128Point>,
    pcl::PointCloud<RobosensePoint>>;

struct LidarScan {
  double timestamp = 0.0;  // Sensor header time in seconds, not bag receipt time.
  LidarPoints points;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_SENSOR_LIDAR_SCAN_H_
