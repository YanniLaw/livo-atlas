#ifndef LIVO_ATLAS_SENSOR_POINT_TYPES_H_
#define LIVO_ATLAS_SENSOR_POINT_TYPES_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

#include <cstdint>

namespace livo_atlas {
// Values of the existing preprocess/lidar_type parameter.
enum LidarType {
  kLivox = 1,
  kVelodyne = 2,
  kOuster = 3,
  kDepthCamera = 4,
  kXt32 = 5,
  kPandar128 = 6,
  kRobosense = 7
};

using Point = pcl::PointXYZINormal;
using PointCloud = pcl::PointCloud<Point>;
using ColoredCloud = pcl::PointCloud<pcl::PointXYZRGB>;
}  // namespace livo_atlas

// Driver field layouts. Time units follow the original preprocessing branches;
// see docs/p1-03-validation.md before adapting another driver.
namespace livo_atlas {
struct EIGEN_ALIGN16 VelodynePoint {
  PCL_ADD_POINT4D;
  float intensity = 0;
  float time = 0;
  std::uint16_t ring = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace livo_atlas
POINT_CLOUD_REGISTER_POINT_STRUCT(
    livo_atlas::VelodynePoint,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        float, time, time)(std::uint16_t, ring, ring))

namespace livo_atlas {
struct EIGEN_ALIGN16 OusterPoint {
  PCL_ADD_POINT4D;
  float intensity = 0;
  std::uint32_t t = 0;
  std::uint16_t reflectivity = 0;
  std::uint8_t ring = 0;
  std::uint16_t ambient = 0;
  std::uint32_t range = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace livo_atlas
POINT_CLOUD_REGISTER_POINT_STRUCT(
    livo_atlas::OusterPoint,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        std::uint32_t, t, t)(std::uint16_t, reflectivity, reflectivity)(
        std::uint8_t, ring, ring)(std::uint16_t, ambient,
                                  ambient)(std::uint32_t, range, range))

namespace livo_atlas {
struct EIGEN_ALIGN16 Xt32Point {
  PCL_ADD_POINT4D;
  float intensity = 0;
  double timestamp = 0;
  std::uint16_t ring = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace livo_atlas
POINT_CLOUD_REGISTER_POINT_STRUCT(
    livo_atlas::Xt32Point,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        double, timestamp, timestamp)(std::uint16_t, ring, ring))

namespace livo_atlas {
struct EIGEN_ALIGN16 Pandar128Point {
  PCL_ADD_POINT4D;
  std::uint8_t intensity = 0;
  double timestamp = 0;
  std::uint16_t ring = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace livo_atlas
POINT_CLOUD_REGISTER_POINT_STRUCT(
    livo_atlas::Pandar128Point,
    (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)(
        double, timestamp, timestamp)(std::uint16_t, ring, ring))

namespace livo_atlas {
struct EIGEN_ALIGN16 RobosensePoint {
  PCL_ADD_POINT4D;
  float intensity = 0;
  double timestamp = 0;
  std::uint16_t ring = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace livo_atlas
POINT_CLOUD_REGISTER_POINT_STRUCT(
    livo_atlas::RobosensePoint,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        double, timestamp, timestamp)(std::uint16_t, ring, ring))

#endif  // LIVO_ATLAS_SENSOR_POINT_TYPES_H_
