#ifndef TYPES_H
#define TYPES_H

#include <vector>

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "livo_atlas/core/math_types.h"
#include "livo_atlas/sensor/point_types.h"

using PointType = livo_atlas::Point;
typedef pcl::PointXYZRGB PointTypeRGB;
typedef pcl::PointXYZRGBA PointTypeRGBA;
using PointCloudXYZI = livo_atlas::PointCloud;
typedef std::vector<PointType, Eigen::aligned_allocator<PointType>> PointVector;
typedef pcl::PointCloud<PointTypeRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<PointTypeRGBA> PointCloudXYZRGBA;

using V2F = livo_atlas::Vector2f;
using V2D = livo_atlas::Vector2d;
using V3D = livo_atlas::Vector3d;
using M3D = livo_atlas::Matrix3d;
using V3F = livo_atlas::Vector3f;
using M3F = livo_atlas::Matrix3f;

#define MD(a, b) Eigen::Matrix<double, (a), (b)>
#define VD(a) Eigen::Matrix<double, (a), 1>
#define MF(a, b) Eigen::Matrix<float, (a), (b)>
#define VF(a) Eigen::Matrix<float, (a), 1>

struct Pose6D
{
  /*** the preintegrated Lidar states at the time of IMU measurements in a frame ***/
  double offset_time; // the offset time of IMU measurement w.r.t the first lidar point
  double acc[3];      // the preintegrated total acceleration (global frame) at the Lidar origin
  double gyr[3];      // the unbiased angular velocity (body frame) at the Lidar origin
  double vel[3];      // the preintegrated velocity (global frame) at the Lidar origin
  double pos[3];      // the preintegrated position (global frame) at the Lidar origin
  double rot[9];      // the preintegrated rotation (global frame) at the Lidar origin
};

#endif
