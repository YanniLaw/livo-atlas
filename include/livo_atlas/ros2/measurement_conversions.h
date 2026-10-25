#ifndef LIVO_ATLAS_ROS2_MEASUREMENT_CONVERSIONS_H_
#define LIVO_ATLAS_ROS2_MEASUREMENT_CONVERSIONS_H_

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

#include "livo_atlas/sensor/lidar_scan.h"
#include "livo_atlas/sensor/measurements.h"

namespace livo_atlas::ros2 {

// Throw std::invalid_argument for invalid nanoseconds or nonfinite/out-of-range
// time.
double ToSeconds(const builtin_interfaces::msg::Time& stamp);
builtin_interfaces::msg::Time FromSeconds(double seconds);

// Copies the supplied sensor time. Rejects non-finite vectors with
// std::invalid_argument; the caller controls offset/driver correction policy.
Imu ConvertImu(const sensor_msgs::msg::Imu& message);

// Empty error means success. On failure, *output is unchanged.
std::string ConvertImage(const sensor_msgs::msg::Image& message,
                         double time_offset, TimedImage* output);
std::string ConvertLivox(const livox_ros_driver2::msg::CustomMsg& message,
                         LidarScan* output);
std::string ConvertPointCloud(const sensor_msgs::msg::PointCloud2& message,
                              int lidar_type, double time_offset,
                              LidarScan* output);

}  // namespace livo_atlas::ros2

#endif  // LIVO_ATLAS_ROS2_MEASUREMENT_CONVERSIONS_H_
