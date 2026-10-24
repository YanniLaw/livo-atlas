#include <cv_bridge/cv_bridge.h>
#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <type_traits>
#include <utility>

#include "livo_atlas/ros2/measurement_conversions.h"

namespace livo_atlas::ros2 {
namespace {

template <typename PointType>
bool IsOptionalField(std::string_view name) {
  // Match legacy PCL zero defaults, including single-line input without ring.
  if (name == "ring") return true;
  if constexpr (std::is_same_v<PointType, VelodynePoint>) {
    return name == "time";  // Preprocess estimates timing from yaw.
  } else if constexpr (std::is_same_v<PointType, Xt32Point>) {
    return name == "timestamp";  // The feature branch has a yaw fallback.
  } else if constexpr (std::is_same_v<PointType, OusterPoint>) {
    return name == "reflectivity" || name == "ambient" || name == "range";
  }
  return false;
}

template <typename PointType>
std::string ValidateCloudLayout(const sensor_msgs::msg::PointCloud2& message) {
  if (message.is_bigendian) {
    return "PointCloud2: big-endian data is unsupported";
  }
  const std::uint64_t count = std::uint64_t{message.width} * message.height;
  const std::uint64_t row_bytes =
      std::uint64_t{message.width} * message.point_step;
  const std::uint64_t data_bytes =
      std::uint64_t{message.row_step} * message.height;
  // PCL 1.10 uses uint32_t for the point count and can memcpy data.size()
  // bytes into the destination. Require the complete declared buffer exactly.
  if (count > std::numeric_limits<std::uint32_t>::max() ||
      message.row_step < row_bytes || data_bytes != message.data.size() ||
      (message.width != 0 && message.height == 0)) {
    return "PointCloud2: inconsistent dimensions, stride or data size";
  }
  if (count == 0) return {};
  if (message.point_step == 0) return "PointCloud2: zero point_step";

  const auto expected_fields = pcl::getFields<PointType>();
  for (const auto& expected : expected_fields) {
    const sensor_msgs::msg::PointField* matched = nullptr;
    for (const auto& field : message.fields) {
      const bool color_alias = expected.name == "rgb" && field.name == "rgba";
      if (field.name != expected.name && !color_alias) continue;
      if (matched != nullptr) {
        return "PointCloud2: ambiguous field " + expected.name;
      }
      matched = &field;
    }
    if (matched == nullptr) {
      if (IsOptionalField<PointType>(expected.name)) continue;
      return "PointCloud2: missing field " + expected.name;
    }
    // PCL accepts rgb/FLOAT32, rgb/UINT32 and rgba/UINT32.
    bool valid_type = matched->datatype == expected.datatype;
    if (expected.name == "rgb") {
      valid_type = matched->datatype == sensor_msgs::msg::PointField::UINT32 ||
                   (matched->name == "rgb" &&
                    matched->datatype == sensor_msgs::msg::PointField::FLOAT32);
    }
    if (!valid_type || matched->count != 1 ||
        std::uint64_t{matched->offset} + pcl::getFieldSize(expected.datatype) >
            message.point_step) {
      return "PointCloud2: invalid type, count or offset for " + expected.name;
    }
  }
  return {};
}

template <typename PointType>
std::string DecodeCloud(const sensor_msgs::msg::PointCloud2& message,
                        double time_offset, LidarScan* output) {
  const std::string error = ValidateCloudLayout<PointType>(message);
  if (!error.empty()) return error;
  pcl::PointCloud<PointType> points;
  // PCL 1.10 indexes points[0] even when its input is empty.
  if (message.width != 0 && message.height != 0) {
    pcl::fromROSMsg(message, points);
  }
  *output = LidarScan{ToSeconds(message.header.stamp) + time_offset,
                      std::move(points)};
  return {};
}

}  // namespace

double ToSeconds(const builtin_interfaces::msg::Time& stamp) {
  if (stamp.nanosec >= 1000000000u) {
    throw std::invalid_argument(
        "timestamp nanoseconds must be below 1000000000");
  }
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1e-9;
}

builtin_interfaces::msg::Time FromSeconds(double seconds) {
  constexpr auto kMin = std::numeric_limits<std::int32_t>::min();
  constexpr auto kMax = std::numeric_limits<std::int32_t>::max();
  if (!std::isfinite(seconds) || seconds < kMin ||
      seconds >= double{kMax} + 1) {
    throw std::invalid_argument("timestamp is outside the ROS2 seconds range");
  }
  const double whole = std::floor(seconds);
  auto sec = static_cast<std::int64_t>(whole);
  auto nanosec = std::llround((seconds - whole) * 1e9);
  if (nanosec == 1000000000) {
    ++sec;
    nanosec = 0;
  }
  if (sec > kMax) {
    throw std::invalid_argument(
        "rounded timestamp is outside the ROS2 seconds range");
  }
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(sec);
  stamp.nanosec = static_cast<std::uint32_t>(nanosec);
  return stamp;
}

ImuMeasurement ConvertImu(const sensor_msgs::msg::Imu& message) {
  ImuMeasurement measurement{
      ToSeconds(message.header.stamp),
      Eigen::Vector3d(message.angular_velocity.x, message.angular_velocity.y,
                      message.angular_velocity.z),
      Eigen::Vector3d(message.linear_acceleration.x,
                      message.linear_acceleration.y,
                      message.linear_acceleration.z)};
  if (!measurement.linear_acceleration.allFinite() ||
      !measurement.angular_velocity.allFinite()) {
    throw std::invalid_argument(
        "IMU acceleration and angular velocity must be finite");
  }
  return measurement;
}

std::string ConvertImage(const sensor_msgs::msg::Image& message,
                         double time_offset, ImageMeasurement* output) {
  if (output == nullptr) return "Image: null output";
  if (message.header.stamp.nanosec >= 1000000000u)
    return "Image: invalid timestamp";
  if (message.width == 0 || message.height == 0 ||
      message.width > std::numeric_limits<int>::max() / 3 ||
      message.height > std::numeric_limits<int>::max() ||
      std::uint64_t{message.height} * message.step != message.data.size()) {
    return "Image: invalid dimensions, stride or data size";
  }
  try {
    cv::Mat image = cv_bridge::toCvCopy(message, "bgr8")->image;
    *output = ImageMeasurement{ToSeconds(message.header.stamp) + time_offset,
                               std::move(image)};
    return {};
  } catch (const cv_bridge::Exception& error) {
    return std::string("Image: ") + error.what();
  } catch (const cv::Exception& error) {
    return std::string("Image: ") + error.what();
  }
}

std::string ConvertLivox(const livox_ros_driver2::msg::CustomMsg& message,
                         LidarScan* output) {
  if (output == nullptr) return "Livox: null output";
  if (message.header.stamp.nanosec >= 1000000000u)
    return "Livox: invalid timestamp";
  if (message.point_num != message.points.size()) {
    return "Livox: point_num does not match points.size()";
  }
  std::vector<LivoxPoint> points;
  points.reserve(message.points.size());
  for (const auto& point : message.points) {
    points.push_back({point.x, point.y, point.z, point.offset_time,
                      point.reflectivity, point.tag, point.line});
  }
  *output = LidarScan{ToSeconds(message.header.stamp), std::move(points)};
  return {};
}

std::string ConvertPointCloud(const sensor_msgs::msg::PointCloud2& message,
                              int lidar_type, double time_offset,
                              LidarScan* output) {
  if (output == nullptr) return "PointCloud2: null output";
  if (message.header.stamp.nanosec >= 1000000000u)
    return "PointCloud2: invalid timestamp";
  // Keep the existing preprocess/lidar_type parameter values.
  switch (lidar_type) {
    case kVelodyne:
      return DecodeCloud<VelodynePoint>(message, time_offset, output);
    case kOuster:
      return DecodeCloud<OusterPoint>(message, time_offset, output);
    case kDepthCamera:
      return DecodeCloud<pcl::PointXYZRGB>(message, time_offset, output);
    case kXt32:
      return DecodeCloud<Xt32Point>(message, time_offset, output);
    case kPandar128:
      return DecodeCloud<Pandar128Point>(message, time_offset, output);
    case kRobosense:
      return DecodeCloud<RobosensePoint>(message, time_offset, output);
    default:
      return "PointCloud2: unsupported lidar_type " +
             std::to_string(lidar_type);
  }
}

}  // namespace livo_atlas::ros2
