#include "livo_atlas/ros1/measurement_conversions.h"

#include <cstdint>
#include <limits>
#include <utility>

#include <cv_bridge/cv_bridge.h>
#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>

namespace livo_atlas::ros1 {
namespace {

template <typename PointType>
std::string ValidateCloudLayout(const sensor_msgs::PointCloud2& message) {
  if (message.is_bigendian) {
    return "PointCloud2: big-endian data is unsupported";
  }
  const std::uint64_t count =
      std::uint64_t{message.width} * message.height;
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
    const sensor_msgs::PointField* matched = nullptr;
    for (const auto& field : message.fields) {
      const bool color_alias = expected.name == "rgb" && field.name == "rgba";
      if (field.name != expected.name && !color_alias) continue;
      if (matched != nullptr) {
        return "PointCloud2: ambiguous field " + expected.name;
      }
      matched = &field;
    }
    if (matched == nullptr) return "PointCloud2: missing field " + expected.name;
    // PCL also permits packed color as uint32, including an rgba field name.
    const bool packed_color = expected.name == "rgb" &&
                              matched->datatype == sensor_msgs::PointField::UINT32;
    if ((!packed_color && matched->datatype != expected.datatype) ||
        matched->count != 1 ||
        std::uint64_t{matched->offset} + pcl::getFieldSize(expected.datatype) >
            message.point_step) {
      return "PointCloud2: invalid type, count or offset for " + expected.name;
    }
  }
  return {};
}

template <typename PointType>
std::string DecodeCloud(const sensor_msgs::PointCloud2& message,
                        double time_offset, LidarScan* output) {
  const std::string error = ValidateCloudLayout<PointType>(message);
  if (!error.empty()) return error;
  pcl::PointCloud<PointType> points;
  // PCL 1.10 indexes points[0] even when its input is empty.
  if (message.width != 0 && message.height != 0) {
    pcl::fromROSMsg(message, points);
  }
  *output = LidarScan{message.header.stamp.toSec() + time_offset,
                      std::move(points)};
  return {};
}

}  // namespace

ImuMeasurement ConvertImu(const sensor_msgs::Imu& message) {
  return {message.header.stamp.toSec(),
          Eigen::Vector3d(message.angular_velocity.x, message.angular_velocity.y,
                          message.angular_velocity.z),
          Eigen::Vector3d(message.linear_acceleration.x,
                          message.linear_acceleration.y,
                          message.linear_acceleration.z)};
}

std::string ConvertImage(const sensor_msgs::Image& message, double time_offset,
                         ImageMeasurement* output) {
  if (output == nullptr) return "Image: null output";
  if (message.width == 0 || message.height == 0 ||
      message.width > std::numeric_limits<int>::max() / 3 ||
      message.height > std::numeric_limits<int>::max() ||
      std::uint64_t{message.height} * message.step != message.data.size()) {
    return "Image: invalid dimensions, stride or data size";
  }
  try {
    cv::Mat image = cv_bridge::toCvCopy(message, "bgr8")->image;
    *output = ImageMeasurement{message.header.stamp.toSec() + time_offset,
                               std::move(image)};
    return {};
  } catch (const cv_bridge::Exception& error) {
    return std::string("Image: ") + error.what();
  } catch (const cv::Exception& error) {
    return std::string("Image: ") + error.what();
  }
}

std::string ConvertLivox(const livox_ros_driver::CustomMsg& message,
                         LidarScan* output) {
  if (output == nullptr) return "Livox: null output";
  if (message.point_num != message.points.size()) {
    return "Livox: point_num does not match points.size()";
  }
  std::vector<LivoxPoint> points;
  points.reserve(message.points.size());
  for (const auto& point : message.points) {
    points.push_back({point.x, point.y, point.z, point.offset_time,
                      point.reflectivity, point.tag, point.line});
  }
  *output = LidarScan{message.header.stamp.toSec(), std::move(points)};
  return {};
}

std::string ConvertPointCloud(const sensor_msgs::PointCloud2& message,
                             int lidar_type, double time_offset,
                             LidarScan* output) {
  if (output == nullptr) return "PointCloud2: null output";
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
      return "PointCloud2: unsupported lidar_type " + std::to_string(lidar_type);
  }
}

}  // namespace livo_atlas::ros1
