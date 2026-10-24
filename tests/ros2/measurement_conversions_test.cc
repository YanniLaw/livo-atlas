#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "livo_atlas/ros2/measurement_conversions.h"
#include "livo_atlas/sensor/preprocess.h"

namespace livo_atlas::ros2 {
namespace {

template <typename PointType>
sensor_msgs::msg::PointCloud2 MakeCloudMessage(
    const pcl::PointCloud<PointType>& cloud) {
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header.stamp = FromSeconds(17.123456789);
  return message;
}

template <typename PointType>
sensor_msgs::msg::PointCloud2 MakeCloudMessage(const PointType& point) {
  pcl::PointCloud<PointType> cloud;
  cloud.push_back(point);
  return MakeCloudMessage(cloud);
}

void RemoveField(const std::string& name,
                 sensor_msgs::msg::PointCloud2* message) {
  auto& fields = message->fields;
  fields.erase(
      std::remove_if(fields.begin(), fields.end(),
                     [&name](const auto& field) { return field.name == name; }),
      fields.end());
}

TEST(MeasurementConversionsTest, ImuPreservesCorrectedTimeAndRawValues) {
  sensor_msgs::msg::Imu message;
  message.header.stamp = FromSeconds(17.123456789);
  message.angular_velocity.x = 1.25;
  message.angular_velocity.y = -2.5;
  message.angular_velocity.z = 3.75;
  message.linear_acceleration.x = -0.25;
  message.linear_acceleration.y = 0.5;
  message.linear_acceleration.z = 1.0;
  // Same ROS Time round trips as the adapter, including driver compensation.
  message.header.stamp = FromSeconds(ToSeconds(message.header.stamp) - 0.03);
  message.header.stamp = FromSeconds(ToSeconds(message.header.stamp) + 983.0);
  const auto result = ConvertImu(message);
  EXPECT_DOUBLE_EQ(result.timestamp, 1000.093456789);
  EXPECT_TRUE(
      result.angular_velocity.isApprox(Eigen::Vector3d(1.25, -2.5, 3.75), 0));
  EXPECT_TRUE(
      result.linear_acceleration.isApprox(Eigen::Vector3d(-0.25, 0.5, 1), 0));
  message.linear_acceleration.z = 99;
  EXPECT_DOUBLE_EQ(result.linear_acceleration.z(), 1.0);
}

TEST(MeasurementConversionsTest, RgbImageOwnsBgrPixelsAndIgnoresRowPadding) {
  sensor_msgs::msg::Image message;
  message.header.stamp = FromSeconds(17.123456789);
  message.width = 1;
  message.height = 2;
  message.encoding = "rgb8";
  message.step = 5;
  message.data = {1, 2, 3, 99, 99, 4, 5, 6, 99, 99};
  ImageMeasurement result;
  ASSERT_TRUE(ConvertImage(message, 0.1, &result).empty());
  message.data.assign(10, 0);
  EXPECT_DOUBLE_EQ(result.timestamp, 17.223456789);
  ASSERT_EQ(result.bgr.type(), CV_8UC3);
  EXPECT_EQ(result.bgr.at<cv::Vec3b>(0, 0), cv::Vec3b(3, 2, 1));
  EXPECT_EQ(result.bgr.at<cv::Vec3b>(1, 0), cv::Vec3b(6, 5, 4));
}

TEST(MeasurementConversionsTest, MonoImageSurvivesMessageDestruction) {
  ImageMeasurement result;
  {
    sensor_msgs::msg::Image message;
    message.width = 2;
    message.height = 1;
    message.encoding = "mono8";
    message.step = 2;
    message.data = {7, 19};
    ASSERT_TRUE(ConvertImage(message, 0, &result).empty());
  }
  EXPECT_EQ(result.bgr.at<cv::Vec3b>(0, 0), cv::Vec3b(7, 7, 7));
  EXPECT_EQ(result.bgr.at<cv::Vec3b>(0, 1), cv::Vec3b(19, 19, 19));
}

TEST(MeasurementConversionsTest, BgrImageDoesNotAliasMessageStorage) {
  sensor_msgs::msg::Image message;
  message.width = 1;
  message.height = 1;
  message.step = 3;
  message.encoding = "bgr8";
  message.data = {7, 8, 9};
  ImageMeasurement result;
  ASSERT_TRUE(ConvertImage(message, 0, &result).empty());
  message.data[0] = 99;
  EXPECT_EQ(result.bgr.at<cv::Vec3b>(0, 0), cv::Vec3b(7, 8, 9));
}

TEST(MeasurementConversionsTest, InvalidImageLeavesOutputUnchanged) {
  ImageMeasurement result{42, cv::Mat(1, 1, CV_8UC3, cv::Scalar(7, 8, 9))};
  sensor_msgs::msg::Image message;
  message.width = 1;
  message.height = 1;
  message.step = 3;
  message.encoding = "invalid";
  message.data = {1, 2, 3};  // Valid size, so the encoding is what fails.
  EXPECT_FALSE(ConvertImage(message, 0, &result).empty());
  message.encoding = "bgr8";
  message.data = {1, 2};
  EXPECT_FALSE(ConvertImage(message, 0, &result).empty());
  message.width = 0;
  message.height = 0;
  message.data.clear();
  EXPECT_FALSE(ConvertImage(message, 0, &result).empty());
  EXPECT_DOUBLE_EQ(result.timestamp, 42);
  EXPECT_EQ(result.bgr.at<cv::Vec3b>(0, 0), cv::Vec3b(7, 8, 9));
}

TEST(MeasurementConversionsTest, LivoxPreservesFieldsAndRejectsCountMismatch) {
  livox_ros_driver2::msg::CustomMsg message;
  message.header.stamp = FromSeconds(17.123456789);
  message.point_num = 2;
  message.points.resize(2);
  message.points[0].x = 1.25;
  message.points[0].y = -2;
  message.points[0].z = 3;
  message.points[0].offset_time = 1234567;
  message.points[0].reflectivity = 231;
  message.points[0].tag = 0x10;
  message.points[0].line = 5;
  message.points[1].offset_time = 7654321;
  LidarScan result;
  ASSERT_TRUE(ConvertLivox(message, &result).empty());
  EXPECT_DOUBLE_EQ(result.timestamp, 17.123456789);
  const auto& points = std::get<std::vector<LivoxPoint>>(result.points);
  ASSERT_EQ(points.size(), 2);
  EXPECT_FLOAT_EQ(points[0].x, 1.25);
  EXPECT_FLOAT_EQ(points[0].y, -2);
  EXPECT_FLOAT_EQ(points[0].z, 3);
  EXPECT_EQ(points[0].offset_time, 1234567u);
  EXPECT_EQ(points[0].reflectivity, 231);
  EXPECT_EQ(points[0].tag, 0x10);
  EXPECT_EQ(points[0].line, 5);
  EXPECT_EQ(points[1].offset_time, 7654321u);
  message.points[0].offset_time = 0;
  message.point_num = 3;
  EXPECT_FALSE(ConvertLivox(message, &result).empty());
  EXPECT_EQ(std::get<std::vector<LivoxPoint>>(result.points)[0].offset_time,
            1234567u);
  message.point_num = 0;
  EXPECT_FALSE(ConvertLivox(message, &result).empty());
}

TEST(MeasurementConversionsTest, VelodynePreservesFloatTimeAndRing) {
  VelodynePoint point{};
  point.x = 3;
  point.intensity = 17;
  point.time = 2500.25f;
  point.ring = 15;
  LidarScan result;
  auto message = MakeCloudMessage(point);
  ASSERT_TRUE(ConvertPointCloud(message, 2, 0.25, &result).empty());
  message.data.clear();
  const auto& decoded = std::get<pcl::PointCloud<VelodynePoint>>(result.points);
  ASSERT_EQ(decoded.size(), 1);
  EXPECT_FLOAT_EQ(decoded[0].x, 3);
  EXPECT_FLOAT_EQ(decoded[0].intensity, 17);
  EXPECT_FLOAT_EQ(decoded[0].time, 2500.25f);
  EXPECT_EQ(decoded[0].ring, 15);
  EXPECT_DOUBLE_EQ(result.timestamp, 17.373456789);
}

TEST(MeasurementConversionsTest, OusterPreservesIntegerFields) {
  OusterPoint point{};
  point.t = 4294967290u;
  point.ring = 63;
  point.reflectivity = 65234;
  point.ambient = 31234;
  point.range = 87654321;
  LidarScan result;
  ASSERT_TRUE(
      ConvertPointCloud(MakeCloudMessage(point), 3, 0, &result).empty());
  const auto& decoded =
      std::get<pcl::PointCloud<OusterPoint>>(result.points)[0];
  EXPECT_EQ(decoded.t, 4294967290u);
  EXPECT_EQ(decoded.ring, 63);
  EXPECT_EQ(decoded.reflectivity, 65234);
  EXPECT_EQ(decoded.ambient, 31234);
  EXPECT_EQ(decoded.range, 87654321u);
}

TEST(MeasurementConversionsTest, MissingVelodyneTimeRetainsYawFallback) {
  pcl::PointCloud<VelodynePoint> points;
  VelodynePoint point{};
  point.x = 2;
  points.push_back(point);
  point.y = -1;
  points.push_back(point);
  auto message = MakeCloudMessage(points);
  RemoveField("time", &message);
  LidarScan scan;
  ASSERT_TRUE(ConvertPointCloud(message, kVelodyne, 0, &scan).empty());
  Preprocess preprocess;
  preprocess.set(false, kVelodyne, 1, 1);
  preprocess.blind_sqr = 1;
  PointCloud result;
  preprocess.Process(scan, &result);
  ASSERT_EQ(result.size(), 1);
  EXPECT_NEAR(result[0].curvature, 7.358730, 0.00001);
}

TEST(MeasurementConversionsTest, OusterAcceptsUnusedFieldsAndRingOmitted) {
  OusterPoint point{};
  point.x = 2;
  point.t = 2500000;
  auto message = MakeCloudMessage(point);
  for (const auto& name : {"reflectivity", "ambient", "range", "ring"}) {
    RemoveField(name, &message);
  }
  LidarScan scan;
  ASSERT_TRUE(ConvertPointCloud(message, kOuster, 0, &scan).empty());
  const auto& decoded = std::get<pcl::PointCloud<OusterPoint>>(scan.points)[0];
  EXPECT_EQ(decoded.ring, 0);
  EXPECT_EQ(decoded.reflectivity, 0);
  EXPECT_EQ(decoded.ambient, 0);
  EXPECT_EQ(decoded.range, 0);
  Preprocess preprocess;
  preprocess.set(false, kOuster, 1, 1);
  preprocess.blind_sqr = 1;
  PointCloud result;
  preprocess.Process(scan, &result);
  ASSERT_EQ(result.size(), 1);
  EXPECT_FLOAT_EQ(result[0].curvature, 2.5);
}

TEST(MeasurementConversionsTest, OptionalRingAndXt32TimestampDefaultToZero) {
  std::vector<sensor_msgs::msg::PointCloud2> messages = {
      MakeCloudMessage(VelodynePoint{}), MakeCloudMessage(Xt32Point{}),
      MakeCloudMessage(Pandar128Point{}), MakeCloudMessage(RobosensePoint{})};
  const int layouts[] = {kVelodyne, kXt32, kPandar128, kRobosense};
  for (std::size_t i = 0; i < messages.size(); ++i) {
    SCOPED_TRACE(layouts[i]);
    RemoveField("ring", &messages[i]);
    if (layouts[i] == kXt32) RemoveField("timestamp", &messages[i]);
    LidarScan scan;
    ASSERT_TRUE(ConvertPointCloud(messages[i], layouts[i], 0, &scan).empty());
    std::visit(
        [](const auto& points) {
          using Points = std::decay_t<decltype(points)>;
          if constexpr (!std::is_same_v<Points, std::vector<LivoxPoint>> &&
                        !std::is_same_v<Points,
                                        pcl::PointCloud<pcl::PointXYZRGB>>) {
            EXPECT_EQ(points[0].ring, 0);
          }
        },
        scan.points);
    if (layouts[i] == kXt32) {
      EXPECT_DOUBLE_EQ(
          std::get<pcl::PointCloud<Xt32Point>>(scan.points)[0].timestamp, 0);
    }
  }
}

TEST(MeasurementConversionsTest, MalformedOptionalFieldsStillReject) {
  const auto valid = MakeCloudMessage(VelodynePoint{});
  for (int kind = 0; kind < 3; ++kind) {
    auto message = valid;
    auto field =
        std::find_if(message.fields.begin(), message.fields.end(),
                     [](const auto& value) { return value.name == "time"; });
    ASSERT_NE(field, message.fields.end());
    if (kind == 0) field->datatype = sensor_msgs::msg::PointField::FLOAT64;
    if (kind == 1) field->count = 2;
    if (kind == 2) field->offset = message.point_step;
    LidarScan scan{42, std::vector<LivoxPoint>(1)};
    EXPECT_FALSE(ConvertPointCloud(message, kVelodyne, 0, &scan).empty());
    EXPECT_DOUBLE_EQ(scan.timestamp, 42);
    EXPECT_TRUE(std::holds_alternative<std::vector<LivoxPoint>>(scan.points));
  }
}

TEST(MeasurementConversionsTest, DepthCloudPreservesPackedColor) {
  pcl::PointXYZRGB point;
  point.x = 2;
  point.r = 1;
  point.g = 2;
  point.b = 3;
  auto message = MakeCloudMessage(point);
  for (int layout = 0; layout < 3; ++layout) {
    for (auto& field : message.fields) {
      if (field.name != "rgb") continue;
      if (layout > 0) field.datatype = sensor_msgs::msg::PointField::UINT32;
      if (layout == 2) field.name = "rgba";
    }
    LidarScan result;
    ASSERT_TRUE(ConvertPointCloud(message, kDepthCamera, 0, &result).empty());
    const auto& decoded =
        std::get<pcl::PointCloud<pcl::PointXYZRGB>>(result.points)[0];
    EXPECT_FLOAT_EQ(decoded.x, 2);
    EXPECT_EQ(decoded.r, 1);
    EXPECT_EQ(decoded.g, 2);
    EXPECT_EQ(decoded.b, 3);
  }
}

TEST(MeasurementConversionsTest, FloatRgbaIsRejectedWithoutChangingOutput) {
  pcl::PointXYZRGB point;
  point.r = 1;
  point.g = 2;
  point.b = 3;
  auto message = MakeCloudMessage(point);
  for (auto& field : message.fields) {
    if (field.name == "rgb") field.name = "rgba";
  }
  LidarScan result{42, std::vector<LivoxPoint>(1)};
  EXPECT_FALSE(ConvertPointCloud(message, kDepthCamera, 0, &result).empty());
  EXPECT_DOUBLE_EQ(result.timestamp, 42);
  EXPECT_TRUE(std::holds_alternative<std::vector<LivoxPoint>>(result.points));
}

TEST(MeasurementConversionsTest, Xt32PreservesDoubleTimestamp) {
  Xt32Point point{};
  point.timestamp = 1700000000.1234567;
  point.ring = 31;
  point.intensity = 53.25;
  LidarScan result;
  ASSERT_TRUE(
      ConvertPointCloud(MakeCloudMessage(point), 5, 0, &result).empty());
  const auto& decoded = std::get<pcl::PointCloud<Xt32Point>>(result.points)[0];
  EXPECT_DOUBLE_EQ(decoded.timestamp, 1700000000.1234567);
  EXPECT_EQ(decoded.ring, 31);
  EXPECT_FLOAT_EQ(decoded.intensity, 53.25);
}

TEST(MeasurementConversionsTest, Pandar128PreservesByteIntensity) {
  Pandar128Point point{};
  point.timestamp = 1000.0025;
  point.ring = 127;
  point.intensity = 255;
  LidarScan result;
  ASSERT_TRUE(
      ConvertPointCloud(MakeCloudMessage(point), 6, 0, &result).empty());
  const auto& decoded =
      std::get<pcl::PointCloud<Pandar128Point>>(result.points)[0];
  EXPECT_DOUBLE_EQ(decoded.timestamp, 1000.0025);
  EXPECT_EQ(decoded.ring, 127);
  EXPECT_EQ(decoded.intensity, 255);
}

TEST(MeasurementConversionsTest, RobosensePreservesFloatIntensity) {
  RobosensePoint point{};
  point.timestamp = 1000.0025;
  point.ring = 127;
  point.intensity = 255.5;
  LidarScan result;
  ASSERT_TRUE(
      ConvertPointCloud(MakeCloudMessage(point), 7, 0, &result).empty());
  const auto& decoded =
      std::get<pcl::PointCloud<RobosensePoint>>(result.points)[0];
  EXPECT_DOUBLE_EQ(decoded.timestamp, 1000.0025);
  EXPECT_EQ(decoded.ring, 127);
  EXPECT_FLOAT_EQ(decoded.intensity, 255.5);
}

TEST(MeasurementConversionsTest, OrganizedCloudUsesRowStrideAndFieldNames) {
  VelodynePoint point{};
  point.x = 2;
  point.time = 2500;
  auto message = MakeCloudMessage(point);
  const auto row = message.data;
  message.height = 2;
  message.row_step += 7;
  message.data.resize(message.row_step * message.height, 99);
  std::copy(row.begin(), row.end(), message.data.begin() + message.row_step);
  std::reverse(message.fields.begin(), message.fields.end());
  LidarScan result;
  ASSERT_TRUE(ConvertPointCloud(message, 2, 0, &result).empty());
  const auto& decoded = std::get<pcl::PointCloud<VelodynePoint>>(result.points);
  ASSERT_EQ(decoded.size(), 2);
  EXPECT_FLOAT_EQ(decoded[1].x, 2);
  EXPECT_FLOAT_EQ(decoded[1].time, 2500);
}

TEST(MeasurementConversionsTest, MalformedCloudLeavesOutputUnchanged) {
  VelodynePoint point{};
  const auto valid = MakeCloudMessage(point);
  std::vector<sensor_msgs::msg::PointCloud2> invalid(10, valid);
  invalid[0].data.pop_back();
  RemoveField("x", &invalid[1]);
  invalid[2].fields[0].datatype = sensor_msgs::msg::PointField::FLOAT64;
  invalid[3].fields[0].offset = valid.point_step;
  invalid[4].fields[0].count = 2;
  invalid[5].row_step = valid.point_step - 1;
  invalid[6].is_bigendian = true;
  invalid[7].width = UINT32_MAX;
  invalid[8].data.push_back(99);
  invalid[9].fields.push_back(valid.fields[0]);
  LidarScan result{42, std::vector<LivoxPoint>(1)};
  for (std::size_t i = 0; i < invalid.size(); ++i) {
    SCOPED_TRACE(i);
    EXPECT_FALSE(ConvertPointCloud(invalid[i], 2, 0, &result).empty());
    EXPECT_DOUBLE_EQ(result.timestamp, 42);
    EXPECT_TRUE(std::holds_alternative<std::vector<LivoxPoint>>(result.points));
  }
  EXPECT_FALSE(ConvertPointCloud(valid, 1, 0, &result).empty());
  EXPECT_FALSE(ConvertPointCloud(valid, 99, 0, &result).empty());
}

TEST(MeasurementConversionsTest, EmptyCloudRetainsItsSelectedLayout) {
  sensor_msgs::msg::PointCloud2 message;
  LidarScan result;
  ASSERT_TRUE(ConvertPointCloud(message, 5, 0, &result).empty());
  EXPECT_TRUE(std::get<pcl::PointCloud<Xt32Point>>(result.points).empty());
}

}  // namespace
}  // namespace livo_atlas::ros2

namespace livo_atlas::ros2 {
TEST(Ros2TimeTest, PreservesLargeEpochExpressionAndNegativeTimes) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 1700000000;
  stamp.nanosec = 123456789;
  EXPECT_DOUBLE_EQ(ToSeconds(stamp), 1700000000.0 + 123456789.0 * 1e-9);
  const auto negative = FromSeconds(-0.25);
  EXPECT_EQ(negative.sec, -1);
  EXPECT_EQ(negative.nanosec, 750000000u);
  EXPECT_DOUBLE_EQ(ToSeconds(negative), -0.25);
}
TEST(Ros2TimeTest, RoundsNanosecondsAndCarriesBeforeCheckingBounds) {
  auto stamp = FromSeconds(17.9999999996);
  EXPECT_EQ(stamp.sec, 18);
  EXPECT_EQ(stamp.nanosec, 0u);
  stamp = FromSeconds(0.1234567894);
  EXPECT_EQ(stamp.nanosec, 123456789u);
  EXPECT_THROW(FromSeconds(std::numeric_limits<double>::infinity()),
               std::invalid_argument);
  EXPECT_THROW(FromSeconds(2147483648.0), std::invalid_argument);
  EXPECT_THROW(FromSeconds(-2147483649.0), std::invalid_argument);
  stamp.nanosec = 1000000000u;
  EXPECT_THROW(ToSeconds(stamp), std::invalid_argument);
}
TEST(Ros2TimeTest, InvalidStampDoesNotModifyImageOrCloudOutput) {
  sensor_msgs::msg::Image image;
  image.header.stamp.nanosec = 1000000000u;
  image.width = image.height = 1;
  image.step = 3;
  image.encoding = "bgr8";
  image.data = {1, 2, 3};
  ImageMeasurement output{42, cv::Mat{}};
  EXPECT_FALSE(ConvertImage(image, 0, &output).empty());
  EXPECT_DOUBLE_EQ(output.timestamp, 42);
  livox_ros_driver2::msg::CustomMsg cloud;
  cloud.header.stamp = image.header.stamp;
  LidarScan scan{42, std::vector<LivoxPoint>{}};
  EXPECT_FALSE(ConvertLivox(cloud, &scan).empty());
  EXPECT_DOUBLE_EQ(scan.timestamp, 42);
}
}  // namespace livo_atlas::ros2

namespace livo_atlas::ros2 {
TEST(MeasurementConversionsTest, RejectsEveryNonfiniteImuComponent) {
  for (int component = 0; component < 6; ++component) {
    for (double value : {std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity()}) {
      sensor_msgs::msg::Imu message;
      message.header.stamp = FromSeconds(100);
      message.linear_acceleration.z = -9.81;
      const std::array<double*, 6> values = {
          &message.angular_velocity.x,    &message.angular_velocity.y,
          &message.angular_velocity.z,    &message.linear_acceleration.x,
          &message.linear_acceleration.y, &message.linear_acceleration.z};
      *values[component] = value;
      EXPECT_THROW(ConvertImu(message), std::invalid_argument);
    }
  }
}
}  // namespace livo_atlas::ros2
