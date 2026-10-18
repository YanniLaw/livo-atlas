#include "livo_atlas/ros1/measurement_conversions.h"

#include <algorithm>
#include <cstdint>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>

namespace livo_atlas::ros1 {
namespace {

template <typename PointType>
sensor_msgs::PointCloud2 MakeCloudMessage(const PointType& point) {
  pcl::PointCloud<PointType> cloud;
  cloud.push_back(point);
  sensor_msgs::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header.stamp = ros::Time(17, 123456789);
  return message;
}

TEST(MeasurementConversionsTest, ImuPreservesCorrectedTimeAndRawValues) {
  sensor_msgs::Imu message;
  message.header.stamp = ros::Time(17, 123456789);
  message.angular_velocity.x = 1.25;
  message.angular_velocity.y = -2.5;
  message.angular_velocity.z = 3.75;
  message.linear_acceleration.x = -0.25;
  message.linear_acceleration.y = 0.5;
  message.linear_acceleration.z = 1.0;
  // Same ROS Time round trips as the adapter, including driver compensation.
  message.header.stamp.fromSec(message.header.stamp.toSec() - 0.03);
  message.header.stamp.fromSec(message.header.stamp.toSec() + 983.0);
  const auto result = ConvertImu(message);
  EXPECT_DOUBLE_EQ(result.timestamp, 1000.093456789);
  EXPECT_TRUE(result.angular_velocity.isApprox(Eigen::Vector3d(1.25, -2.5, 3.75), 0));
  EXPECT_TRUE(result.linear_acceleration.isApprox(Eigen::Vector3d(-0.25, 0.5, 1), 0));
  message.linear_acceleration.z = 99;
  EXPECT_DOUBLE_EQ(result.linear_acceleration.z(), 1.0);
}

TEST(MeasurementConversionsTest, RgbImageOwnsBgrPixelsAndIgnoresRowPadding) {
  sensor_msgs::Image message;
  message.header.stamp = ros::Time(17, 123456789);
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
    sensor_msgs::Image message;
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
  sensor_msgs::Image message;
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
  sensor_msgs::Image message;
  message.width = 1;
  message.height = 1;
  message.step = 3;
  message.encoding = "invalid";
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
  livox_ros_driver::CustomMsg message;
  message.header.stamp = ros::Time(17, 123456789);
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
  EXPECT_EQ(std::get<std::vector<LivoxPoint>>(result.points)[0].offset_time, 1234567u);
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
  ASSERT_TRUE(ConvertPointCloud(MakeCloudMessage(point), 3, 0, &result).empty());
  const auto& decoded = std::get<pcl::PointCloud<OusterPoint>>(result.points)[0];
  EXPECT_EQ(decoded.t, 4294967290u);
  EXPECT_EQ(decoded.ring, 63);
  EXPECT_EQ(decoded.reflectivity, 65234);
  EXPECT_EQ(decoded.ambient, 31234);
  EXPECT_EQ(decoded.range, 87654321u);
}

TEST(MeasurementConversionsTest, DepthCloudPreservesPackedColor) {
  pcl::PointXYZRGB point;
  point.x = 2;
  point.r = 1;
  point.g = 2;
  point.b = 3;
  LidarScan result;
  ASSERT_TRUE(ConvertPointCloud(MakeCloudMessage(point), 4, 0, &result).empty());
  const auto& decoded = std::get<pcl::PointCloud<pcl::PointXYZRGB>>(result.points)[0];
  EXPECT_FLOAT_EQ(decoded.x, 2);
  EXPECT_EQ(decoded.r, 1);
  EXPECT_EQ(decoded.g, 2);
  EXPECT_EQ(decoded.b, 3);
}

TEST(MeasurementConversionsTest, Xt32PreservesDoubleTimestamp) {
  Xt32Point point{};
  point.timestamp = 1700000000.1234567;
  point.ring = 31;
  point.intensity = 53.25;
  LidarScan result;
  ASSERT_TRUE(ConvertPointCloud(MakeCloudMessage(point), 5, 0, &result).empty());
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
  ASSERT_TRUE(ConvertPointCloud(MakeCloudMessage(point), 6, 0, &result).empty());
  const auto& decoded = std::get<pcl::PointCloud<Pandar128Point>>(result.points)[0];
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
  ASSERT_TRUE(ConvertPointCloud(MakeCloudMessage(point), 7, 0, &result).empty());
  const auto& decoded = std::get<pcl::PointCloud<RobosensePoint>>(result.points)[0];
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
  std::vector<sensor_msgs::PointCloud2> invalid(10, valid);
  invalid[0].data.pop_back();
  invalid[1].fields.pop_back();
  invalid[2].fields[0].datatype = sensor_msgs::PointField::FLOAT64;
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
  sensor_msgs::PointCloud2 message;
  LidarScan result;
  ASSERT_TRUE(ConvertPointCloud(message, 5, 0, &result).empty());
  EXPECT_TRUE(std::get<pcl::PointCloud<Xt32Point>>(result.points).empty());
}

}  // namespace
}  // namespace livo_atlas::ros1
