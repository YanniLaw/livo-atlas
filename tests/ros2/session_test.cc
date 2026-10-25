#include <cv_bridge/cv_bridge.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <limits>

#include "livo_atlas/ros2/measurement_conversions.h"
#include "livo_atlas/core/livo_system.h"
#include "ros2/fixtures.h"
#include "test_images.h"
namespace livo_atlas::ros2 {
namespace {
std::vector<float> Points(const FrontendResult& result,
                          const PreparedOutput& prepared) {
  std::vector<float> values;
  if (result.world_cloud)
    for (const auto& point : *result.world_cloud) {
      values.insert(values.end(), {point.x, point.y, point.z, point.intensity,
                                   point.curvature});
    }
  if (prepared.colored_cloud)
    for (const auto& point : *prepared.colored_cloud) {
      values.insert(values.end(), {point.x, point.y, point.z, float(point.r),
                                   float(point.g), float(point.b)});
    }
  return values;
}
void CompareSessions(FrontendMode mode) {
  auto options = test::Options(mode);
  // The legacy processor starts its counter at one and counts LIO and VIO.
  options.processor.scan_interval = 4;
  LivoSystem native(options);
  options.exporter.directory /= "converted";
  LivoSystem converted(options);
  std::vector<FrontendResult> a, b;
  std::vector<bool> clouds_a, clouds_b;
  std::vector<std::vector<float>> points_a, points_b;
  std::vector<std::uint64_t> images_a, images_b;
  std::vector<std::size_t> visual_cloud_sizes;
  auto first = [&](const auto& r, const auto& p, double) {
    a.push_back(r);
    clouds_a.push_back(p.has_cloud);
    if (r.kind == UpdateKind::kVisual)
      visual_cloud_sizes.push_back(p.colored_cloud->size());
    points_a.push_back(Points(r, p));
    images_a.push_back(test_data::ImageHash(r.tracking_image));
  };
  auto second = [&](const auto& r, const auto& p, double) {
    b.push_back(r);
    clouds_b.push_back(p.has_cloud);
    points_b.push_back(Points(r, p));
    images_b.push_back(test_data::ImageHash(r.tracking_image));
  };
  int visual = 0, lidar = 0;
  for (int frame = 0; frame < 6; ++frame) {
    double t = 100 + frame * .25;
    auto scan = test::Scan(t);
    ASSERT_TRUE(native.AddLidar(scan));
    livox_ros_driver2::msg::CustomMsg msg;
    msg.header.stamp = FromSeconds(t);
    for (const auto& p : std::get<std::vector<LivoxPoint>>(scan.points)) {
      livox_ros_driver2::msg::CustomPoint q;
      q.x = p.x;
      q.y = p.y;
      q.z = p.z;
      q.offset_time = p.offset_time;
      q.line = p.line;
      q.tag = p.tag;
      msg.points.push_back(q);
    }
    msg.point_num = msg.points.size();
    LidarScan decoded;
    ASSERT_TRUE(ConvertLivox(msg, &decoded).empty());
    ASSERT_TRUE(converted.AddLidar(decoded));
    for (double timestamp : {t + .0625, t + .125, t + .1875}) {
      Imu imu{timestamp, Eigen::Vector3d::Zero(), {0, 0, -9.81}};
      native.AddImu(imu);
      sensor_msgs::msg::Imu m;
      m.header.stamp = FromSeconds(timestamp);
      m.linear_acceleration.z = -9.81;
      converted.AddImu(ConvertImu(m));
    }
    if (options.image_enabled) {
      auto image = test_data::MakeImage(512, 512, 3, true);
      native.AddImage({t + .125, image});
      cv_bridge::CvImage bridge;
      bridge.header.stamp = FromSeconds(t + .125);
      bridge.encoding = "bgr8";
      bridge.image = image;
      TimedImage decoded_image;
      ASSERT_TRUE(
          ConvertImage(*bridge.toImageMsg(), 0, &decoded_image).empty());
      converted.AddImage(std::move(decoded_image));
    }
    native.ProcessAvailable(first);
    converted.ProcessAvailable(second);
    ASSERT_EQ(a.size(), b.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
      EXPECT_EQ(a[i].kind, b[i].kind);
      EXPECT_DOUBLE_EQ(a[i].timestamp, b[i].timestamp);
      EXPECT_TRUE(a[i].estimate.rot_end == b[i].estimate.rot_end);
      EXPECT_TRUE(a[i].estimate.pos_end == b[i].estimate.pos_end);
      EXPECT_TRUE(a[i].estimate.cov == b[i].estimate.cov);
      EXPECT_TRUE(a[i].estimate.pos_end.allFinite());
    }
  }
  for (const auto& result : a)
    result.kind == UpdateKind::kVisual ? ++visual : ++lidar;
  EXPECT_GT(lidar, 2);
  if (options.image_enabled) EXPECT_GT(visual, 2);
  EXPECT_EQ(clouds_a, clouds_b);
  EXPECT_EQ(points_a, points_b);
  EXPECT_EQ(images_a, images_b);
  if (options.image_enabled) {
    EXPECT_NE(
        std::find(visual_cloud_sizes.begin(), visual_cloud_sizes.end(), 0),
        visual_cloud_sizes.end());
    EXPECT_TRUE(std::any_of(visual_cloud_sizes.begin(),
                            visual_cloud_sizes.end(),
                            [](auto size) { return size > 0; }));
  }
  EXPECT_NE(std::find(clouds_a.begin(), clouds_a.end(), true), clouds_a.end());
  native.Finish();
  converted.Finish();
}
TEST(SystemTest, NativeAndConvertedLidarOnlyMatch) {
  CompareSessions(FrontendMode::kLidarOnly);
}
TEST(SystemTest, NativeAndConvertedLidarInertialMatch) {
  CompareSessions(FrontendMode::kLidarInertial);
}
TEST(SystemTest, NativeAndConvertedRepeatedVisualMatch) {
  CompareSessions(FrontendMode::kLidarInertialVisual);
}
}  // namespace
}  // namespace livo_atlas::ros2
