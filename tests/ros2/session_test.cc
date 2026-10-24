#include <cv_bridge/cv_bridge.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <limits>

#include "livo_atlas/ros2/measurement_conversions.h"
#include "ros2/fixtures.h"
#include "ros2/mapping_session.h"
#include "test_images.h"
namespace livo_atlas::ros2 {
namespace {
TEST(SessionTest, RejectsEarlyAndReverseDataWithoutBlockingValidScan) {
  MappingSession session(test::Options(FrontendMode::kLidarInertial));
  EXPECT_FALSE(session.AddImu({100, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  EXPECT_TRUE(session.AddLidar(test::Scan(100)));
  EXPECT_TRUE(session.AddImu({100.1, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  EXPECT_FALSE(session.AddImu({99, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  EXPECT_FALSE(session.AddLidar({99, {}}));
  session.Finish();
}
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
  MappingSession native(options);
  options.output.directory /= "converted";
  MappingSession converted(options);
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
      ImuMeasurement imu{timestamp, Eigen::Vector3d::Zero(), {0, 0, -9.81}};
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
      ImageMeasurement decoded_image;
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
TEST(SessionTest, NativeAndConvertedLidarOnlyMatch) {
  CompareSessions(FrontendMode::kLidarOnly);
}
TEST(SessionTest, NativeAndConvertedLidarInertialMatch) {
  CompareSessions(FrontendMode::kLidarInertial);
}
TEST(SessionTest, NativeAndConvertedRepeatedVisualMatch) {
  CompareSessions(FrontendMode::kLidarInertialVisual);
}
TEST(SessionTest, IndependentPropagationCursors) {
  auto options = test::Options(FrontendMode::kLidarInertial);
  options.propagate_imu = true;
  MappingSession a(options);
  options.output.directory /= "second";
  MappingSession b(options);
  for (int frame = 0; frame < 3; ++frame) {
    double t = 100 + frame * .25;
    for (auto* session : {&a, &b}) {
      ASSERT_TRUE(session->AddLidar(test::Scan(t)));
      session->AddImu({t + .0625, Eigen::Vector3d::Zero(), {0, 0, -9.81}});
      session->AddImu({t + .125, Eigen::Vector3d::Zero(), {0, 0, -9.81}});
      session->ProcessAvailable([](const auto&, const auto&, double) {});
      session->AddImu({t + .1875, Eigen::Vector3d::Zero(), {1, 0, -9.81}});
    }
    auto first = a.PropagateLatest(), second = b.PropagateLatest();
    if (frame == 0) {
      EXPECT_FALSE(first);
      EXPECT_FALSE(second);
      continue;
    }
    ASSERT_TRUE(first);
    ASSERT_TRUE(second);
    EXPECT_TRUE(first->state.pos_end == second->state.pos_end);
    EXPECT_FALSE(a.PropagateLatest());
    EXPECT_FALSE(b.PropagateLatest());
  }
  a.Finish();
  b.Finish();
}
}  // namespace
}  // namespace livo_atlas::ros2

namespace livo_atlas::ros2 {
TEST(SessionTest, ImageRejectionAndHiltiCountersBelongToEachInstance) {
  auto options = test::Options(FrontendMode::kLidarInertialVisual);
  MappingSession normal(options);
  const auto image = test_data::MakeImage(512, 512, 3);
  EXPECT_FALSE(normal.AddImage({100.1, image}));
  ASSERT_TRUE(normal.AddLidar(test::Scan(100)));
  EXPECT_TRUE(normal.AddImage({100.1, image}));
  EXPECT_FALSE(normal.AddImage({100.1, image}));
  EXPECT_FALSE(normal.AddImage({100.11, image}));
  EXPECT_FALSE(normal.AddImage({100.08, image}));
  EXPECT_TRUE(normal.AddImage({100.2, image}));
  normal.Finish();
  options.hilti = true;
  options.output.directory /= "hilti-first";
  MappingSession first(options);
  options.output.directory /= "hilti-second";
  MappingSession second(options);
  ASSERT_TRUE(first.AddLidar(test::Scan(100)));
  ASSERT_TRUE(second.AddLidar(test::Scan(100)));
  for (int i = 1; i <= 4; ++i) {
    EXPECT_EQ(first.AddImage({100 + i * .05, image}), i == 4);
    EXPECT_EQ(second.AddImage({100 + i * .05, image}), i == 4);
  }
  first.Finish();
  second.Finish();
}
}  // namespace livo_atlas::ros2

namespace livo_atlas::ros2 {
namespace {
void ExpectSameStates(const std::vector<FrontendResult>& reference,
                      const std::vector<FrontendResult>& actual) {
  ASSERT_EQ(reference.size(), actual.size());
  ASSERT_GT(reference.size(), 2u);
  for (std::size_t i = 0; i < reference.size(); ++i) {
    EXPECT_DOUBLE_EQ(reference[i].timestamp, actual[i].timestamp);
    const auto& a = reference[i].estimate;
    const auto& b = actual[i].estimate;
    EXPECT_TRUE(a.rot_end == b.rot_end);
    EXPECT_TRUE(a.pos_end == b.pos_end);
    EXPECT_TRUE(a.vel_end == b.vel_end);
    EXPECT_TRUE(a.bias_a == b.bias_a);
    EXPECT_TRUE(a.bias_g == b.bias_g);
    EXPECT_TRUE(a.gravity == b.gravity);
    EXPECT_TRUE(a.cov == b.cov);
    EXPECT_DOUBLE_EQ(a.inv_expo_time, b.inv_expo_time);
    EXPECT_TRUE(b.pos_end.allFinite() && b.cov.allFinite());
  }
}
TEST(SessionTest, RejectingNonfiniteImuLeavesValidSequenceUnchanged) {
  auto options = test::Options(FrontendMode::kLidarInertial);
  MappingSession reference(options);
  options.output.directory /= "injected";
  MappingSession actual(options);
  std::vector<FrontendResult> expected, observed;
  for (int frame = 0; frame < 6; ++frame) {
    const double t = 100 + frame * .25;
    reference.AddLidar(test::Scan(t));
    actual.AddLidar(test::Scan(t));
    for (int axis = 0; axis < 6; ++axis) {
      ImuMeasurement invalid{t + 1000, Eigen::Vector3d::Zero(), {0, 0, -9.81}};
      if (axis < 3)
        invalid.angular_velocity[axis] =
            std::numeric_limits<double>::quiet_NaN();
      else
        invalid.linear_acceleration[axis - 3] =
            std::numeric_limits<double>::infinity();
      ASSERT_FALSE(actual.AddImu(invalid));
    }
    for (double offset : {.0625, .125, .1875}) {
      const ImuMeasurement imu{
          t + offset, Eigen::Vector3d::Zero(), {frame * .01, 0, -9.81}};
      ASSERT_TRUE(reference.AddImu(imu));
      ASSERT_TRUE(actual.AddImu(imu));
    }
    reference.ProcessAvailable(
        [&](const auto& r, const auto&, double) { expected.push_back(r); });
    actual.ProcessAvailable(
        [&](const auto& r, const auto&, double) { observed.push_back(r); });
  }
  ExpectSameStates(expected, observed);
  reference.Finish();
  actual.Finish();
}
TEST(SessionTest, AppliesImuOffsetAndDriverCorrectionAtActualIngress) {
  struct Case {
    double begin;
    double offset;
    double raw_delta;
    bool driver;
  };
  // Corrected reference stamps are dyadic fractions. The final case crosses
  // a whole-second nanosecond carry: 1000.999999999 - (-0.6ns) -> 1001.
  for (const auto& c :
       std::vector<Case>{{1000, .03, .03, false},
                         {1700000000, .03, -512 + .03, true},
                         {1000.9375, -.0000000006, -.000000001, false}}) {
    SCOPED_TRACE(c.begin);
    auto options = test::Options(FrontendMode::kLidarInertial);
    MappingSession reference(options);
    options.output.directory /= "corrected";
    options.imu_time_offset = c.offset;
    options.driver_fix = c.driver;
    MappingSession actual(options);
    std::vector<FrontendResult> expected, observed;
    for (int frame = 0; frame < 6; ++frame) {
      const double t = c.begin + frame * .25;
      reference.AddLidar(test::Scan(t));
      actual.AddLidar(test::Scan(t));
      for (double offset : {.0625, .125, .1875}) {
        ImuMeasurement imu{
            t + offset, Eigen::Vector3d::Zero(), {frame * .01, 0, -9.81}};
        ASSERT_TRUE(reference.AddImu(imu));
        imu.timestamp = ToSeconds(FromSeconds(t + offset + c.raw_delta));
        ASSERT_TRUE(actual.AddImu(imu));
      }
      ImuMeasurement reverse{ToSeconds(FromSeconds(t + .04 + c.raw_delta)),
                             Eigen::Vector3d::Zero(),
                             {0, 0, -9.81}};
      EXPECT_FALSE(actual.AddImu(reverse));
      reference.ProcessAvailable(
          [&](const auto& r, const auto&, double) { expected.push_back(r); });
      actual.ProcessAvailable(
          [&](const auto& r, const auto&, double) { observed.push_back(r); });
    }
    ExpectSameStates(expected, observed);
    reference.Finish();
    actual.Finish();
  }
}
}  // namespace
}  // namespace livo_atlas::ros2
