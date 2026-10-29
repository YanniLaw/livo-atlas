#include "io/colmap_camera.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace livo_atlas {
namespace {
CameraConfig Config() {
  return {"Pinhole", 12, 8, .5, 8, 10, 6, 4, {0, 0, 0, 0}};
}
Eigen::Matrix3d Rz90() {
  Eigen::Matrix3d rotation;
  rotation << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  return rotation;
}
cv::Mat Image(int width, int height) {
  cv::Mat image(height, width, CV_8UC3);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x)
      image.at<cv::Vec3b>(y, x) = {static_cast<unsigned char>(10 + x),
                                   static_cast<unsigned char>(20 + y),
                                   static_cast<unsigned char>(30 + x + y)};
  }
  return image;
}
Keyframe Frame() {
  Keyframe frame;
  frame.timestamp = frame.cloud_timestamp = 1;
  frame.image_timestamp = 1;
  frame.odom_from_body = {Rz90(), Eigen::Vector3d(1, 2, 3)};
  frame.camera_from_odom =
      RigidTransform{Eigen::Matrix3d::Identity(), Eigen::Vector3d(-1, -2, -2)};
  frame.image = Image(6, 4);
  PointCloud::Ptr cloud(new PointCloud);
  Point point{};
  point.x = .5;
  point.z = 3;
  point.intensity = 7;
  cloud->push_back(point);
  frame.cloud = cloud;
  return frame;
}

TEST(ColmapCameraTest,
     PreservesTheObservedCameraPoseAndProjectsLiteralGeometry) {
  ColmapCamera camera(Config());
  const auto frame = Frame();
  const auto view = camera.MakeView(frame, frame.odom_from_body, 2000);
  EXPECT_EQ(camera.pinhole().width, 6);
  EXPECT_EQ(camera.pinhole().height, 4);
  EXPECT_EQ(camera.pinhole().scale, 1);
  EXPECT_EQ(camera.pinhole().fx, 4);
  EXPECT_EQ(camera.pinhole().fy, 5);
  EXPECT_EQ(camera.pinhole().cx, 3);
  EXPECT_EQ(camera.pinhole().cy, 2);
  EXPECT_TRUE(
      view.camera_from_parent.rotation.isApprox(Eigen::Matrix3d::Identity()));
  EXPECT_TRUE(view.camera_from_parent.translation.isApprox(
      Eigen::Vector3d(-1, -2, -2)));
  ASSERT_EQ(view.samples.size(), 1u);
  EXPECT_TRUE(view.samples[0].position.isApprox(Eigen::Vector3d(1, 2.5, 6)));
  EXPECT_TRUE(view.samples[0].pixel.isApprox(Eigen::Vector2d(3, 2.625)));
  EXPECT_EQ(view.samples[0].rgb, cv::Vec3b(35, 22, 13));
}

TEST(ColmapCameraTest, AppliesTheSameBodyCorrectionToTheCameraAndTheSeeds) {
  ColmapCamera camera(Config());
  const auto frame = Frame();
  RigidTransform optimized;
  optimized.translation << 10, 0, 0;
  const auto view = camera.MakeView(frame, optimized, 2000);
  EXPECT_TRUE(view.camera_from_parent.rotation.isApprox(Rz90()));
  EXPECT_TRUE(
      view.camera_from_parent.translation.isApprox(Eigen::Vector3d(0, -10, 1)));
  ASSERT_EQ(view.samples.size(), 1u);
  EXPECT_TRUE(view.samples[0].position.isApprox(Eigen::Vector3d(10.5, 0, 3)));
  EXPECT_TRUE(view.samples[0].pixel.isApprox(Eigen::Vector2d(3, 2.625)));
  EXPECT_EQ(view.samples[0].rgb, cv::Vec3b(35, 22, 13));
}

TEST(ColmapCameraTest, ReturnsOwnedImagesWithoutChangingTheInputCloudOrPixels) {
  const auto frame = Frame();
  const auto original = frame.image.clone();
  auto view =
      ColmapCamera(Config()).MakeView(frame, frame.odom_from_body, 2000);
  EXPECT_EQ(cv::norm(view.image, original, cv::NORM_INF), 0);
  ASSERT_NE(view.image.data, frame.image.data);
  view.image.setTo(cv::Scalar(0, 0, 0));
  EXPECT_EQ(cv::norm(frame.image, original, cv::NORM_INF), 0);
  EXPECT_EQ(frame.cloud->front().x, .5);
  EXPECT_EQ(frame.cloud->front().z, 3);
  EXPECT_EQ(frame.cloud->front().intensity, 7);
}

TEST(ColmapCameraTest, RectifiesAllPinholeCoefficientsEvenWhenK1IsZero) {
  auto config = Config();
  config.width = 32;
  config.height = 24;
  config.fx = 16;
  config.fy = 19;
  config.cx = 16;
  config.cy = 12;
  config.distortion = {0, .3, .04, -.03};
  auto frame = Frame();
  frame.image = Image(16, 12);
  for (int y = 0; y < frame.image.rows; ++y) {
    for (int x = 0; x < frame.image.cols; ++x)
      frame.image.at<cv::Vec3b>(y, x)[0] = (x + y) % 2 ? 255 : 0;
  }
  const cv::Mat intrinsics =
      (cv::Mat_<double>(3, 3) << 8, 0, 8, 0, 9.5, 6, 0, 0, 1);
  const cv::Mat distortion = (cv::Mat_<double>(4, 1) << 0, .3, .04, -.03);
  cv::Mat expected;
  cv::undistort(frame.image, expected, intrinsics, distortion, intrinsics);
  ASSERT_GT(cv::norm(expected, frame.image, cv::NORM_INF), 4);
  const auto view =
      ColmapCamera(config).MakeView(frame, frame.odom_from_body, 2000);
  EXPECT_LE(cv::norm(view.image, expected, cv::NORM_INF), 1);
}

TEST(ColmapCameraTest, RectifiesFisheyeIncludingTheOpticalAxis) {
  auto config = Config();
  config.model = "EquidistantCamera";
  config.distortion = {.1, -.02, .003, -.0004};
  auto frame = Frame();
  frame.odom_from_body = {};
  frame.camera_from_odom = RigidTransform{};
  PointCloud::Ptr cloud(new PointCloud(*frame.cloud));
  cloud->front().x = 0;
  frame.cloud = cloud;
  const cv::Mat intrinsics =
      (cv::Mat_<double>(3, 3) << 4, 0, 3, 0, 5, 2, 0, 0, 1);
  const cv::Mat distortion = (cv::Mat_<double>(4, 1) << .1, -.02, .003, -.0004);
  cv::Mat expected;
  cv::fisheye::undistortImage(frame.image, expected, intrinsics, distortion,
                              intrinsics, frame.image.size());
  const auto view =
      ColmapCamera(config).MakeView(frame, frame.odom_from_body, 2000);
  EXPECT_LE(cv::norm(view.image, expected, cv::NORM_INF), 1);
  ASSERT_EQ(view.samples.size(), 1u);
  EXPECT_TRUE(view.samples[0].pixel.isApprox(Eigen::Vector2d(3, 2)));
  EXPECT_EQ(view.samples[0].rgb, cv::Vec3b(35, 22, 13));
}

TEST(ColmapCameraTest, TruncatesScaledDimensionsWithoutScalingTwice) {
  auto config = Config();
  config.width = 11;
  config.height = 9;
  auto frame = Frame();
  frame.image = Image(5, 4);
  const auto view =
      ColmapCamera(config).MakeView(frame, frame.odom_from_body, 2000);
  EXPECT_EQ(view.image.cols, 5);
  EXPECT_EQ(view.image.rows, 4);
  ASSERT_EQ(view.samples.size(), 1u);
  EXPECT_TRUE(view.samples[0].pixel.isApprox(Eigen::Vector2d(3, 2.625)));
}

TEST(ColmapCameraTest,
     KeepsTheNearestVisiblePixelAndRejectsBordersAndBadDepth) {
  auto frame = Frame();
  PointCloud::Ptr cloud(new PointCloud(*frame.cloud));
  auto far = cloud->front();
  far.x = 1;
  far.z = 7;
  cloud->points.insert(cloud->points.begin(), far);
  auto outside = far;
  outside.x = 20;
  cloud->points.push_back(outside);
  auto behind = far;
  behind.z = -2;
  cloud->points.push_back(behind);
  auto border = far;
  border.x = 0;
  border.y = -2.25;
  border.z = 3;
  cloud->points.push_back(border);
  auto lateral = far;
  lateral.x = std::numeric_limits<float>::max();
  lateral.z = std::numeric_limits<float>::denorm_min();
  cloud->points.push_back(lateral);
  cloud->width = cloud->size();
  cloud->height = 1;
  frame.cloud = cloud;
  const auto view =
      ColmapCamera(Config()).MakeView(frame, frame.odom_from_body, 2000);
  ASSERT_EQ(view.samples.size(), 1u);
  EXPECT_TRUE(view.samples[0].position.isApprox(Eigen::Vector3d(1, 2.5, 6)));
}

TEST(ColmapCameraTest, SamplesInRasterOrderWithAnEnforcedPointLimit) {
  auto frame = Frame();
  frame.odom_from_body = {};
  frame.camera_from_odom = RigidTransform{};
  PointCloud::Ptr cloud(new PointCloud);
  for (float x : {1.f, -1.f, 0.f, -2.f}) {
    Point point{};
    point.x = x;
    point.z = 4;
    cloud->push_back(point);
  }
  frame.cloud = cloud;
  const ColmapCamera camera(Config());
  const auto one = camera.MakeView(frame, {}, 1);
  ASSERT_EQ(one.samples.size(), 1u);
  EXPECT_TRUE(one.samples[0].position.isApprox(Eigen::Vector3d(-2, 0, 4)));
  const auto two = camera.MakeView(frame, {}, 2);
  ASSERT_EQ(two.samples.size(), 2u);
  EXPECT_TRUE(two.samples[0].position.isApprox(Eigen::Vector3d(-2, 0, 4)));
  EXPECT_TRUE(two.samples[1].position.isApprox(Eigen::Vector3d(0, 0, 4)));
}

TEST(ColmapCameraTest, RejectsInvalidViewsLimitsAndOverflowingComposition) {
  ColmapCamera camera(Config());
  auto frame = Frame();
  EXPECT_THROW(camera.MakeView(frame, {}, 0), std::invalid_argument);
  EXPECT_THROW(camera.MakeView(frame, {}, UINT32_MAX), std::invalid_argument);
  frame.image = cv::Mat(4, 6, CV_8UC1);
  EXPECT_THROW(camera.MakeView(frame, {}, 1), std::invalid_argument);
  frame.image = Image(5, 4);
  EXPECT_THROW(camera.MakeView(frame, {}, 1), std::invalid_argument);
  frame = Frame();
  frame.camera_from_odom.reset();
  EXPECT_THROW(camera.MakeView(frame, {}, 1), std::invalid_argument);
  frame = Frame();
  frame.odom_from_body = {};
  frame.camera_from_odom = RigidTransform{};
  frame.odom_from_body.translation.setConstant(
      std::numeric_limits<double>::max());
  frame.camera_from_odom->translation.setConstant(
      std::numeric_limits<double>::max());
  EXPECT_THROW(camera.MakeView(frame, {}, 1), std::invalid_argument);
}
}  // namespace
}  // namespace livo_atlas
