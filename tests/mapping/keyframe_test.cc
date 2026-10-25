#include "livo_atlas/mapping/keyframe.h"

#include <gtest/gtest.h>

#include <functional>
#include <limits>
#include <stdexcept>
#include <vector>

#include "mapping/keyframe_fixtures.h"

namespace livo_atlas {
namespace {
TEST(KeyframeTest, OwnsStridedImageAndCloudInTheAnchorBodyFrame) {
  auto result = test_data::KeyframeResult();
  cv::Mat padded(4, 8, CV_8UC3, cv::Scalar(7, 8, 9));
  result.color_image = padded(cv::Rect(0, 0, 6, 4));
  ASSERT_FALSE(result.color_image.isContinuous());
  auto snapshot = CaptureKeyframe(37, result);
  EXPECT_EQ(snapshot.id, 37u);
  EXPECT_DOUBLE_EQ(snapshot.timestamp, 10.25);
  EXPECT_DOUBLE_EQ(snapshot.cloud_timestamp, 10.2);
  ASSERT_TRUE(snapshot.image_timestamp);
  EXPECT_DOUBLE_EQ(*snapshot.image_timestamp, 10.24);
  EXPECT_DOUBLE_EQ(snapshot.inverse_exposure_time, .8);
  ASSERT_TRUE(snapshot.camera_from_odom);
  EXPECT_EQ(snapshot.camera_from_odom->translation, Eigen::Vector3d(7, 8, 9));
  ASSERT_EQ(snapshot.cloud->size(), 1u);
  const auto& point = snapshot.cloud->front();
  EXPECT_FLOAT_EQ(point.x, 1);
  EXPECT_FLOAT_EQ(point.y, 0);
  EXPECT_FLOAT_EQ(point.z, 0);
  EXPECT_FLOAT_EQ(point.intensity, 42);
  EXPECT_EQ(snapshot.image.at<cv::Vec3b>(0, 0), cv::Vec3b(7, 8, 9));
  padded.setTo(cv::Scalar(100, 100, 100));
  result.color_image.release();
  result.world_cloud.reset();
  result.estimate.pos_end.setZero();
  EXPECT_EQ(snapshot.image.at<cv::Vec3b>(3, 5), cv::Vec3b(7, 8, 9));
  EXPECT_FLOAT_EQ(snapshot.cloud->front().x, 1);
  EXPECT_EQ(snapshot.odom_from_body.translation, Eigen::Vector3d(1, 2, 3));
}

TEST(KeyframeTest, LidarSnapshotRebuildsGeometryWithAnIndependentPose) {
  auto source = test_data::KeyframeResult(UpdateKind::kLidarOnly);
  PointCloud::Ptr points(new PointCloud(*source.world_cloud));
  source.world_cloud = points;
  const auto snapshot = CaptureKeyframe(0, source);
  points->front().x = 100;
  points->clear();
  EXPECT_FALSE(snapshot.image_timestamp);
  EXPECT_FALSE(snapshot.camera_from_odom);
  EXPECT_TRUE(snapshot.image.empty());
  RigidTransform optimized;
  optimized.translation << 10, 0, 0;
  const auto global = TransformKeyframeCloud(snapshot, optimized);
  ASSERT_EQ(global.size(), 1u);
  EXPECT_FLOAT_EQ(global[0].x, 11);
  EXPECT_FLOAT_EQ(global[0].y, 0);
  EXPECT_FLOAT_EQ(global[0].z, 0);
  EXPECT_FLOAT_EQ(global[0].intensity, 42);
  EXPECT_FLOAT_EQ(snapshot.cloud->front().x, 1);
  EXPECT_EQ(snapshot.odom_from_body.translation, Eigen::Vector3d(1, 2, 3));
}

TEST(KeyframeTest, RejectsUnsuccessfulOrMalformedSourceData) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const std::vector<std::function<void(FrontendResult&)>> corrupt = {
      [](auto& r) { r.updated = false; },
      [](auto& r) { r.world_cloud.reset(); },
      [](auto& r) { r.world_cloud.reset(new PointCloud); },
      [=](auto& r) { r.timestamp = nan; },
      [=](auto& r) { r.cloud_timestamp = nan; },
      [=](auto& r) { r.estimate.pos_end.x() = nan; },
      [](auto& r) { r.estimate.rot_end.setZero(); },
      [=](auto& r) { r.estimate.inv_expo_time = nan; },
      [](auto& r) { r.color_image = cv::Mat(4, 6, CV_8UC1); },
      [](auto& r) { r.image_timestamp.reset(); },
      [=](auto& r) { r.image_timestamp = nan; },
      [](auto& r) { r.camera_from_world_rotation.coeffs().setZero(); }};
  for (const auto& change : corrupt) {
    auto result = test_data::KeyframeResult();
    change(result);
    EXPECT_THROW(CaptureKeyframe(0, result), std::invalid_argument);
  }
}

TEST(KeyframeTest, RejectsNonfinitePointsAndInvalidDestinationTransform) {
  auto result = test_data::KeyframeResult(UpdateKind::kLidarOnly);
  PointCloud::Ptr points(new PointCloud(*result.world_cloud));
  result.world_cloud = points;
  points->front().z = std::numeric_limits<float>::infinity();
  EXPECT_THROW(CaptureKeyframe(0, result), std::invalid_argument);
  points->front().z = 3;
  const auto snapshot = CaptureKeyframe(0, result);
  RigidTransform destination;
  destination.rotation.setZero();
  EXPECT_THROW(TransformKeyframeCloud(snapshot, destination),
               std::invalid_argument);
}
}  // namespace
}  // namespace livo_atlas
