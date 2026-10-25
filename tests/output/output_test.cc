#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <vector>

#include "livo_atlas/output/output_processor.h"

namespace livo_atlas {
namespace {

OutputProcessorOptions Options() {
  OutputProcessorOptions options;
  options.calibration.camera = {"Pinhole", 16, 16, 1.0, 4, 4, 8, 8, {}};
  return options;
}

FrontendResult Frame(const std::vector<Eigen::Vector3d>& positions) {
  FrontendResult result;
  result.kind = UpdateKind::kVisual;
  result.updated = true;
  PointCloud::Ptr cloud(new PointCloud);
  for (const auto& position : positions) {
    Point point;
    point.x = position.x();
    point.y = position.y();
    point.z = position.z();
    cloud->push_back(point);
  }
  result.world_cloud = cloud;
  result.color_image = cv::Mat(16, 16, CV_8UC3, cv::Scalar(10, 20, 30));
  return result;
}

TEST(OutputProcessorTest, BilinearColorUsesBgrAndRespectsImageStride) {
  OutputProcessor processor(Options());
  auto result = Frame({{0.125, 0.125, 1}});
  cv::Mat padded(16, 20, CV_8UC3);
  result.color_image = padded(cv::Rect(0, 0, 16, 16));
  ASSERT_FALSE(result.color_image.isContinuous());
  for (int y = 0; y < 16; ++y) {
    for (int x = 0; x < 16; ++x) {
      result.color_image.at<cv::Vec3b>(y, x) =
          cv::Vec3b(2 * x + 4 * y, 20 + x, 40 + y);
    }
  }
  const auto output = processor.Process(result);
  ASSERT_TRUE(output.has_cloud);
  ASSERT_TRUE(output.colored_cloud);
  ASSERT_EQ(output.colored_cloud->size(), 1);
  const auto& point = output.colored_cloud->front();
  EXPECT_EQ(point.b, 51);
  EXPECT_EQ(point.g, 28);
  EXPECT_EQ(point.r, 48);
  EXPECT_FLOAT_EQ(point.x, 0.125);
}

TEST(OutputProcessorTest, RejectsBehindBorderAndNearPoints) {
  OutputProcessor processor(Options());
  const auto output = processor.Process(
      Frame({{0, 0, 1}, {0, 0, -1}, {2, 0, 1}, {0, 0, 0.005}}));
  ASSERT_TRUE(output.colored_cloud);
  ASSERT_EQ(output.colored_cloud->size(), 1);
  EXPECT_FLOAT_EQ(output.colored_cloud->front().z, 1);
}

TEST(OutputProcessorTest, ProjectionUsesCameraPoseAndRetainsWorldCoordinates) {
  OutputProcessor processor(Options());
  auto result = Frame({{1, 0, 1}});
  result.camera_from_world_rotation = Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5);
  result.camera_from_world_translation << 0, 0, 1;
  result.estimate.pos_end << 100, 200, 300;
  const auto output = processor.Process(result);
  ASSERT_TRUE(output.colored_cloud);
  ASSERT_EQ(output.colored_cloud->size(), 1);
  EXPECT_FLOAT_EQ(output.colored_cloud->front().x, 1);
  EXPECT_FLOAT_EQ(output.colored_cloud->front().z, 1);
}

TEST(OutputProcessorTest, MixedLidarVisualCadenceAccumulatesUntilThreshold) {
  auto options = Options();
  options.scan_interval = 5;
  OutputProcessor processor(options);
  auto lidar = Frame({{0, 0, 1}});
  lidar.kind = UpdateKind::kLidarInertial;
  auto first = processor.Process(lidar);
  EXPECT_TRUE(first.has_cloud);
  first = processor.Process(Frame({{0, 0, 1}}));
  ASSERT_TRUE(first.colored_cloud);
  EXPECT_TRUE(first.colored_cloud->empty());
  processor.Process(lidar);
  const auto second = processor.Process(Frame({{0, 0, 2}}));
  ASSERT_TRUE(second.colored_cloud);
  EXPECT_EQ(second.colored_cloud->size(), 2);
}

TEST(OutputProcessorTest, EmptyProjectionRetainsAccumulatedWorldPoints) {
  OutputProcessor processor(Options());
  auto result = Frame({{0, 0, -1}});
  const auto first = processor.Process(result);
  ASSERT_TRUE(first.colored_cloud);
  EXPECT_TRUE(first.colored_cloud->empty());
  result.camera_from_world_rotation = Eigen::Quaterniond(0, 0, 1, 0);
  const auto second = processor.Process(result);
  ASSERT_TRUE(second.colored_cloud);
  EXPECT_EQ(second.colored_cloud->size(), 2);
}

TEST(OutputProcessorTest, InstancesHaveIndependentCadenceAndAccumulation) {
  auto options = Options();
  options.scan_interval = 3;
  OutputProcessor first(options);
  OutputProcessor second(options);
  auto lidar = Frame({{0, 0, 1}});
  lidar.kind = UpdateKind::kLidarInertial;
  first.Process(lidar);
  second.Process(lidar);
  const auto a = first.Process(Frame({{0, 0, 1}}));
  const auto b = second.Process(Frame({{0, 0, 2}}));
  ASSERT_TRUE(a.colored_cloud);
  ASSERT_TRUE(b.colored_cloud);
  ASSERT_EQ(a.colored_cloud->size(), 1);
  ASSERT_EQ(b.colored_cloud->size(), 1);
  EXPECT_FLOAT_EQ(a.colored_cloud->front().z, 1);
  EXPECT_FLOAT_EQ(b.colored_cloud->front().z, 2);
}

TEST(OutputProcessorTest, SkipsFailedAndEmptyFrames) {
  OutputProcessor processor(Options());
  auto failed = Frame({{0, 0, 1}});
  failed.updated = false;
  EXPECT_FALSE(processor.Process(failed).has_cloud);
  EXPECT_FALSE(processor.Process(Frame({})).has_cloud);
  EXPECT_TRUE(processor.Process(Frame({{0, 0, 1}})).has_cloud);
}

TEST(OutputProcessorTest, ColmapImageUsesExistingCameraUndistortion) {
  auto options = Options();
  options.colmap = true;
  OutputProcessor processor(options);
  auto result = Frame({{0, 0, 1}});
  const auto output = processor.Process(result);
  ASSERT_FALSE(output.colmap_image.empty());
  EXPECT_EQ(output.colmap_image.size(), result.color_image.size());
  EXPECT_EQ(cv::norm(output.colmap_image, result.color_image, cv::NORM_INF), 0);
}

TEST(OutputProcessorTest, RejectsNonpositiveCadenceAndUnsupportedColmapModel) {
  auto options = Options();
  options.scan_interval = 0;
  EXPECT_THROW(OutputProcessor processor(options), std::invalid_argument);
  options = Options();
  options.colmap = true;
  options.calibration.camera.model = "Equidistant";
  EXPECT_THROW(OutputProcessor processor(options), std::invalid_argument);
}

}  // namespace
}  // namespace livo_atlas
