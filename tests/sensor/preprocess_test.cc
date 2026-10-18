#include "preprocess.h"

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <iterator>
#include <new>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "livo_atlas/sensor/lidar_scan.h"

namespace livo_atlas {
namespace {

class SensorPreprocessTest : public ::testing::Test {
 protected:
  void SetUp() override {
    preprocess_.feature_enabled = false;
    preprocess_.blind_sqr = 1;
    preprocess_.point_filter_num = 1;
  }

  Preprocess preprocess_;
};

TEST_F(SensorPreprocessTest, LivoxKeepsFilteringTimingRepairAndOutputOwnership) {
  std::vector<LivoxPoint> points(4);
  for (auto& point : points) {
    point.x = 2;
    point.reflectivity = 17;
  }
  points[1].offset_time = 4167;
  points[2].offset_time = 2000000;
  points[3].line = 6;  // Excluded by the original six-line configuration.
  PointCloud first;
  preprocess_.Process(LidarScan{17, points}, &first);
  ASSERT_EQ(first.size(), 3);
  EXPECT_FLOAT_EQ(first[0].curvature, 0);
  EXPECT_FLOAT_EQ(first[1].curvature, 0.004167f);
  EXPECT_FLOAT_EQ(first[2].curvature, 0.008333667f);
  EXPECT_FLOAT_EQ(first[1].intensity, 17);
  preprocess_.point_filter_num = 2;
  PointCloud second;
  preprocess_.Process(LidarScan{18, points}, &second);
  ASSERT_EQ(second.size(), 1);
  EXPECT_FLOAT_EQ(second[0].curvature, 0.004167f);
  EXPECT_EQ(first.size(), 3);
}

TEST_F(SensorPreprocessTest, OusterConvertsNanosecondsAndSorts) {
  pcl::PointCloud<OusterPoint> points;
  OusterPoint point{};
  point.x = 2;
  point.t = 2500000;
  points.push_back(point);
  point.t = 1000000;
  points.push_back(point);
  PointCloud result;
  preprocess_.Process(LidarScan{17, points}, &result);
  ASSERT_EQ(result.size(), 2);
  EXPECT_FLOAT_EQ(result[0].curvature, 1);
  EXPECT_FLOAT_EQ(result[1].curvature, 2.5);
}

TEST_F(SensorPreprocessTest, VelodyneRetainsOriginalTimeDivision) {
  pcl::PointCloud<VelodynePoint> points;
  VelodynePoint point{};
  point.x = 2;
  point.time = 2500;
  points.push_back(point);
  PointCloud result;
  preprocess_.Process(LidarScan{17, points}, &result);
  ASSERT_EQ(result.size(), 1);
  EXPECT_FLOAT_EQ(result[0].curvature, 2.5);
}

template <typename PointType>
PointCloud ProcessTimestampCloud(Preprocess* preprocess) {
  pcl::PointCloud<PointType> points;
  PointType point{};
  point.x = 2;
  point.timestamp = 1000;
  point.intensity = 255;
  points.push_back(point);
  point.timestamp = 1000.0025;
  points.push_back(point);
  PointCloud result;
  preprocess->Process(LidarScan{17, points}, &result);
  return result;
}

TEST_F(SensorPreprocessTest, Xt32UsesDifferenceFromFirstPoint) {
  const auto result = ProcessTimestampCloud<Xt32Point>(&preprocess_);
  ASSERT_EQ(result.size(), 2);
  EXPECT_FLOAT_EQ(result[0].curvature, 0);
  EXPECT_FLOAT_EQ(result[1].curvature, 2.5);
  EXPECT_FLOAT_EQ(result[1].intensity, 255);
}

TEST_F(SensorPreprocessTest, Pandar128NormalizesByteIntensity) {
  const auto result = ProcessTimestampCloud<Pandar128Point>(&preprocess_);
  ASSERT_EQ(result.size(), 2);
  EXPECT_FLOAT_EQ(result[1].curvature, 2.5);
  EXPECT_FLOAT_EQ(result[1].intensity, 1);
}

TEST_F(SensorPreprocessTest, RobosenseRetainsFloatIntensity) {
  const auto result = ProcessTimestampCloud<RobosensePoint>(&preprocess_);
  ASSERT_EQ(result.size(), 2);
  EXPECT_FLOAT_EQ(result[1].curvature, 2.5);
  EXPECT_FLOAT_EQ(result[1].intensity, 255);
}

TEST_F(SensorPreprocessTest, DepthCloudKeepsRgbAndHasZeroOffsets) {
  pcl::PointCloud<pcl::PointXYZRGB> points;
  pcl::PointXYZRGB point;
  point.x = 2;
  point.r = 1;
  point.g = 2;
  point.b = 3;
  points.push_back(point);
  PointCloud result;
  preprocess_.Process(LidarScan{17, points}, &result);
  ASSERT_EQ(result.size(), 1);
  EXPECT_FLOAT_EQ(result[0].normal_x, 1);
  EXPECT_FLOAT_EQ(result[0].normal_y, 2);
  EXPECT_FLOAT_EQ(result[0].normal_z, 3);
  EXPECT_FLOAT_EQ(result[0].curvature, 0);
}

TEST_F(SensorPreprocessTest, EmptyInputClearsPreviousOutputForEveryLayout) {
  std::vector<LidarPoints> layouts = {
      std::vector<LivoxPoint>{}, pcl::PointCloud<VelodynePoint>{},
      pcl::PointCloud<OusterPoint>{}, pcl::PointCloud<pcl::PointXYZRGB>{},
      pcl::PointCloud<Xt32Point>{}, pcl::PointCloud<Pandar128Point>{},
      pcl::PointCloud<RobosensePoint>{}};
  for (auto& points : layouts) {
    PointCloud result;
    result.resize(1);
    preprocess_.Process(LidarScan{17, std::move(points)}, &result);
    EXPECT_TRUE(result.empty());
  }
}

TEST(SensorPreprocessDefaultsTest, BlindRadiusIgnoresPreviousStorageBytes) {
  alignas(Preprocess) std::byte storage[sizeof(Preprocess)];
  std::fill(std::begin(storage), std::end(storage), std::byte{0x7f});
  auto* preprocess = new (storage) Preprocess;
  EXPECT_DOUBLE_EQ(preprocess->blind_sqr, 0.0001);
  preprocess->~Preprocess();
}

TEST_F(SensorPreprocessTest, SetUpdatesBlindRadiusUsedForFiltering) {
  preprocess_.set(false, kOuster, 3, 1);
  pcl::PointCloud<OusterPoint> points;
  OusterPoint point{};
  point.x = 2;
  points.push_back(point);
  point.x = 4;
  points.push_back(point);
  PointCloud result;
  preprocess_.Process(LidarScan{17, points}, &result);
  ASSERT_EQ(result.size(), 1);
  EXPECT_FLOAT_EQ(result[0].x, 4);
}

TEST_F(SensorPreprocessTest, InvalidRingsCannotIndexYawFallbackArrays) {
  for (bool feature_enabled : {false, true}) {
    EXPECT_EXIT({
      preprocess_.feature_enabled = feature_enabled;
      preprocess_.N_SCANS = 128;
      pcl::PointCloud<VelodynePoint> points;
      VelodynePoint point{};
      point.x = 2;
      point.ring = feature_enabled ? 80 : 65535;
      points.push_back(point);
      point.ring = 0;
      points.push_back(point);
      point.y = -1;
      points.push_back(point);
      PointCloud result;
      preprocess_.Process(LidarScan{17, points}, &result);
      const bool valid = feature_enabled ? preprocess_.pl_buff[80].empty()
                                         : result.size() == 1;
      std::_Exit(valid ? 0 : 1);
    }, ::testing::ExitedWithCode(0), "");
  }
  // XT32 uses the same fixed-size yaw fallback when timestamp is absent.
  EXPECT_EXIT({
    preprocess_.feature_enabled = true;
    preprocess_.N_SCANS = 128;
    pcl::PointCloud<Xt32Point> points;
    Xt32Point point{};
    point.x = 2;
    point.ring = 80;
    points.push_back(point);
    PointCloud result;
    preprocess_.Process(LidarScan{17, points}, &result);
    std::_Exit(preprocess_.pl_buff[80].empty() ? 0 : 1);
  }, ::testing::ExitedWithCode(0), "");
}

TEST_F(SensorPreprocessTest, SparseOusterFeatureLinesReturnSafely) {
  EXPECT_EXIT({
    preprocess_.feature_enabled = true;
    pcl::PointCloud<OusterPoint> points;
    OusterPoint point{};
    point.x = 2;
    points.push_back(point);
    PointCloud result;
    preprocess_.Process(LidarScan{17, points}, &result);
    std::_Exit(result.empty() ? 0 : 1);
  }, ::testing::ExitedWithCode(0), "");
}

TEST_F(SensorPreprocessTest, AllBlindFeatureLineReturnsSafely) {
  EXPECT_EXIT({
    preprocess_.feature_enabled = true;
    std::vector<LivoxPoint> points(10);
    for (std::size_t i = 0; i < points.size(); ++i) {
      points[i].x = 0.01f * (i + 1);
      points[i].tag = 0x10;
    }
    PointCloud result;
    preprocess_.Process(LidarScan{17, points}, &result);
    std::_Exit(result.empty() ? 0 : 1);
  }, ::testing::ExitedWithCode(0), "");
}

}  // namespace
}  // namespace livo_atlas
