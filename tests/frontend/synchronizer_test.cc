#include "livo_atlas/sensor/synchronizer.h"

#include <cmath>
#include <initializer_list>
#include <limits>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

namespace livo_atlas {
namespace {

PointCloudMeasurement MakeCloud(double time,
                                std::initializer_list<float> offsets) {
  PointCloudMeasurement measurement;
  measurement.timestamp = time;
  measurement.cloud.reset(new PointCloud);
  for (float offset : offsets) {
    Point point{};
    point.x = 2.0f + measurement.cloud->size();
    point.curvature = offset;
    measurement.cloud->push_back(point);
  }
  return measurement;
}

class SynchronizerTest : public ::testing::Test {
 protected:
  void AddImu(double time) {
    ImuMeasurement measurement;
    measurement.timestamp = time;
    buffers_.imu.push_back(measurement);
    buffers_.latest_imu_time = time;
  }

  void AddImage(double time) {
    buffers_.images.push_back(
        {time, cv::Mat(1, 1, CV_8UC3, cv::Scalar(7, 8, 9))});
  }

  void AddLivoInputs() {
    buffers_.point_clouds.push_back(MakeCloud(100.0, {0, 125, 250}));
    AddImu(100.0);
    AddImu(100.0625);
    AddImu(100.125);
    AddImu(100.25);
    AddImage(100.125);
  }

  SynchronizationResult Run(
      FrontendMode mode = FrontendMode::kLidarInertialVisual,
      double exposure = 0.0) {
    return SensorSynchronizer(mode, exposure)
        .TrySynchronize(&buffers_, &output_);
  }

  SensorBuffers buffers_;
  SynchronizedMeasurements output_;
};

TEST_F(SynchronizerTest, WaitsForRequiredInputsWithoutConsuming) {
  EXPECT_EQ(Run(), SynchronizationResult::kWaiting);
  buffers_.point_clouds.push_back(MakeCloud(100.0, {0, 125}));
  EXPECT_EQ(Run(), SynchronizationResult::kWaiting);
  EXPECT_EQ(Run(FrontendMode::kLidarInertial),
            SynchronizationResult::kWaiting);
  AddImu(100.125);
  EXPECT_EQ(Run(), SynchronizationResult::kWaiting);
  EXPECT_EQ(buffers_.point_clouds.size(), 1);
  EXPECT_EQ(buffers_.imu.size(), 1);
}

TEST_F(SynchronizerTest, LidarOnlyUsesLastPointWithoutImuOrImage) {
  buffers_.point_clouds.push_back(MakeCloud(100.0, {0, 250, 125}));
  ASSERT_EQ(Run(FrontendMode::kLidarOnly), SynchronizationResult::kReady);
  EXPECT_EQ(output_.lio_vio_flg, UpdateKind::kLidarOnly);
  EXPECT_DOUBLE_EQ(output_.lidar_frame_end_time, 100.125);
  ASSERT_EQ(output_.measures.size(), 1);
  EXPECT_DOUBLE_EQ(output_.measures.front().lio_time, 100.125);
  EXPECT_EQ(output_.lidar->size(), 3);
  EXPECT_TRUE(buffers_.point_clouds.empty());
}

TEST_F(SynchronizerTest, LioWaitsForUnroundedCoverageAndIncludesEndImu) {
  buffers_.point_clouds.push_back(MakeCloud(100.0, {0, 125}));
  AddImu(99.875);
  AddImu(100.125);
  AddImu(100.25);
  buffers_.latest_imu_time = std::nextafter(100.125, 0.0);
  EXPECT_EQ(Run(FrontendMode::kLidarInertial),
            SynchronizationResult::kWaiting);
  ASSERT_EQ(buffers_.imu.size(), 3);
  buffers_.latest_imu_time = 100.125;
  ASSERT_EQ(Run(FrontendMode::kLidarInertial),
            SynchronizationResult::kReady);
  ASSERT_EQ(output_.measures.size(), 1);
  const auto& group = output_.measures.front();
  ASSERT_EQ(group.imu.size(), 2);
  EXPECT_DOUBLE_EQ(group.imu.front().timestamp, 99.875);
  EXPECT_DOUBLE_EQ(group.imu.back().timestamp, 100.125);
  ASSERT_EQ(buffers_.imu.size(), 1);
  EXPECT_DOUBLE_EQ(buffers_.imu.front().timestamp, 100.25);
  EXPECT_DOUBLE_EQ(output_.last_lio_update_time, 100.0);
}

TEST_F(SynchronizerTest, RepeatedLioWaitKeepsFirstScan) {
  buffers_.point_clouds.push_back(MakeCloud(100.0, {0, 125}));
  const auto first = buffers_.point_clouds.front().cloud;
  AddImu(100.0625);
  EXPECT_EQ(Run(FrontendMode::kLidarInertial),
            SynchronizationResult::kWaiting);
  buffers_.point_clouds.push_back(MakeCloud(100.25, {0, 125}));
  EXPECT_EQ(Run(FrontendMode::kLidarInertial),
            SynchronizationResult::kWaiting);
  AddImu(100.125);
  ASSERT_EQ(Run(FrontendMode::kLidarInertial),
            SynchronizationResult::kReady);
  EXPECT_EQ(output_.lidar, first);
  EXPECT_EQ(output_.pcl_proc_cur, first);
  ASSERT_EQ(buffers_.point_clouds.size(), 1);
  EXPECT_DOUBLE_EQ(buffers_.point_clouds.front().timestamp, 100.25);
}

TEST_F(SynchronizerTest, LivoCutsPointsStrictlyAndImuInclusively) {
  AddLivoInputs();
  ASSERT_EQ(Run(), SynchronizationResult::kReady);
  EXPECT_EQ(output_.lio_vio_flg, UpdateKind::kLidarInertial);
  ASSERT_EQ(output_.pcl_proc_cur->size(), 1);
  EXPECT_FLOAT_EQ(output_.pcl_proc_cur->front().x, 2);
  ASSERT_EQ(output_.pcl_proc_next->size(), 2);
  EXPECT_FLOAT_EQ(output_.pcl_proc_next->front().curvature, 0);
  EXPECT_FLOAT_EQ(output_.pcl_proc_next->back().curvature, 125);
  ASSERT_EQ(output_.measures.size(), 1);
  ASSERT_EQ(output_.measures.front().imu.size(), 2);
  EXPECT_DOUBLE_EQ(output_.measures.front().imu.front().timestamp, 100.0625);
  EXPECT_DOUBLE_EQ(output_.measures.front().imu.back().timestamp, 100.125);
  ASSERT_EQ(buffers_.imu.size(), 1);
  EXPECT_DOUBLE_EQ(buffers_.imu.front().timestamp, 100.25);
  EXPECT_DOUBLE_EQ(output_.last_lio_update_time, 100.0);
  EXPECT_EQ(buffers_.images.size(), 1);
}

TEST_F(SynchronizerTest, ExposureShiftsImageWindowOnly) {
  AddLivoInputs();
  buffers_.images.front().timestamp = 100.0;
  ASSERT_EQ(Run(FrontendMode::kLidarInertialVisual, 0.125),
            SynchronizationResult::kReady);
  ASSERT_EQ(output_.measures.size(), 1);
  EXPECT_DOUBLE_EQ(output_.measures.front().lio_time, 100.125);
  EXPECT_DOUBLE_EQ(buffers_.images.front().timestamp, 100.0);
  EXPECT_EQ(output_.pcl_proc_cur->size(), 1);
}

TEST_F(SynchronizerTest, DropsOnlyOneImageBelowStaleThreshold) {
  AddLivoInputs();
  output_.last_lio_update_time = 100.0;
  buffers_.images.front().timestamp = std::nextafter(100.00001, 0.0);
  AddImage(100.125);
  EXPECT_EQ(Run(), SynchronizationResult::kImageDropped);
  ASSERT_EQ(buffers_.images.size(), 1);
  EXPECT_DOUBLE_EQ(buffers_.images.front().timestamp, 100.125);
  EXPECT_EQ(buffers_.point_clouds.size(), 1);
  EXPECT_EQ(buffers_.imu.size(), 4);
}

TEST_F(SynchronizerTest, AcceptsImageAtStaleThreshold) {
  AddLivoInputs();
  output_.last_lio_update_time = 100.0;
  buffers_.images.front().timestamp = 100.0 + 0.00001;
  ASSERT_EQ(Run(), SynchronizationResult::kReady);
  EXPECT_EQ(output_.pcl_proc_cur->size(), 1);
}

TEST_F(SynchronizerTest, IncompleteCoverageConsumesNothing) {
  AddLivoInputs();
  buffers_.point_clouds.back().cloud->back().curvature = 62.5;
  EXPECT_EQ(Run(), SynchronizationResult::kWaiting);
  EXPECT_EQ(buffers_.point_clouds.size(), 1);
  EXPECT_EQ(buffers_.images.size(), 1);
  EXPECT_EQ(buffers_.imu.size(), 4);
  buffers_.point_clouds.back().cloud->back().curvature = 250;
  buffers_.imu.pop_back();
  buffers_.imu.pop_back();
  EXPECT_EQ(Run(), SynchronizationResult::kWaiting);
  EXPECT_EQ(buffers_.imu.size(), 2);
  EXPECT_TRUE(output_.measures.empty());
  AddImu(100.125);
  EXPECT_EQ(Run(), SynchronizationResult::kReady);
}

TEST_F(SynchronizerTest, MultipleScansKeepOrderAndWindowOffsets) {
  buffers_.point_clouds.push_back(MakeCloud(100.0, {0, 250}));
  buffers_.point_clouds.push_back(MakeCloud(100.125, {0, 125}));
  AddImu(100.25);
  AddImage(100.1875);
  ASSERT_EQ(Run(), SynchronizationResult::kReady);
  ASSERT_EQ(output_.pcl_proc_cur->size(), 2);
  EXPECT_FLOAT_EQ(output_.pcl_proc_cur->front().curvature, 0);
  EXPECT_FLOAT_EQ(output_.pcl_proc_cur->back().curvature, 125);
  ASSERT_EQ(output_.pcl_proc_next->size(), 2);
  EXPECT_FLOAT_EQ(output_.pcl_proc_next->front().curvature, 62.5);
  EXPECT_FLOAT_EQ(output_.pcl_proc_next->back().curvature, 62.5);
  EXPECT_TRUE(buffers_.point_clouds.empty());
}

TEST_F(SynchronizerTest, RemainderIsCopiedIntoNextLioWithoutRepartition) {
  AddLivoInputs();
  ASSERT_EQ(Run(), SynchronizationResult::kReady);
  output_.last_lio_update_time = 100.125;
  buffers_.point_clouds.push_back(MakeCloud(100.25, {0, 125}));
  ASSERT_EQ(Run(), SynchronizationResult::kReady);
  EXPECT_EQ(output_.lio_vio_flg, UpdateKind::kVisual);
  AddImage(100.25);
  ASSERT_EQ(Run(), SynchronizationResult::kReady);
  ASSERT_EQ(output_.pcl_proc_cur->size(), 2);
  EXPECT_FLOAT_EQ(output_.pcl_proc_cur->front().x, 3);
  EXPECT_FLOAT_EQ(output_.pcl_proc_cur->back().x, 4);
  EXPECT_FLOAT_EQ(output_.pcl_proc_cur->back().curvature, 125);
  EXPECT_EQ(output_.pcl_proc_next->size(), 2);
}

TEST_F(SynchronizerTest, VisualOutputRetainsImageAfterQueuePop) {
  AddLivoInputs();
  ASSERT_EQ(Run(), SynchronizationResult::kReady);
  output_.last_lio_update_time = 100.125;
  buffers_.point_clouds.push_back(MakeCloud(100.25, {0, 125}));
  ASSERT_EQ(Run(), SynchronizationResult::kReady);
  ASSERT_EQ(output_.measures.size(), 1);
  const auto& group = output_.measures.front();
  EXPECT_EQ(group.img.at<cv::Vec3b>(0, 0), cv::Vec3b(7, 8, 9));
  EXPECT_DOUBLE_EQ(group.vio_time, 100.125);
  EXPECT_DOUBLE_EQ(group.lio_time, 100.125);
  EXPECT_TRUE(buffers_.images.empty());
  EXPECT_EQ(buffers_.imu.size(), 1);
}

TEST_F(SynchronizerTest, PendingVisualStillNeedsCloudQueue) {
  AddLivoInputs();
  ASSERT_EQ(Run(), SynchronizationResult::kReady);
  output_.last_lio_update_time = 100.125;
  EXPECT_EQ(Run(), SynchronizationResult::kWaiting);
  EXPECT_EQ(buffers_.images.size(), 1);
  buffers_.point_clouds.push_back(MakeCloud(100.25, {0, 125}));
  ASSERT_EQ(Run(), SynchronizationResult::kReady);
  EXPECT_EQ(output_.lio_vio_flg, UpdateKind::kVisual);
}

TEST_F(SynchronizerTest, PendingVisualStillNeedsImuQueue) {
  AddLivoInputs();
  buffers_.imu.pop_back();
  ASSERT_EQ(Run(), SynchronizationResult::kReady);
  output_.last_lio_update_time = 100.125;
  buffers_.point_clouds.push_back(MakeCloud(100.25, {0, 125}));
  EXPECT_EQ(Run(), SynchronizationResult::kWaiting);
  AddImu(100.25);
  ASSERT_EQ(Run(), SynchronizationResult::kReady);
  EXPECT_EQ(output_.lio_vio_flg, UpdateKind::kVisual);
}

TEST_F(SynchronizerTest, SinglePointLioIsRemovedBeforeChoosingTimeAnchor) {
  buffers_.point_clouds.push_back(MakeCloud(99.0, {0}));
  buffers_.point_clouds.push_back(MakeCloud(100.0, {0, 125}));
  AddImu(100.125);
  EXPECT_EQ(Run(FrontendMode::kLidarInertial),
            SynchronizationResult::kInvalidCloud);
  EXPECT_DOUBLE_EQ(output_.last_lio_update_time, -1.0);
  ASSERT_EQ(Run(FrontendMode::kLidarInertial),
            SynchronizationResult::kReady);
  EXPECT_DOUBLE_EQ(output_.last_lio_update_time, 100.0);
}

TEST_F(SynchronizerTest, InvalidCloudEndpointsAreRemoved) {
  for (bool at_front : {false, true}) {
    for (bool null_cloud : {false, true}) {
      SCOPED_TRACE(::testing::Message() << at_front << ',' << null_cloud);
      buffers_.point_clouds.clear();
      buffers_.point_clouds.push_back(MakeCloud(100.0, {0, 125}));
      auto invalid = MakeCloud(99.0, {});
      if (null_cloud) invalid.cloud.reset();
      if (at_front) {
        buffers_.point_clouds.push_front(invalid);
      } else {
        buffers_.point_clouds.push_back(invalid);
      }
      ASSERT_EQ(Run(FrontendMode::kLidarOnly),
                SynchronizationResult::kInvalidCloud);
      ASSERT_EQ(buffers_.point_clouds.size(), 1);
      ASSERT_EQ(Run(FrontendMode::kLidarOnly),
                SynchronizationResult::kReady);
      EXPECT_DOUBLE_EQ(output_.lidar_frame_end_time, 100.125);
    }
  }
}

TEST_F(SynchronizerTest, InvalidInteriorCloudIsSkippedDuringCut) {
  buffers_.point_clouds.push_back(MakeCloud(100.0, {0, 125}));
  buffers_.point_clouds.push_back({100.0625, {}});
  buffers_.point_clouds.push_back(MakeCloud(100.125, {0, 125}));
  AddImu(100.25);
  AddImage(100.1875);
  ASSERT_EQ(Run(), SynchronizationResult::kReady);
  EXPECT_EQ(output_.pcl_proc_cur->size(), 3);
  EXPECT_EQ(output_.pcl_proc_next->size(), 1);
  EXPECT_TRUE(buffers_.point_clouds.empty());
}

TEST_F(SynchronizerTest, CuttingDoesNotRewriteInputCloud) {
  AddLivoInputs();
  const auto input = buffers_.point_clouds.front().cloud;
  output_.last_lio_update_time = 99.875;
  ASSERT_EQ(Run(), SynchronizationResult::kReady);
  EXPECT_FLOAT_EQ(output_.pcl_proc_cur->front().curvature, 125);
  EXPECT_FLOAT_EQ(input->front().curvature, 0);
  EXPECT_FLOAT_EQ(input->at(1).curvature, 125);
  EXPECT_FLOAT_EQ(input->back().curvature, 250);
}

TEST_F(SynchronizerTest, LivoCoverageUsesQueuedImuTime) {
  AddLivoInputs();
  buffers_.imu.pop_back();
  buffers_.imu.pop_back();
  buffers_.latest_imu_time = 200.0;
  EXPECT_EQ(Run(), SynchronizationResult::kWaiting);
  AddImu(100.125);
  buffers_.latest_imu_time = 99.0;
  EXPECT_EQ(Run(), SynchronizationResult::kReady);
}

TEST_F(SynchronizerTest, LidarOnlyAcceptsSinglePoint) {
  buffers_.point_clouds.push_back(MakeCloud(100.0, {125}));
  ASSERT_EQ(Run(FrontendMode::kLidarOnly), SynchronizationResult::kReady);
  EXPECT_DOUBLE_EQ(output_.lidar_frame_end_time, 100.125);
  EXPECT_EQ(output_.lidar->size(), 1);
}

}  // namespace
}  // namespace livo_atlas
