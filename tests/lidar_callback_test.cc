#include "LIVMapper.h"

#include <cmath>
#include <initializer_list>
#include <limits>
#include <memory>

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

namespace livo_atlas {
namespace {

class LidarCallbackTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ros::NodeHandle node;
    mapper_ = std::make_unique<LIVMapper>(node);
    mapper_->p_pre->set(false, kLivox, 1, 1);
    mapper_->p_pre->blind_sqr = 1;
  }

  void ExpectOriginalQueue(const PointCloud::Ptr& original) {
    ASSERT_EQ(mapper_->lid_raw_data_buffer.size(), 1);
    EXPECT_EQ(mapper_->lid_raw_data_buffer.front(), original);
    ASSERT_EQ(mapper_->lid_header_time_buffer.size(), 1);
    EXPECT_DOUBLE_EQ(mapper_->lid_header_time_buffer.front(), 20);
    EXPECT_DOUBLE_EQ(mapper_->last_timestamp_lidar, 20);
  }

  void ExpectRolledBackQueue() {
    ASSERT_EQ(mapper_->lid_raw_data_buffer.size(), 1);
    EXPECT_FLOAT_EQ(mapper_->lid_raw_data_buffer.front()->front().x, 4);
    ASSERT_EQ(mapper_->lid_header_time_buffer.size(), 1);
    EXPECT_DOUBLE_EQ(mapper_->lid_header_time_buffer.front(), 12);
    EXPECT_DOUBLE_EQ(mapper_->last_timestamp_lidar, 12);
  }

  std::unique_ptr<LIVMapper> mapper_;
};

livox_ros_driver::CustomMsg::Ptr LivoxMessage(double time, float x) {
  livox_ros_driver::CustomMsg::Ptr message(new livox_ros_driver::CustomMsg);
  message->header.stamp.fromSec(time);
  message->point_num = 1;
  message->points.resize(1);
  message->points[0].x = x;
  return message;
}

sensor_msgs::PointCloud2::Ptr StandardMessage(double time, float x) {
  pcl::PointCloud<VelodynePoint> points;
  VelodynePoint point{};
  point.x = x;
  point.time = 2500;
  points.push_back(point);
  sensor_msgs::PointCloud2::Ptr message(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(points, *message);
  message->header.stamp.fromSec(time);
  return message;
}

TEST_F(LidarCallbackTest, LivoxValidatesBeforeChangingPairedRollbackQueues) {
  mapper_->livox_pcl_cbk(LivoxMessage(20, 2));
  ASSERT_EQ(mapper_->lid_raw_data_buffer.size(), 1);
  const auto original = mapper_->lid_raw_data_buffer.front();
  auto malformed = LivoxMessage(10, 4);
  malformed->point_num = 2;
  mapper_->livox_pcl_cbk(malformed);
  ExpectOriginalQueue(original);
  auto empty = LivoxMessage(11, 0);
  empty->point_num = 0;
  empty->points.clear();
  mapper_->livox_pcl_cbk(empty);
  ExpectOriginalQueue(original);
  mapper_->livox_pcl_cbk(LivoxMessage(12, 4));
  ExpectRolledBackQueue();
}

TEST_F(LidarCallbackTest, StandardValidatesBeforeChangingPairedRollbackQueues) {
  mapper_->p_pre->lidar_type = kVelodyne;
  mapper_->standard_pcl_cbk(StandardMessage(20, 2));
  ASSERT_EQ(mapper_->lid_raw_data_buffer.size(), 1);
  const auto original = mapper_->lid_raw_data_buffer.front();
  auto malformed = StandardMessage(10, 4);
  malformed->data.pop_back();
  mapper_->standard_pcl_cbk(malformed);
  ExpectOriginalQueue(original);
  auto empty = StandardMessage(11, 0);
  empty->width = 0;
  empty->row_step = 0;
  empty->data.clear();
  mapper_->standard_pcl_cbk(empty);
  ExpectOriginalQueue(original);
  mapper_->standard_pcl_cbk(StandardMessage(12, 4));
  ExpectRolledBackQueue();
}


class SynchronizerIntegrationTest : public ::testing::Test {
 protected:
  void CreateMapper(SLAM_MODE mode) {
    ros::NodeHandle node;
    node.setParam("common/lidar_en", 1);
    node.setParam("common/img_en", mode == LIVO ? 1 : 0);
    node.setParam("imu/imu_en", mode != ONLY_LO);
    mapper_ = std::make_unique<LIVMapper>(node);
    ASSERT_EQ(mapper_->slam_mode_, mode);
  }

  void AddCloud(double timestamp, std::initializer_list<float> offsets) {
    PointCloud::Ptr cloud(new PointCloud);
    for (float offset : offsets) {
      Point point;
      point.x = 2 + cloud->size();
      point.curvature = offset;
      cloud->push_back(point);
    }
    mapper_->lid_raw_data_buffer.push_back(cloud);
    mapper_->lid_header_time_buffer.push_back(timestamp);
  }

  void AddImu(double timestamp) {
    mapper_->imu_buffer.push_back(
        {timestamp, Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, -9.81)});
    mapper_->last_timestamp_imu = timestamp;
  }

  void AddImage(double timestamp) {
    mapper_->img_buffer.emplace_back(1, 1, CV_8UC3, cv::Scalar(7, 8, 9));
    mapper_->img_time_buffer.push_back(timestamp);
  }

  std::unique_ptr<LIVMapper> mapper_;
};

TEST_F(SynchronizerIntegrationTest, LivoCutsBoundaryAndAlternatesSameImage) {
  CreateMapper(LIVO);
  AddCloud(100, {0, 125, 250});
  for (double timestamp : {100.0, 100.0625, 100.125, 100.25}) {
    AddImu(timestamp);
  }
  AddImage(100.125);
  LidarMeasureGroup measurements;
  ASSERT_TRUE(mapper_->sync_packages(measurements));
  EXPECT_EQ(measurements.lio_vio_flg, LIO);
  EXPECT_DOUBLE_EQ(measurements.last_lio_update_time, 100);
  ASSERT_EQ(measurements.pcl_proc_cur->size(), 1);
  EXPECT_FLOAT_EQ(measurements.pcl_proc_cur->front().x, 2);
  ASSERT_EQ(measurements.pcl_proc_next->size(), 2);
  EXPECT_FLOAT_EQ((*measurements.pcl_proc_next)[0].curvature, 0);
  EXPECT_FLOAT_EQ((*measurements.pcl_proc_next)[1].curvature, 125);
  ASSERT_EQ(measurements.measures.back().imu.size(), 2);
  EXPECT_DOUBLE_EQ(measurements.measures.back().imu.front().timestamp, 100.0625);
  EXPECT_DOUBLE_EQ(measurements.measures.back().imu.back().timestamp, 100.125);

  // The estimator advances this time; synchronization itself does not.
  measurements.last_lio_update_time = 100.125;
  EXPECT_FALSE(mapper_->sync_packages(measurements));
  AddCloud(100.25, {0, 125});
  ASSERT_TRUE(mapper_->sync_packages(measurements));
  EXPECT_EQ(measurements.lio_vio_flg, VIO);
  EXPECT_DOUBLE_EQ(measurements.measures.back().vio_time, 100.125);
  EXPECT_EQ(measurements.measures.back().img.at<cv::Vec3b>(0, 0),
            cv::Vec3b(7, 8, 9));

  AddImage(100.25);
  ASSERT_TRUE(mapper_->sync_packages(measurements));
  EXPECT_EQ(measurements.lio_vio_flg, LIO);
  ASSERT_EQ(measurements.pcl_proc_cur->size(), 2);
  EXPECT_FLOAT_EQ((*measurements.pcl_proc_cur)[0].x, 3);
  EXPECT_FLOAT_EQ((*measurements.pcl_proc_cur)[1].x, 4);
  EXPECT_FLOAT_EQ((*measurements.pcl_proc_cur)[1].curvature, 125);
}

TEST_F(SynchronizerIntegrationTest, LioUsesCoverageBeforeFinalRosRounding) {
  CreateMapper(ONLY_LIO);
  AddCloud(100, {0, 125});
  AddImu(100.0625);
  AddImu(100.125);
  AddImu(100.25);
  mapper_->last_timestamp_imu =
      std::nextafter(100.125, -std::numeric_limits<double>::infinity());
  LidarMeasureGroup measurements;
  EXPECT_FALSE(mapper_->sync_packages(measurements));
  mapper_->last_timestamp_imu = 100.125;
  ASSERT_TRUE(mapper_->sync_packages(measurements));
  EXPECT_EQ(measurements.lio_vio_flg, LIO);
  EXPECT_DOUBLE_EQ(measurements.measures.back().lio_time, 100.125);
  ASSERT_EQ(measurements.measures.back().imu.size(), 2);
  EXPECT_DOUBLE_EQ(measurements.measures.back().imu.back().timestamp, 100.125);
}

TEST_F(SynchronizerIntegrationTest, LidarOnlyNeedsNeitherImuNorImage) {
  CreateMapper(ONLY_LO);
  AddCloud(100, {0, 125});
  LidarMeasureGroup measurements;
  ASSERT_TRUE(mapper_->sync_packages(measurements));
  EXPECT_EQ(measurements.lio_vio_flg, LO);
  EXPECT_DOUBLE_EQ(measurements.measures.back().lio_time, 100.125);
  ASSERT_EQ(measurements.lidar->size(), 2);
  EXPECT_FLOAT_EQ(measurements.lidar->back().curvature, 125);
}


}  // namespace
}  // namespace livo_atlas

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_callback_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
