#include "LIVMapper.h"

#include <memory>

#include <gtest/gtest.h>
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

}  // namespace
}  // namespace livo_atlas

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_callback_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
