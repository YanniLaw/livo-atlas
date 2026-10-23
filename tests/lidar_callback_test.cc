#include "LIVMapper.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <ros/package.h>


namespace livo_atlas {
namespace {

std::filesystem::path OutputDirectory() {
  return std::filesystem::temp_directory_path() /
         ("livo_ros_" + std::to_string(ros::WallTime::now().toNSec()));
}

std::string ReadText(const std::filesystem::path& path) {
  std::ifstream stream(path);
  return std::string(std::istreambuf_iterator<char>(stream), {});
}


class LidarCallbackTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ros::NodeHandle node;
    output_directory_ = OutputDirectory();
    node.setParam("output/directory", output_directory_.string());
    mapper_ = std::make_unique<LIVMapper>(node);
    mapper_->p_pre->set(false, kLivox, 1, 1);
    mapper_->p_pre->blind_sqr = 1;
  }

  void TearDown() override {
    mapper_.reset();
    if (!output_directory_.empty()) {
      std::filesystem::remove_all(output_directory_);
    }
  }

  void ExpectOriginalQueue(const PointCloud::Ptr& original) {
    ASSERT_EQ(mapper_->sensor_buffers_.point_clouds.size(), 1);
    EXPECT_EQ(mapper_->sensor_buffers_.point_clouds.front().cloud, original);
    EXPECT_DOUBLE_EQ(mapper_->sensor_buffers_.point_clouds.front().timestamp, 20);
    EXPECT_DOUBLE_EQ(mapper_->last_timestamp_lidar, 20);
  }

  void ExpectRolledBackQueue() {
    ASSERT_EQ(mapper_->sensor_buffers_.point_clouds.size(), 1);
    EXPECT_FLOAT_EQ(mapper_->sensor_buffers_.point_clouds.front().cloud->front().x, 4);
    EXPECT_DOUBLE_EQ(mapper_->sensor_buffers_.point_clouds.front().timestamp, 12);
    EXPECT_DOUBLE_EQ(mapper_->last_timestamp_lidar, 12);
  }

  std::unique_ptr<LIVMapper> mapper_;
  std::filesystem::path output_directory_;
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
  ASSERT_EQ(mapper_->sensor_buffers_.point_clouds.size(), 1);
  const auto original = mapper_->sensor_buffers_.point_clouds.front().cloud;
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
  ASSERT_EQ(mapper_->sensor_buffers_.point_clouds.size(), 1);
  const auto original = mapper_->sensor_buffers_.point_clouds.front().cloud;
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
  void CreateMapper(SLAM_MODE mode, int initialization_frames = 3) {
    ros::NodeHandle node;
    output_directory_ = OutputDirectory();
    node.setParam("output/directory", output_directory_.string());
    node.setParam("imu/imu_int_frame", initialization_frames);
    node.setParam("common/lidar_en", 1);
    node.setParam("common/img_en", mode == LIVO ? 1 : 0);
    node.setParam("imu/imu_en", mode != ONLY_LO);
    mapper_ = std::make_unique<LIVMapper>(node);
    ASSERT_EQ(mapper_->slam_mode_, mode);
  }

  void TearDown() override {
    mapper_.reset();
    if (!output_directory_.empty()) {
      std::filesystem::remove_all(output_directory_);
    }
  }

  void AddCloud(double timestamp, std::initializer_list<float> offsets) {
    PointCloud::Ptr cloud(new PointCloud);
    for (float offset : offsets) {
      Point point;
      point.x = 2 + cloud->size();
      point.curvature = offset;
      cloud->push_back(point);
    }
    mapper_->sensor_buffers_.point_clouds.push_back({timestamp, cloud});
  }

  void AddImu(double timestamp) {
    mapper_->sensor_buffers_.imu.push_back(
        {timestamp, Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, -9.81)});
    mapper_->sensor_buffers_.latest_imu_time = timestamp;
  }

  void AddImage(double timestamp) {
    mapper_->sensor_buffers_.images.push_back(
        {timestamp, cv::Mat(1, 1, CV_8UC3, cv::Scalar(7, 8, 9))});
  }

  std::unique_ptr<LIVMapper> mapper_;
  std::filesystem::path output_directory_;
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
  mapper_->sensor_buffers_.latest_imu_time =
      std::nextafter(100.125, -std::numeric_limits<double>::infinity());
  LidarMeasureGroup measurements;
  EXPECT_FALSE(mapper_->sync_packages(measurements));
  mapper_->sensor_buffers_.latest_imu_time = 100.125;
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

TEST_F(SynchronizerIntegrationTest, ImuCallbackPreservesCoverageAndRollback) {
  CreateMapper(ONLY_LIO);
  mapper_->ros_driver_fix_en = true;
  mapper_->last_timestamp_lidar = 0.975;
  AddCloud(0.975, {0, 125});
  sensor_msgs::Imu::Ptr message(new sensor_msgs::Imu);
  message->header.stamp = ros::Time(100, 100000000);
  mapper_->imu_cbk(message);
  ASSERT_EQ(mapper_->sensor_buffers_.imu.size(), 1);
  EXPECT_DOUBLE_EQ(mapper_->sensor_buffers_.imu.front().timestamp, 1.1);
  EXPECT_LT(mapper_->sensor_buffers_.latest_imu_time, 1.1);
  LidarMeasureGroup measurements;
  EXPECT_FALSE(mapper_->sync_packages(measurements));

  message->header.stamp = ros::Time(100, 250000000);
  mapper_->imu_cbk(message);
  ASSERT_EQ(mapper_->sensor_buffers_.imu.size(), 2);
  EXPECT_DOUBLE_EQ(mapper_->sensor_buffers_.latest_imu_time, 1.25);
  message->header.stamp = ros::Time(100, 0);
  mapper_->imu_cbk(message);
  EXPECT_EQ(mapper_->sensor_buffers_.imu.size(), 2);
  EXPECT_DOUBLE_EQ(mapper_->sensor_buffers_.latest_imu_time, 1.25);
  EXPECT_DOUBLE_EQ(mapper_->last_timestamp_imu, 1.25);
  ASSERT_TRUE(mapper_->sync_packages(measurements));
  ASSERT_EQ(measurements.measures.back().imu.size(), 1);
  EXPECT_DOUBLE_EQ(measurements.measures.back().imu.front().timestamp, 1.1);
}

TEST_F(SynchronizerIntegrationTest, LidarOnlyPipelineKeepsAnchorAndWorldPoints) {
  ros::NodeHandle node;
  node.setParam("extrin_calib/extrinsic_T", std::vector<double>{0, 0, 0});
  node.setParam("extrin_calib/extrinsic_R",
                std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1});
  CreateMapper(ONLY_LO);
  image_transport::ImageTransport transport(node);
  mapper_->initializeSubscribersAndPublishers(node, transport);
  for (double timestamp : {100.0, 100.25}) {
    AddCloud(timestamp, {0, 125});
    ASSERT_TRUE(mapper_->sync_packages(mapper_->LidarMeasures));
    mapper_->ProcessMeasurements();
    EXPECT_DOUBLE_EQ(mapper_->LidarMeasures.last_lio_update_time,
                     timestamp + 0.125);
    EXPECT_DOUBLE_EQ((mapper_->last_result_.timestamp - mapper_->last_result_.relative_time), -1.0);
    ASSERT_EQ(mapper_->last_result_.world_cloud->size(), 2);
    EXPECT_FLOAT_EQ(mapper_->last_result_.world_cloud->front().x, 2);
    EXPECT_FLOAT_EQ(mapper_->last_result_.world_cloud->back().x, 3);
    EXPECT_TRUE(mapper_->frontend_->state().pos_end.isZero(0));
  }
}

TEST_F(SynchronizerIntegrationTest, InitialImuOutputDoesNotBuildMap) {
  CreateMapper(ONLY_LIO);
  AddCloud(100, {0, 125});
  AddImu(100.0625);
  AddImu(100.125);
  ASSERT_TRUE(mapper_->sync_packages(mapper_->LidarMeasures));
  mapper_->ProcessMeasurements();
  EXPECT_TRUE(mapper_->last_result_.undistorted_cloud->empty());
  EXPECT_FALSE(mapper_->last_result_.updated);
  EXPECT_FALSE(mapper_->last_result_.world_cloud);
  EXPECT_TRUE(mapper_->frontend_->state().gravity.isApprox(Eigen::Vector3d(0, 0, 9.81)));
}

TEST_F(SynchronizerIntegrationTest, VisualCloudIsConsumedWithoutPublishing) {
  CreateMapper(LIVO, 1);
  const auto& camera = mapper_->frontend_options_.calibration.camera;
  const auto add_image = [&](double timestamp) {
    mapper_->sensor_buffers_.images.push_back(
        {timestamp, cv::Mat(static_cast<int>(camera.height * camera.scale),
                            static_cast<int>(camera.width * camera.scale),
                            CV_8UC3, cv::Scalar(7, 8, 9))});
  };
  AddCloud(100, {0, 125});
  for (double timestamp : {100.0, 100.0625, 100.125, 100.25, 100.5, 100.75}) {
    AddImu(timestamp);
  }
  add_image(100.125);
  auto& measurements = mapper_->LidarMeasures;
  auto& frontend = *mapper_->frontend_;
  ASSERT_TRUE(mapper_->sync_packages(measurements));
  EXPECT_FALSE(frontend.Process(&measurements).updated);
  AddCloud(100.25, {0, 125});
  AddCloud(100.625, {0, 125});
  ASSERT_TRUE(mapper_->sync_packages(measurements));
  EXPECT_FALSE(frontend.Process(&measurements).updated);

  add_image(100.5);
  ASSERT_TRUE(mapper_->sync_packages(measurements));
  ASSERT_EQ(measurements.lio_vio_flg, LIO);
  ASSERT_TRUE(frontend.Process(&measurements).updated);
  ASSERT_TRUE(mapper_->sync_packages(measurements));
  ASSERT_EQ(measurements.lio_vio_flg, VIO);
  const auto visual = frontend.Process(&measurements);
  ASSERT_TRUE(visual.updated);
  ASSERT_TRUE(visual.world_cloud);
  EXPECT_FALSE(visual.world_cloud->empty());
  ASSERT_FALSE(visual.color_image.empty());
  ASSERT_FALSE(visual.tracking_image.empty());
  EXPECT_EQ(visual.color_image.at<cv::Vec3b>(0, 0), cv::Vec3b(7, 8, 9));
  // Preserve the quaternion from the rounded calibration without normalizing.
  EXPECT_NEAR(visual.camera_from_world_rotation.norm(), 1, 1e-6);

  // No publisher runs. The next valid image window has no LiDAR points.
  add_image(100.5625);
  ASSERT_TRUE(mapper_->sync_packages(measurements));
  ASSERT_EQ(measurements.lio_vio_flg, LIO);
  EXPECT_FALSE(frontend.Process(&measurements).updated);
  ASSERT_TRUE(mapper_->sync_packages(measurements));
  ASSERT_EQ(measurements.lio_vio_flg, VIO);
  EXPECT_FALSE(frontend.Process(&measurements).updated);
}

TEST_F(SynchronizerIntegrationTest, OutputUsesConfiguredDirectory) {
  const auto source_log =
      std::filesystem::path(ros::package::getPath("livo_atlas")) / "Log";
  const auto original = ReadText(source_log / "mat_pre.txt");
  CreateMapper(ONLY_LO);
  ros::NodeHandle node;
  image_transport::ImageTransport transport(node);
  mapper_->initializeSubscribersAndPublishers(node, transport);
  AddCloud(100, {0, 125});
  ASSERT_TRUE(mapper_->sync_packages(mapper_->LidarMeasures));
  mapper_->ProcessMeasurements();
  EXPECT_FALSE(ReadText(output_directory_ / "mat_pre.txt").empty());
  EXPECT_FALSE(ReadText(output_directory_ / "mat_out.txt").empty());
  EXPECT_EQ(ReadText(source_log / "mat_pre.txt"), original);
}




}  // namespace
}  // namespace livo_atlas

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_callback_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
