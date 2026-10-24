/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIVO_ATLAS_LIV_MAPPER_H_
#define LIVO_ATLAS_LIV_MAPPER_H_

#include <condition_variable>
#include <memory>
#include <mutex>

#include <image_transport/image_transport.h>
#include <livox_ros_driver/CustomMsg.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include "common_lib.h"
#include "livo_atlas/frontend/frontend.h"
#include "livo_atlas/io/result_writer.h"
#include "livo_atlas/ros1/result_publisher.h"
#include "livo_atlas/sensor/synchronizer.h"
#include "preprocess.h"

class LIVMapper {
 public:
  explicit LIVMapper(ros::NodeHandle& node);
  ~LIVMapper();
  void initializeSubscribersAndPublishers(
      ros::NodeHandle& node, image_transport::ImageTransport& transport);
  void run();
  void InitializeFrontend();
  void InitializeOutputs();
  void ProcessMeasurements();
  bool sync_packages(LidarMeasureGroup& measurements);
  void prop_imu_once(StatesGroup& state, double dt, V3D acceleration,
                     V3D angular_velocity);
  void imu_prop_callback(const ros::TimerEvent& event);
  void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr& message);
  void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr& message);
  void imu_cbk(const sensor_msgs::Imu::ConstPtr& message);
  void img_cbk(const sensor_msgs::ImageConstPtr& message);
  void readParameters(ros::NodeHandle& node);
  void ValidateImageParameters() const;

  std::mutex mtx_buffer, mtx_buffer_imu_prop;
  std::condition_variable sig_buffer;
  SLAM_MODE slam_mode_;
  std::string lid_topic, imu_topic, img_topic;
  double last_timestamp_lidar = -1.0;
  double last_timestamp_imu = -1.0;
  double last_timestamp_img = -1.0;
  bool ros_driver_fix_en = false;
  bool hilti_en = false;

  StatesGroup imu_propagate, latest_ekf_state;
  bool new_imu = false;
  bool state_update_flg = false;
  bool imu_prop_enable = true;
  bool ekf_finish_once = false;
  std::deque<livo_atlas::ImuMeasurement> prop_imu_buffer;
  livo_atlas::ImuMeasurement newest_imu;
  ros::Time newest_imu_stamp_;
  double latest_ekf_time = 0.0;
  double last_time_from_lidar_end_ = 0.0;
  double imu_time_offset = 0.0;
  double lidar_time_offset = 0.0;
  bool imu_en = false;
  int img_en = 1;
  int lidar_en = 1;
  double exposure_time_init = 0.0;
  double img_time_offset = 0.0;

  livo_atlas::SensorBuffers sensor_buffers_;
  livo_atlas::SensorSynchronizer synchronizer_;
  livo_atlas::Calibration calibration_;
  livo_atlas::FrontendOptions frontend_options_;
  std::unique_ptr<livo_atlas::Frontend> frontend_;
  livo_atlas::FrontendResult last_result_;
  livo_atlas::OutputOptions output_options_;
  livo_atlas::OutputProcessorOptions output_processor_options_;
  std::unique_ptr<livo_atlas::OutputProcessor> output_processor_;
  std::unique_ptr<livo_atlas::ResultWriter> result_writer_;
  std::unique_ptr<livo_atlas::ros1::ResultPublisher> result_publisher_;
  LidarMeasureGroup LidarMeasures;
  PreprocessPtr p_pre;
  ros::Subscriber sub_pcl, sub_imu, sub_img;
  ros::Timer imu_prop_timer;
};

#endif  // LIVO_ATLAS_LIV_MAPPER_H_
