/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "LIVMapper.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <utility>

#include <cv_bridge/cv_bridge.h>

#include "livo_atlas/ros1/calibration_reader.h"
#include "livo_atlas/ros1/measurement_conversions.h"
#include "livo_atlas/ros1/voxel_options_reader.h"

namespace {

std::filesystem::path DefaultOutputDirectory() {
  const char* ros_home = std::getenv("ROS_HOME");
  if (ros_home != nullptr && *ros_home != '\0') {
    return std::filesystem::path(ros_home) / "livo_atlas";
  }
  const char* user_home = std::getenv("HOME");
  if (user_home != nullptr && *user_home != '\0') {
    return std::filesystem::path(user_home) / ".ros/livo_atlas";
  }
  return std::filesystem::current_path() / "livo_atlas_output";
}

}  // namespace

LIVMapper::LIVMapper(ros::NodeHandle& node) {
  p_pre = std::make_shared<Preprocess>();
  readParameters(node);
  ValidateImageParameters();
  frontend_options_.voxel_map = livo_atlas::ros1::ReadVoxelOptions(node);
  InitializeFrontend();
  InitializeOutputs();
}

LIVMapper::~LIVMapper() = default;

void LIVMapper::readParameters(ros::NodeHandle &nh)
{
  auto& visual_options = frontend_options_.visual;
  auto& imu_options = frontend_options_.imu;
  nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
  nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");
  nh.param<bool>("common/ros_driver_bug_fix", ros_driver_fix_en, false);
  nh.param<int>("common/img_en", img_en, 1);
  nh.param<int>("common/lidar_en", lidar_en, 1);
  nh.param<string>("common/img_topic", img_topic, "/left_camera/image");

  nh.param<bool>("vio/normal_en", visual_options.normal_enabled, true);
  nh.param<bool>("vio/inverse_composition_en",
                 visual_options.inverse_composition, false);
  nh.param<int>("vio/max_iterations", visual_options.max_iterations, 5);
  nh.param<double>("vio/img_point_cov",
                   visual_options.image_point_covariance, 100);
  nh.param<bool>("vio/raycast_en", visual_options.raycast, false);
  nh.param<bool>("vio/exposure_estimate_en",
                 visual_options.estimate_exposure, true);
  nh.param<double>("vio/inv_expo_cov",
                   imu_options.inverse_exposure_covariance, 0.2);
  nh.param<int>("vio/grid_size", visual_options.grid_size, 5);
  nh.param<int>("vio/grid_n_height", visual_options.grid_rows, 17);
  nh.param<int>("vio/patch_pyrimid_level",
                visual_options.patch_pyramid_levels, 3);
  nh.param<int>("vio/patch_size", visual_options.patch_size, 8);
  nh.param<double>("vio/outlier_threshold",
                   visual_options.outlier_threshold, 1000);

  nh.param<double>("time_offset/exposure_time_init", exposure_time_init, 0.0);
  nh.param<double>("time_offset/img_time_offset", img_time_offset, 0.0);
  nh.param<double>("time_offset/imu_time_offset", imu_time_offset, 0.0);
  nh.param<double>("time_offset/lidar_time_offset", lidar_time_offset, 0.0);
  nh.param<bool>("uav/imu_rate_odom", imu_prop_enable, false);
  nh.param<bool>("uav/gravity_align_en",
                 frontend_options_.align_gravity, false);

  nh.param<string>("evo/seq_name", output_options_.sequence_name, "01");
  nh.param<bool>("evo/pose_output_en", output_options_.trajectory, false);
  nh.param<double>("imu/gyr_cov", imu_options.gyroscope_covariance, 1.0);
  nh.param<double>("imu/acc_cov", imu_options.accelerometer_covariance, 1.0);
  nh.param<int>("imu/imu_int_frame", imu_options.initialization_frames, 3);
  nh.param<bool>("imu/imu_en", imu_en, false);
  nh.param<bool>("imu/gravity_est_en", imu_options.estimate_gravity, true);
  nh.param<bool>("imu/ba_bg_est_en", imu_options.estimate_bias, true);

  nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
  nh.param<double>("preprocess/filter_size_surf",
                   frontend_options_.surface_filter_size, 0.5);
  nh.param<bool>("preprocess/hilti_en", hilti_en, false);
  nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
  nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 6);
  nh.param<int>("preprocess/point_filter_num", p_pre->point_filter_num, 3);
  nh.param<bool>("preprocess/feature_extract_enabled",
                 p_pre->feature_enabled, false);

  nh.param<int>("pcd_save/interval", output_options_.pcd_interval, -1);
  nh.param<bool>("pcd_save/pcd_save_en", output_options_.pcd, false);
  nh.param<int>("pcd_save/type", output_options_.pcd_type, 0);
  nh.param<bool>("image_save/img_save_en", output_options_.images, false);
  nh.param<int>("image_save/interval", output_options_.image_interval, 1);

  nh.param<bool>("pcd_save/colmap_output_en", output_options_.colmap, false);
  nh.param<double>("pcd_save/filter_size_pcd", output_options_.voxel_size, 0.5);
  nh.param<double>("debug/plot_time", visual_options.plot_time, -10);
  nh.param<int>("debug/frame_cnt", visual_options.plot_frame_count, 6);

  nh.param<double>("publish/blind_rgb_points",
                   output_processor_options_.minimum_color_distance, 0.01);
  nh.param<int>("publish/pub_scan_num",
                output_processor_options_.scan_interval, 1);
  nh.param<bool>("publish/pub_effect_point_en",
                 frontend_options_.output_effective_points, false);
  nh.param<bool>("publish/dense_map_en", frontend_options_.dense_cloud, false);

  std::string directory;
  if (nh.hasParam("output/directory")) {
    if (!nh.getParam("output/directory", directory)) {
      throw std::invalid_argument("output/directory must be a string");
    }
  } else {
    directory = DefaultOutputDirectory().string();
  }
  output_options_.directory = directory;

  p_pre->blind_sqr = p_pre->blind * p_pre->blind;

  const ros::NodeHandle camera_node("~");
  const std::string error =
      livo_atlas::ros1::ReadCalibration(nh, camera_node, &calibration_);
  if (!error.empty()) {
    throw std::invalid_argument(error);
  }
}

void LIVMapper::ValidateImageParameters() const {
  // Camera validation has already checked these conversions and index bounds.
  const auto& camera = calibration_.camera;
  const auto& visual_options = frontend_options_.visual;
  const int width = static_cast<int>(camera.width * camera.scale);
  const int height = static_cast<int>(camera.height * camera.scale);
  int cell_size = visual_options.grid_size;
  if (cell_size <= 0) {
    throw std::invalid_argument("vio/grid_size must be positive");
  }
  if (cell_size <= 10) {
    if (visual_options.grid_rows <= 0) {
      throw std::invalid_argument("vio/grid_n_height must be positive");
    }
    cell_size = height / visual_options.grid_rows;
  }
  if (cell_size == 0 || width / cell_size == 0 || height / cell_size == 0) {
    throw std::invalid_argument("VIO grid is incompatible with camera dimensions");
  }
  if (visual_options.patch_size <= 0 || visual_options.patch_size % 2 != 0) {
    throw std::invalid_argument("vio/patch_size must be a positive even number");
  }
  // Search can add two to the largest pyramid index (levels - 1).
  if (visual_options.patch_pyramid_levels <= 0 || visual_options.patch_pyramid_levels > 28) {
    throw std::invalid_argument("vio/patch_pyrimid_level is outside the index range");
  }
  constexpr int kMaxIndex = std::numeric_limits<int>::max();
  const std::int64_t patch_area = std::int64_t{visual_options.patch_size} * visual_options.patch_size;
  if (patch_area > kMaxIndex / visual_options.patch_pyramid_levels) {
    throw std::invalid_argument("vio/patch_size exceeds the patch index range");
  }
  const std::int64_t scale = std::int64_t{1} << (visual_options.patch_pyramid_levels + 1);
  if ((visual_options.patch_size / 2 + 1) > kMaxIndex / scale || width > kMaxIndex / scale) {
    throw std::invalid_argument("vio/patch_pyrimid_level exceeds image index bounds");
  }
  // Preserve VIO's floor grid counts. Truncated tails must be excluded by its
  // image border, so every accepted pixel has an allocated row and column.
  const std::int64_t border = (visual_options.patch_size / 2 + 1) * (scale / 2);
  if (width % cell_size > border || height % cell_size > border) {
    throw std::invalid_argument("VIO grid does not cover the accepted image area");
  }
  if (output_options_.colmap && camera.model != "Pinhole") {
    throw std::invalid_argument("pcd_save/colmap_output_en requires a Pinhole camera");
  }
}

void LIVMapper::InitializeFrontend() {
  frontend_options_.calibration = calibration_;
  frontend_options_.imu.enabled = imu_en;
  frontend_ = std::make_unique<livo_atlas::Frontend>(frontend_options_);
  slam_mode_ = (img_en && lidar_en) ? LIVO : imu_en ? ONLY_LIO : ONLY_LO;
  synchronizer_ = livo_atlas::SensorSynchronizer(slam_mode_, exposure_time_init);
}

void LIVMapper::InitializeOutputs() {
  output_options_.calibration = calibration_;
  output_options_.mode = slam_mode_;
  output_processor_options_.calibration = calibration_;
  output_processor_options_.mode = slam_mode_;
  output_processor_options_.colmap = output_options_.colmap;
  livo_atlas::ValidateOutputOptions(output_options_);
  output_processor_ = std::make_unique<livo_atlas::OutputProcessor>(
      output_processor_options_);
  result_writer_ = std::make_unique<livo_atlas::ResultWriter>(output_options_);
  result_publisher_ =
      std::make_unique<livo_atlas::ros1::ResultPublisher>(slam_mode_);
}

void LIVMapper::initializeSubscribersAndPublishers(
    ros::NodeHandle& node, image_transport::ImageTransport& transport) {
  if (p_pre->lidar_type == AVIA) {
    sub_pcl = node.subscribe(lid_topic, 200000, &LIVMapper::livox_pcl_cbk, this);
  } else {
    sub_pcl = node.subscribe(lid_topic, 200000, &LIVMapper::standard_pcl_cbk,
                             this);
  }
  sub_imu = node.subscribe(imu_topic, 200000, &LIVMapper::imu_cbk, this);
  sub_img = node.subscribe(img_topic, 200000, &LIVMapper::img_cbk, this);
  result_publisher_->Initialize(node, transport);
  imu_prop_timer = node.createTimer(ros::Duration(0.004),
                                    &LIVMapper::imu_prop_callback, this);
}

void LIVMapper::ProcessMeasurements() {
  const bool imu_was_initialized = last_result_.imu_initialized;
  const auto start = std::chrono::steady_clock::now();
  last_result_ = frontend_->Process(&LidarMeasures);
  const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();
  if (last_result_.first_frame) std::cout << "FIRST LIDAR FRAME!" << std::endl;
  if (!imu_was_initialized && last_result_.imu_initialized) {
    ROS_INFO("IMU initialization completed");
  }
  if (last_result_.updated && imu_prop_enable) {
    ekf_finish_once = true;
    latest_ekf_state = last_result_.estimate;
    latest_ekf_time = last_result_.timestamp;
    state_update_flg = true;
  }
  const auto prepared = output_processor_->Process(last_result_);
  result_writer_->Write(last_result_, prepared);
  if (!last_result_.updated) return;
  result_publisher_->Publish(last_result_, prepared);
  const char* stage = last_result_.kind == VIO ? "VIO Time" : "LIO Mapping Time";
  std::printf("%s\n| Current Total Time | %.6f |\n", stage, elapsed);
}

void LIVMapper::run() 
{
  ros::Rate rate(5000);
  while (ros::ok()) 
  {
    ros::spinOnce();
    if (!sync_packages(LidarMeasures)) 
    {
      rate.sleep();
      continue;
    }
    ProcessMeasurements();
  }
  result_writer_->Finish();
}

void LIVMapper::prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr)
{
  double mean_acc_norm = frontend_->imu_mean_acceleration_norm();
  acc_avr = acc_avr * G_m_s2 / mean_acc_norm - imu_prop_state.bias_a;
  angvel_avr -= imu_prop_state.bias_g;

  M3D Exp_f = Exp(angvel_avr, dt);
  /* propogation of IMU attitude */
  imu_prop_state.rot_end = imu_prop_state.rot_end * Exp_f;

  /* Specific acceleration (global frame) of IMU */
  V3D acc_imu = imu_prop_state.rot_end * acc_avr + V3D(imu_prop_state.gravity[0], imu_prop_state.gravity[1], imu_prop_state.gravity[2]);

  /* propogation of IMU */
  imu_prop_state.pos_end = imu_prop_state.pos_end + imu_prop_state.vel_end * dt + 0.5 * acc_imu * dt * dt;

  /* velocity of IMU */
  imu_prop_state.vel_end = imu_prop_state.vel_end + acc_imu * dt;
}

void LIVMapper::imu_prop_callback(const ros::TimerEvent &e)
{
  if (frontend_->imu_needs_initialization() || !new_imu || !ekf_finish_once) { return; }
  mtx_buffer_imu_prop.lock();
  new_imu = false; // 控制propagate频率和IMU频率一致
  if (imu_prop_enable && !prop_imu_buffer.empty())
  {
    static double last_t_from_lidar_end_time = 0;
    if (state_update_flg)
    {
      imu_propagate = latest_ekf_state;
      // drop all useless imu pkg
      while ((!prop_imu_buffer.empty() && prop_imu_buffer.front().timestamp < latest_ekf_time))
      {
        prop_imu_buffer.pop_front();
      }
      last_t_from_lidar_end_time = 0;
      for (int i = 0; i < prop_imu_buffer.size(); i++)
      {
        double t_from_lidar_end_time = prop_imu_buffer[i].timestamp - latest_ekf_time;
        double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
        // cout << "prop dt" << dt << ", " << t_from_lidar_end_time << ", " << last_t_from_lidar_end_time << endl;
        V3D acc_imu(prop_imu_buffer[i].linear_acceleration.x(), prop_imu_buffer[i].linear_acceleration.y(), prop_imu_buffer[i].linear_acceleration.z());
        V3D omg_imu(prop_imu_buffer[i].angular_velocity.x(), prop_imu_buffer[i].angular_velocity.y(), prop_imu_buffer[i].angular_velocity.z());
        prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
        last_t_from_lidar_end_time = t_from_lidar_end_time;
      }
      state_update_flg = false;
    }
    else
    {
      V3D acc_imu(newest_imu.linear_acceleration.x(), newest_imu.linear_acceleration.y(), newest_imu.linear_acceleration.z());
      V3D omg_imu(newest_imu.angular_velocity.x(), newest_imu.angular_velocity.y(), newest_imu.angular_velocity.z());
      double t_from_lidar_end_time = newest_imu.timestamp - latest_ekf_time;
      double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
      prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
      last_t_from_lidar_end_time = t_from_lidar_end_time;
    }

    result_publisher_->PublishImuPropagation(imu_propagate,
                                               newest_imu_stamp_);
  }
  mtx_buffer_imu_prop.unlock();
}

void LIVMapper::standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  if (!lidar_en) return;
  std::unique_lock<std::mutex> lock(mtx_buffer);

  double cur_head_time = msg->header.stamp.toSec() + lidar_time_offset;
  // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
  livo_atlas::LidarScan scan;
  const std::string error = livo_atlas::ros1::ConvertPointCloud(
      *msg, p_pre->lidar_type, lidar_time_offset, &scan);
  if (!error.empty()) {
    ROS_ERROR_STREAM(error);
    lock.unlock();
    return;
  }
  livo_atlas::PointCloudMeasurement measurement{
      scan.timestamp, PointCloudXYZI::Ptr(new PointCloudXYZI())};
  p_pre->Process(scan, measurement.cloud.get());
  if (measurement.cloud->empty()) {
    ROS_ERROR("Received an empty point cloud");
    lock.unlock();
    return;
  }
  if (cur_head_time < last_timestamp_lidar) {
    ROS_ERROR("lidar loop back, clear buffer");
    sensor_buffers_.point_clouds.clear();
  }
  sensor_buffers_.point_clouds.push_back(std::move(measurement));
  last_timestamp_lidar = cur_head_time;

  lock.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in)
{
  if (!lidar_en) return;
  std::unique_lock<std::mutex> lock(mtx_buffer);
  const auto& msg = msg_in;
  // if ((abs(msg->header.stamp.toSec() - last_timestamp_lidar) > 0.2 && last_timestamp_lidar > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("lidar jumps %.3f\n", msg->header.stamp.toSec() - last_timestamp_lidar);
  //   sync_jump_flag = true;
  //   msg->header.stamp = ros::Time().fromSec(last_timestamp_lidar + 0.1);
  // }
  if (abs(last_timestamp_imu - msg->header.stamp.toSec()) > 1.0 && !sensor_buffers_.imu.empty())
  {
    double timediff_imu_wrt_lidar = last_timestamp_imu - msg->header.stamp.toSec();
    printf("\033[95mSelf sync IMU and LiDAR, HARD time lag is %.10lf \n\033[0m", timediff_imu_wrt_lidar - 0.100);
    // imu_time_offset = timediff_imu_wrt_lidar;
  }

  double cur_head_time = msg->header.stamp.toSec();
  ROS_INFO("Get LiDAR, its header time: %.6f", cur_head_time);
  // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
  livo_atlas::LidarScan scan;
  const std::string error = livo_atlas::ros1::ConvertLivox(*msg, &scan);
  if (!error.empty()) {
    ROS_ERROR_STREAM(error);
    lock.unlock();
    return;
  }
  livo_atlas::PointCloudMeasurement measurement{
      scan.timestamp, PointCloudXYZI::Ptr(new PointCloudXYZI())};
  p_pre->Process(scan, measurement.cloud.get());

  if (measurement.cloud->empty()) {
    ROS_ERROR("Received an empty point cloud");
    lock.unlock();
    return;
  }

  if (cur_head_time < last_timestamp_lidar) {
    ROS_ERROR("lidar loop back, clear buffer");
    sensor_buffers_.point_clouds.clear();
  }
  sensor_buffers_.point_clouds.push_back(std::move(measurement));
  last_timestamp_lidar = cur_head_time;

  lock.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
  if (!imu_en) return;

  if (last_timestamp_lidar < 0.0) return;
  // ROS_INFO("get imu at time: %.6f", msg_in->header.stamp.toSec());
  sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
  msg->header.stamp = ros::Time().fromSec(msg->header.stamp.toSec() - imu_time_offset);
  double timestamp = msg->header.stamp.toSec();

  if (fabs(last_timestamp_lidar - timestamp) > 0.5 && (!ros_driver_fix_en))
  {
    ROS_WARN("IMU and LiDAR not synced! delta time: %lf .\n", last_timestamp_lidar - timestamp);
  }

  if (ros_driver_fix_en) timestamp += std::round(last_timestamp_lidar - timestamp);
  msg->header.stamp = ros::Time().fromSec(timestamp);

  mtx_buffer.lock();

  if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu)
  {
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    ROS_ERROR("imu loop back, offset: %lf \n", last_timestamp_imu - timestamp);
    return;
  }

  // if (last_timestamp_imu > 0.0 && timestamp > last_timestamp_imu + 0.2)
  // {

  //   ROS_WARN("imu time stamp Jumps %0.4lf seconds \n", timestamp - last_timestamp_imu);
  //   mtx_buffer.unlock();
  //   sig_buffer.notify_all();
  //   return;
  // }

  last_timestamp_imu = timestamp;

  const auto measurement = livo_atlas::ros1::ConvertImu(*msg);
  sensor_buffers_.imu.push_back(measurement);
  sensor_buffers_.latest_imu_time = last_timestamp_imu;
  mtx_buffer.unlock();
  if (imu_prop_enable)
  {
    mtx_buffer_imu_prop.lock();
    if (imu_prop_enable && !frontend_->imu_needs_initialization()) { prop_imu_buffer.push_back(measurement); }
    newest_imu = measurement;
    newest_imu_stamp_ = msg->header.stamp;
    new_imu = true;
    mtx_buffer_imu_prop.unlock();
  }
  sig_buffer.notify_all();
}

void LIVMapper::img_cbk(const sensor_msgs::ImageConstPtr &msg_in)
{
  if (!img_en) return;
  const auto& msg = msg_in;
  // if ((abs(msg->header.stamp.toSec() - last_timestamp_img) > 0.2 && last_timestamp_img > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("img jumps %.3f\n", msg->header.stamp.toSec() - last_timestamp_img);
  //   sync_jump_flag = true;
  //   msg->header.stamp = ros::Time().fromSec(last_timestamp_img + 0.1);
  // }

  // Hiliti2022 40Hz
  if (hilti_en)
  {
    static int frame_counter = 0;
    if (++frame_counter % 4 != 0) return;
  }
  // double msg_header_time =  msg->header.stamp.toSec();
  double msg_header_time = msg->header.stamp.toSec() + img_time_offset;
  if (abs(msg_header_time - last_timestamp_img) < 0.001) return;
  ROS_INFO("Get image, its header time: %.6f", msg_header_time);
  if (last_timestamp_lidar < 0) return;

  if (msg_header_time < last_timestamp_img)
  {
    ROS_ERROR("image loop back. \n");
    return;
  }

  std::unique_lock<std::mutex> lock(mtx_buffer);

  double img_time_correct = msg_header_time; // last_timestamp_lidar + 0.105;

  if (img_time_correct - last_timestamp_img < 0.02)
  {
    ROS_WARN("Image need Jumps: %.6f", img_time_correct);
    lock.unlock();
    sig_buffer.notify_all();
    return;
  }

  livo_atlas::ImageMeasurement measurement;
  const std::string error =
      livo_atlas::ros1::ConvertImage(*msg, img_time_offset, &measurement);
  if (!error.empty()) {
    ROS_ERROR_STREAM(error);
    lock.unlock();
    return;
  }
  sensor_buffers_.images.push_back(std::move(measurement));

  // ROS_INFO("Correct Image time: %.6f", img_time_correct);

  last_timestamp_img = img_time_correct;
  // cv::imshow("img", img);
  // cv::waitKey(1);
  // cout<<"last_timestamp_img:::"<<last_timestamp_img<<endl;
  lock.unlock();
  sig_buffer.notify_all();
}

bool LIVMapper::sync_packages(LidarMeasureGroup& measurements) {
  livo_atlas::SynchronizationResult result;
  {
    std::lock_guard<std::mutex> lock(mtx_buffer);
    result = synchronizer_.TrySynchronize(&sensor_buffers_, &measurements);
  }
  if (result != livo_atlas::SynchronizationResult::kWaiting) {
    sig_buffer.notify_all();
  }
  if (result == livo_atlas::SynchronizationResult::kImageDropped) {
    ROS_ERROR("[ Data Cut ] Throw one image frame! \n");
  } else if (result == livo_atlas::SynchronizationResult::kInvalidCloud) {
    ROS_ERROR("Discarded an invalid point cloud during synchronization");
  }
  return result == livo_atlas::SynchronizationResult::kReady;
}
