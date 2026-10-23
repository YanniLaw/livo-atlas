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
#include <cstdint>
#include <iomanip>
#include <limits>
#include <ostream>
#include <stdexcept>

#include <pcl_conversions/pcl_conversions.h>

#include "livo_atlas/ros1/calibration_reader.h"
#include "livo_atlas/ros1/measurement_conversions.h"

LIVMapper::LIVMapper(ros::NodeHandle &nh)
    : extT(0, 0, 0),
      extR(M3D::Identity())
{
  p_pre.reset(new Preprocess());

  readParameters(nh);
  ValidateImageParameters();
  loadVoxelConfig(nh, frontend_options_.voxel_map);

  visual_sub_map.reset(new PointCloudXYZI());
  pcl_w_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_save.reset(new PointCloudXYZRGB());
  pcl_wait_save_intensity.reset(new PointCloudXYZI());
  root_dir = ROOT_DIR;
  initializeFiles();
  InitializeFrontend();
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "camera_init";
}

LIVMapper::~LIVMapper() {}

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
  nh.param<bool>("vio/inverse_composition_en", visual_options.inverse_composition, false);
  nh.param<int>("vio/max_iterations", visual_options.max_iterations, 5);
  nh.param<double>("vio/img_point_cov", visual_options.image_point_covariance, 100);
  nh.param<bool>("vio/raycast_en", visual_options.raycast, false);
  nh.param<bool>("vio/exposure_estimate_en", visual_options.estimate_exposure, true);
  nh.param<double>("vio/inv_expo_cov", imu_options.inverse_exposure_covariance, 0.2);
  nh.param<int>("vio/grid_size", visual_options.grid_size, 5);
  nh.param<int>("vio/grid_n_height", visual_options.grid_rows, 17);
  nh.param<int>("vio/patch_pyrimid_level", visual_options.patch_pyramid_levels, 3);
  nh.param<int>("vio/patch_size", visual_options.patch_size, 8);
  nh.param<double>("vio/outlier_threshold", visual_options.outlier_threshold, 1000);

  nh.param<double>("time_offset/exposure_time_init", exposure_time_init, 0.0);
  nh.param<double>("time_offset/img_time_offset", img_time_offset, 0.0);
  nh.param<double>("time_offset/imu_time_offset", imu_time_offset, 0.0);
  nh.param<double>("time_offset/lidar_time_offset", lidar_time_offset, 0.0);
  nh.param<bool>("uav/imu_rate_odom", imu_prop_enable, false);
  nh.param<bool>("uav/gravity_align_en", frontend_options_.align_gravity, false);

  nh.param<string>("evo/seq_name", seq_name, "01");
  nh.param<bool>("evo/pose_output_en", pose_output_en, false);
  nh.param<double>("imu/gyr_cov", imu_options.gyroscope_covariance, 1.0);
  nh.param<double>("imu/acc_cov", imu_options.accelerometer_covariance, 1.0);
  nh.param<int>("imu/imu_int_frame", imu_options.initialization_frames, 3);
  nh.param<bool>("imu/imu_en", imu_en, false);
  nh.param<bool>("imu/gravity_est_en", imu_options.estimate_gravity, true);
  nh.param<bool>("imu/ba_bg_est_en", imu_options.estimate_bias, true);

  nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
  nh.param<double>("preprocess/filter_size_surf", frontend_options_.surface_filter_size, 0.5);
  nh.param<bool>("preprocess/hilti_en", hilti_en, false);
  nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
  nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 6);
  nh.param<int>("preprocess/point_filter_num", p_pre->point_filter_num, 3);
  nh.param<bool>("preprocess/feature_extract_enabled", p_pre->feature_enabled, false);

  nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
  nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
  nh.param<int>("pcd_save/type", pcd_save_type, 0);
  nh.param<bool>("image_save/img_save_en", img_save_en, false);
  nh.param<int>("image_save/interval", img_save_interval, 1);

  nh.param<bool>("pcd_save/colmap_output_en", colmap_output_en, false);
  nh.param<double>("pcd_save/filter_size_pcd", filter_size_pcd, 0.5);
  nh.param<double>("debug/plot_time", visual_options.plot_time, -10);
  nh.param<int>("debug/frame_cnt", visual_options.plot_frame_count, 6);

  nh.param<double>("publish/blind_rgb_points", blind_rgb_points, 0.01);
  nh.param<int>("publish/pub_scan_num", pub_scan_num, 1);
  nh.param<bool>("publish/pub_effect_point_en", pub_effect_point_en, false);
  nh.param<bool>("publish/dense_map_en", frontend_options_.dense_cloud, false);

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
  if (colmap_output_en && camera.model != "Pinhole") {
    throw std::invalid_argument("pcd_save/colmap_output_en requires a Pinhole camera");
  }
}

void LIVMapper::InitializeFrontend() {
  extT = calibration_.imu_from_lidar.translation;
  extR = calibration_.imu_from_lidar.rotation;
  frontend_options_.calibration = calibration_;
  frontend_options_.imu.enabled = imu_en;
  frontend_options_.visual.colmap_output = colmap_output_en;
  frontend_ = std::make_unique<livo_atlas::Frontend>(frontend_options_);
  slam_mode_ = (img_en && lidar_en) ? LIVO : imu_en ? ONLY_LIO : ONLY_LO;
  synchronizer_ = livo_atlas::SensorSynchronizer(slam_mode_, exposure_time_init);
}

void LIVMapper::initializeFiles() 
{
  if (pcd_save_en && colmap_output_en)
  {
      const std::string folderPath = std::string(ROOT_DIR) + "/scripts/colmap_output.sh";
      
      std::string chmodCommand = "chmod +x " + folderPath;
      
      int chmodRet = system(chmodCommand.c_str());  
      if (chmodRet != 0) {
          std::cerr << "Failed to set execute permissions for the script." << std::endl;
          return;
      }

      int executionRet = system(folderPath.c_str());
      if (executionRet != 0) {
          std::cerr << "Failed to execute the script." << std::endl;
          return;
      }
  }
  if(colmap_output_en) fout_points.open(std::string(ROOT_DIR) + "Log/Colmap/sparse/0/points3D.txt", std::ios::out);
  if(pcd_save_en) fout_lidar_pos.open(std::string(ROOT_DIR) + "Log/pcd/lidar_poses.txt", std::ios::out);
  if(img_save_en) fout_visual_pos.open(std::string(ROOT_DIR) + "Log/image/image_poses.txt", std::ios::out);
  fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), std::ios::out);
  fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), std::ios::out);
}

void LIVMapper::initializeSubscribersAndPublishers(ros::NodeHandle &nh, image_transport::ImageTransport &it) 
{
  sub_pcl = p_pre->lidar_type == AVIA ? 
            nh.subscribe(lid_topic, 200000, &LIVMapper::livox_pcl_cbk, this): 
            nh.subscribe(lid_topic, 200000, &LIVMapper::standard_pcl_cbk, this);
  sub_imu = nh.subscribe(imu_topic, 200000, &LIVMapper::imu_cbk, this);
  sub_img = nh.subscribe(img_topic, 200000, &LIVMapper::img_cbk, this);
  
  pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);
  pubNormal = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 100);
  pubSubVisualMap = nh.advertise<sensor_msgs::PointCloud2>("/cloud_visual_sub_map_before", 100);
  pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100);
  pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100);
  pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
  pubPath = nh.advertise<nav_msgs::Path>("/path", 10);
  plane_pub = nh.advertise<visualization_msgs::Marker>("/planner_normal", 1);
  voxel_pub = nh.advertise<visualization_msgs::MarkerArray>("/voxels", 1);
  pubLaserCloudDyn = nh.advertise<sensor_msgs::PointCloud2>("/dyn_obj", 100);
  pubLaserCloudDynRmed = nh.advertise<sensor_msgs::PointCloud2>("/dyn_obj_removed", 100);
  pubLaserCloudDynDbg = nh.advertise<sensor_msgs::PointCloud2>("/dyn_obj_dbg_hist", 100);
  mavros_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
  pubImage = it.advertise("/rgb_img", 1);
  pubImuPropOdom = nh.advertise<nav_msgs::Odometry>("/LIVO2/imu_propagate", 10000);
  imu_prop_timer = nh.createTimer(ros::Duration(0.004), &LIVMapper::imu_prop_callback, this);
  frontend_->local_map().voxel_map_pub_= nh.advertise<visualization_msgs::MarkerArray>("/planes", 10000);
}

namespace {

void WriteStateRow(std::ostream& stream, double time,
                   const livo_atlas::State& state) {
  const V3D euler = RotMtoEuler(state.rot_end);
  stream << std::setw(20) << time << " " << euler.transpose() * 57.3 << " "
         << state.pos_end.transpose() << " " << state.vel_end.transpose() << " "
         << state.bias_g.transpose() << " " << state.bias_a.transpose() << " "
         << V3D(state.inv_expo_time, 0, 0).transpose();
}

}  // namespace

void LIVMapper::ProcessMeasurements() {
  const auto start = std::chrono::steady_clock::now();
  last_result_ = frontend_->Process(&LidarMeasures);
  const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();
  if (last_result_.first_frame) std::cout << "FIRST LIDAR FRAME!" << std::endl;
  WriteStateRow(fout_pre, last_result_.relative_time, last_result_.prediction);
  fout_pre << std::endl;
  if (!last_result_.updated) return;

  pcl_w_wait_pub = last_result_.world_cloud;
  feats_undistort = last_result_.undistorted_cloud;
  if (imu_prop_enable) {
    ekf_finish_once = true;
    latest_ekf_state = last_result_.estimate;
    latest_ekf_time = last_result_.timestamp;
    state_update_flg = true;
  }
  auto& visual = frontend_->visual();
  if (last_result_.kind == VIO) {
    publish_frame_world(pubLaserCloudFullRes, visual);
    publish_img_rgb(pubImage, visual);
  } else {
    if (pose_output_en) {
      static bool opened = false;
      const auto mode = opened ? std::ios::app : std::ios::out;
      std::ofstream trajectory(std::string(ROOT_DIR) + "Log/result/" +
                               seq_name + ".txt", mode);
      opened = true;
      if (!trajectory.is_open()) ROS_ERROR("open fail\n");
      const auto& state = last_result_.estimate;
      const Eigen::Quaterniond orientation(state.rot_end);
      trajectory << std::fixed << last_result_.timestamp << " "
                 << state.pos_end[0] << " " << state.pos_end[1] << " "
                 << state.pos_end[2] << " " << orientation.x() << " "
                 << orientation.y() << " " << orientation.z() << " "
                 << orientation.w() << std::endl;
    }
    const V3D euler = RotMtoEuler(last_result_.estimate.rot_end);
    geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler(0), euler(1), euler(2));
    publish_odometry(pubOdomAftMapped);
    publish_frame_world(pubLaserCloudFullRes, visual);
    auto& map = frontend_->local_map();
    if (pub_effect_point_en) {
      publish_effect_world(pubLaserCloudEffect, map.ptpl_list_);
    }
    if (map.config_setting_.is_pub_plane_map_) map.pubVoxelMap();
    publish_path(pubPath);
    publish_mavros(mavros_pose_publisher);
    // This measures the complete frontend call, including IMU processing.
    printf("LIO Mapping Time\n| Current Total Time | %.6f |\n", elapsed);
  }
  WriteStateRow(fout_out, last_result_.relative_time, last_result_.estimate);
  fout_out << " " << last_result_.undistorted_cloud->points.size() << std::endl;
}

void LIVMapper::savePCD() 
{
  if (pcd_save_en && (pcl_wait_save->points.size() > 0 || pcl_wait_save_intensity->points.size() > 0) && pcd_save_interval < 0) 
  {
    std::string raw_points_dir = std::string(ROOT_DIR) + "Log/pcd/all_raw_points.pcd";
    std::string downsampled_points_dir = std::string(ROOT_DIR) + "Log/pcd/all_downsampled_points.pcd";
    pcl::PCDWriter pcd_writer;

    if (img_en)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
      voxel_filter.setInputCloud(pcl_wait_save);
      voxel_filter.setLeafSize(filter_size_pcd, filter_size_pcd, filter_size_pcd);
      voxel_filter.filter(*downsampled_cloud);
  
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save); // Save the raw point cloud data
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save->points.size() << RESET << std::endl;
      
      pcd_writer.writeBinary(downsampled_points_dir, *downsampled_cloud); // Save the downsampled point cloud data
      std::cout << GREEN << "Downsampled point cloud data saved to: " << downsampled_points_dir 
                << " with point count after filtering: " << downsampled_cloud->points.size() << RESET << std::endl;

      if(colmap_output_en)
      {
        fout_points << "# 3D point list with one line of data per point\n";
        fout_points << "#  POINT_ID, X, Y, Z, R, G, B, ERROR\n";
        for (size_t i = 0; i < downsampled_cloud->size(); ++i) 
        {
            const auto& point = downsampled_cloud->points[i];
            fout_points << i << " "
                        << std::fixed << std::setprecision(6)
                        << point.x << " " << point.y << " " << point.z << " "
                        << static_cast<int>(point.r) << " "
                        << static_cast<int>(point.g) << " "
                        << static_cast<int>(point.b) << " "
                        << 0 << std::endl;
        }
      }
    }
    else
    {      
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save_intensity);
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save_intensity->points.size() << RESET << std::endl;
    }
  }
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
  savePCD();
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

    V3D posi, vel_i;
    Eigen::Quaterniond q;
    posi = imu_propagate.pos_end;
    vel_i = imu_propagate.vel_end;
    q = Eigen::Quaterniond(imu_propagate.rot_end);
    imu_prop_odom.header.frame_id = "world";
    imu_prop_odom.header.stamp = newest_imu_stamp_;
    imu_prop_odom.pose.pose.position.x = posi.x();
    imu_prop_odom.pose.pose.position.y = posi.y();
    imu_prop_odom.pose.pose.position.z = posi.z();
    imu_prop_odom.pose.pose.orientation.w = q.w();
    imu_prop_odom.pose.pose.orientation.x = q.x();
    imu_prop_odom.pose.pose.orientation.y = q.y();
    imu_prop_odom.pose.pose.orientation.z = q.z();
    imu_prop_odom.twist.twist.linear.x = vel_i.x();
    imu_prop_odom.twist.twist.linear.y = vel_i.y();
    imu_prop_odom.twist.twist.linear.z = vel_i.z();
    pubImuPropOdom.publish(imu_prop_odom);
  }
  mtx_buffer_imu_prop.unlock();
}

void LIVMapper::pointBodyToWorld(const PointType &pi, PointType &po)
{
  V3D p_body(pi.x, pi.y, pi.z);
  V3D p_global(frontend_->state().rot_end * (extR * p_body + extT) + frontend_->state().pos_end);
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

template <typename T> void LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(frontend_->state().rot_end * (extR * p_body + extT) + frontend_->state().pos_end);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

template <typename T> Matrix<T, 3, 1> LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi)
{
  V3D p(pi[0], pi[1], pi[2]);
  p = (frontend_->state().rot_end * (extR * p + extT) + frontend_->state().pos_end);
  Matrix<T, 3, 1> po(p[0], p[1], p[2]);
  return po;
}

void LIVMapper::RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(frontend_->state().rot_end * (extR * p_body + extT) + frontend_->state().pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void LIVMapper::RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu(extR * p_body_lidar + extT);

  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
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

void LIVMapper::publish_img_rgb(const image_transport::Publisher &pubImage, VIOManager& vio_manager)
{
  cv::Mat img_rgb = vio_manager.img_cp;
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = ros::Time::now();
  // out_msg.header.frame_id = "camera_init";
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = img_rgb;
  pubImage.publish(out_msg.toImageMsg());
}

// Provide output format for LiDAR-visual BA
void LIVMapper::publish_frame_world(const ros::Publisher &pubLaserCloudFullRes, VIOManager& vio_manager)
{
  if (pcl_w_wait_pub->empty()) return;
  PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());
  static int pub_num = 1;
  pub_num++;

  if (LidarMeasures.lio_vio_flg == VIO)
  {
    *pcl_wait_pub += *pcl_w_wait_pub;
    if(pub_num >= pub_scan_num)
    {
      pub_num = 1;
      size_t size = pcl_wait_pub->points.size();
      laserCloudWorldRGB->reserve(size);
      // double inv_expo = frontend_->state().inv_expo_time;
      cv::Mat img_rgb = vio_manager.img_rgb;
      for (size_t i = 0; i < size; i++)
      {
        PointTypeRGB pointRGB;
        pointRGB.x = pcl_wait_pub->points[i].x;
        pointRGB.y = pcl_wait_pub->points[i].y;
        pointRGB.z = pcl_wait_pub->points[i].z;

        V3D p_w(pcl_wait_pub->points[i].x, pcl_wait_pub->points[i].y, pcl_wait_pub->points[i].z);
        V3D pf(vio_manager.new_frame_->w2f(p_w)); if (pf[2] < 0) continue;
        V2D pc(vio_manager.new_frame_->w2c(p_w));

        if (vio_manager.new_frame_->cam_->isInFrame(pc.cast<int>(), 3)) // 100
        {
          V3F pixel = vio_manager.getInterpolatedPixel(img_rgb, pc);
          pointRGB.r = pixel[2];
          pointRGB.g = pixel[1];
          pointRGB.b = pixel[0];
          // pointRGB.r = pixel[2] * inv_expo; pointRGB.g = pixel[1] * inv_expo; pointRGB.b = pixel[0] * inv_expo;
          // if (pointRGB.r > 255) pointRGB.r = 255; else if (pointRGB.r < 0) pointRGB.r = 0;
          // if (pointRGB.g > 255) pointRGB.g = 255; else if (pointRGB.g < 0) pointRGB.g = 0;
          // if (pointRGB.b > 255) pointRGB.b = 255; else if (pointRGB.b < 0) pointRGB.b = 0;
          if (pf.norm() > blind_rgb_points) laserCloudWorldRGB->push_back(pointRGB);
        }
      }
    }
  }

  /*** Publish Frame ***/
  sensor_msgs::PointCloud2 laserCloudmsg;
  if (slam_mode_ == LIVO && LidarMeasures.lio_vio_flg == VIO)
  {
    pcl::toROSMsg(*laserCloudWorldRGB, laserCloudmsg);
  }
  if (slam_mode_ == ONLY_LIO || slam_mode_ == ONLY_LO)
  { 
    pcl::toROSMsg(*pcl_w_wait_pub, laserCloudmsg); 
  }
  laserCloudmsg.header.stamp = ros::Time::now(); //.fromSec(last_timestamp_lidar);
  laserCloudmsg.header.frame_id = "camera_init";
  pubLaserCloudFullRes.publish(laserCloudmsg);

  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  double update_time = 0.0;
  if (LidarMeasures.lio_vio_flg == VIO) {
    update_time = LidarMeasures.measures.back().vio_time;
  } else { // LIO / LO
    update_time = LidarMeasures.measures.back().lio_time;
  }
  std::stringstream ss_time;
  ss_time << std::fixed << std::setprecision(6) << update_time;

  if (pcd_save_en)
  {
    static int scan_wait_num = 0;

    switch (pcd_save_type)
    {
      case 0: /** world frame **/
        if (slam_mode_ == LIVO)
        {
          *pcl_wait_save += *laserCloudWorldRGB;
        }
        else
        {
          *pcl_wait_save_intensity += *pcl_w_wait_pub;
        }
        if(LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO) scan_wait_num++;
        break;

      case 1: /** body frame **/
        if (LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
        {
          int size = feats_undistort->points.size();
          PointCloudXYZI::Ptr laserCloudBody(new PointCloudXYZI(size, 1));
          for (int i = 0; i < size; i++)
          {
            RGBpointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudBody->points[i]);
          }
          *pcl_wait_save_intensity += *laserCloudBody;
          scan_wait_num++;
          cout << "save body frame points: " << pcl_wait_save_intensity->points.size() << endl;
        }
        pcd_save_interval = 1;
        
        break;

      default:
        pcd_save_interval = 1;
        scan_wait_num++;
        break;
    }
    if ((pcl_wait_save->size() > 0 || pcl_wait_save_intensity->size() > 0) && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
    {
      string all_points_dir(string(string(ROOT_DIR) + "Log/pcd/") + ss_time.str() + string(".pcd"));

      pcl::PCDWriter pcd_writer;

      cout << "current scan saved to " << all_points_dir << endl;
      if (pcl_wait_save->points.size() > 0)
      {
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save); // pcl::io::savePCDFileASCII(all_points_dir, *pcl_wait_save);
        PointCloudXYZRGB().swap(*pcl_wait_save);
      }
      if(pcl_wait_save_intensity->points.size() > 0)
      {
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_intensity);
        PointCloudXYZI().swap(*pcl_wait_save_intensity);
      }
      scan_wait_num = 0;
    }
    
    if(LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
    {
      Eigen::Quaterniond q(frontend_->state().rot_end);
      fout_lidar_pos << std::fixed << std::setprecision(6);
      fout_lidar_pos <<  LidarMeasures.measures.back().lio_time << " " << frontend_->state().pos_end[0] << " " << frontend_->state().pos_end[1] << " " << frontend_->state().pos_end[2] << " " << q.x() << " " << q.y() << " " << q.z()
          << " " << q.w() << " " << endl;
    }
  }
  if (img_save_en && LidarMeasures.lio_vio_flg == VIO)
  {
    static int img_wait_num = 0;
    img_wait_num++;

    if (img_save_interval > 0 && img_wait_num >= img_save_interval)
    {
      imwrite(string(string(ROOT_DIR) + "Log/image/") + ss_time.str() + string(".png"), vio_manager.img_rgb);
      
      Eigen::Quaterniond q(frontend_->state().rot_end);
      fout_visual_pos << std::fixed << std::setprecision(6);
      fout_visual_pos << LidarMeasures.measures.back().vio_time << " " << frontend_->state().pos_end[0] << " " << frontend_->state().pos_end[1] << " " << frontend_->state().pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
      img_wait_num = 0;
    }
  }

  if(laserCloudWorldRGB->size() > 0)  PointCloudXYZI().swap(*pcl_wait_pub); 
}

void LIVMapper::publish_visual_sub_map(const ros::Publisher &pubSubVisualMap)
{
  PointCloudXYZI::Ptr laserCloudFullRes(visual_sub_map);
  int size = laserCloudFullRes->points.size(); if (size == 0) return;
  PointCloudXYZI::Ptr sub_pcl_visual_map_pub(new PointCloudXYZI());
  *sub_pcl_visual_map_pub = *laserCloudFullRes;
  if (1)
  {
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*sub_pcl_visual_map_pub, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time::now();
    laserCloudmsg.header.frame_id = "camera_init";
    pubSubVisualMap.publish(laserCloudmsg);
  }
}

void LIVMapper::publish_effect_world(const ros::Publisher &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list)
{
  int effect_feat_num = ptpl_list.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effect_feat_num, 1));
  for (int i = 0; i < effect_feat_num; i++)
  {
    laserCloudWorld->points[i].x = ptpl_list[i].point_w_[0];
    laserCloudWorld->points[i].y = ptpl_list[i].point_w_[1];
    laserCloudWorld->points[i].z = ptpl_list[i].point_w_[2];
  }
  sensor_msgs::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = ros::Time::now();
  laserCloudFullRes3.header.frame_id = "camera_init";
  pubLaserCloudEffect.publish(laserCloudFullRes3);
}

template <typename T> void LIVMapper::set_posestamp(T &out)
{
  out.position.x = frontend_->state().pos_end(0);
  out.position.y = frontend_->state().pos_end(1);
  out.position.z = frontend_->state().pos_end(2);
  out.orientation.x = geoQuat.x;
  out.orientation.y = geoQuat.y;
  out.orientation.z = geoQuat.z;
  out.orientation.w = geoQuat.w;
}

void LIVMapper::publish_odometry(const ros::Publisher &pubOdomAftMapped)
{
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "aft_mapped";
  odomAftMapped.header.stamp = ros::Time::now(); //.ros::Time()fromSec(last_timestamp_lidar);
  set_posestamp(odomAftMapped.pose.pose);

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  transform.setOrigin(tf::Vector3(frontend_->state().pos_end(0), frontend_->state().pos_end(1), frontend_->state().pos_end(2)));
  q.setW(geoQuat.w);
  q.setX(geoQuat.x);
  q.setY(geoQuat.y);
  q.setZ(geoQuat.z);
  transform.setRotation(q);
  br.sendTransform( tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "aft_mapped") );
  pubOdomAftMapped.publish(odomAftMapped);
}

void LIVMapper::publish_mavros(const ros::Publisher &mavros_pose_publisher)
{
  msg_body_pose.header.stamp = ros::Time::now();
  msg_body_pose.header.frame_id = "camera_init";
  set_posestamp(msg_body_pose.pose);
  mavros_pose_publisher.publish(msg_body_pose);
}

void LIVMapper::publish_path(const ros::Publisher pubPath)
{
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = ros::Time::now();
  msg_body_pose.header.frame_id = "camera_init";
  path.poses.push_back(msg_body_pose);
  pubPath.publish(path);
}
