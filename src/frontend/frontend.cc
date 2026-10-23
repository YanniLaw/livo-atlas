/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "livo_atlas/frontend/frontend.h"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/filters/voxel_grid.h>

#include "IMU_Processing.h"
#include "livo_atlas/camera/camera_factory.h"
#include "vio.h"

namespace livo_atlas {

class Frontend::Impl {
 public:
  explicit Impl(const FrontendOptions& options)
      : options_(options),
        imu_(new ImuProcess),
        camera_(CreateCamera(options.calibration.camera)),
        visual_(new VIOManager) {
    if (!camera_) throw std::invalid_argument("Invalid camera calibration");
    std::unordered_map<VOXEL_LOCATION, VoxelOctoTree*> empty_map;
    map_.reset(new VoxelMapManager(options_.voxel_map, empty_map));
    const auto& extrinsics = options_.calibration.imu_from_lidar;
    map_->extT_ = extrinsics.translation;
    map_->extR_ = extrinsics.rotation;
    filter_.setLeafSize(options_.surface_filter_size,
                        options_.surface_filter_size,
                        options_.surface_filter_size);

    const auto& visual = options_.visual;
    visual_->cam = camera_.get();
    visual_->grid_size = visual.grid_size;
    visual_->patch_size = visual.patch_size;
    visual_->outlier_threshold = visual.outlier_threshold;
    visual_->setImuToLidarExtrinsic(
        extrinsics.translation, extrinsics.rotation);
    visual_->SetLidarToCameraExtrinsic(
        options_.calibration.camera_from_lidar);
    visual_->state = &state_;
    visual_->state_propagat = &propagated_;
    visual_->max_iterations = visual.max_iterations;
    visual_->img_point_cov = visual.image_point_covariance;
    visual_->normal_en = visual.normal_enabled;
    visual_->inverse_composition_en = visual.inverse_composition;
    visual_->raycast_en = visual.raycast;
    visual_->grid_n_height = visual.grid_rows;
    visual_->patch_pyrimid_level = visual.patch_pyramid_levels;
    visual_->exposure_estimate_en = visual.estimate_exposure;
    visual_->initializeVIO();

    const auto& imu = options_.imu;
    imu_->set_extrinsic(extrinsics.translation, extrinsics.rotation);
    imu_->set_gyr_cov_scale(
        Eigen::Vector3d::Constant(imu.gyroscope_covariance));
    imu_->set_acc_cov_scale(
        Eigen::Vector3d::Constant(imu.accelerometer_covariance));
    imu_->set_inv_expo_cov(imu.inverse_exposure_covariance);
    imu_->set_gyr_bias_cov(Eigen::Vector3d::Constant(0.0001));
    imu_->set_acc_bias_cov(Eigen::Vector3d::Constant(0.0001));
    imu_->set_imu_init_frame_num(imu.initialization_frames);
    if (!imu.enabled) imu_->disable_imu();
    if (!imu.estimate_gravity) imu_->disable_gravity_est();
    if (!imu.estimate_bias) imu_->disable_bias_est();
    if (!visual.estimate_exposure) imu_->disable_exposure_est();
  }

  ~Impl() {
    // Visual objects can borrow map data; destroy them before the voxel roots.
    visual_.reset();
    for (const auto& entry : map_->voxel_map_) delete entry.second;
    map_->voxel_map_.clear();
  }

  FrontendResult Process(SynchronizedMeasurements* measurements) {
    assert(measurements != nullptr);
    FrontendResult result;
    result.kind = measurements->lio_vio_flg;
    if (measurements->measures.empty() || result.kind == UpdateKind::kWaiting) {
      return result;
    }
    if (!started_) {
      first_lidar_time_ = measurements->last_lio_update_time;
      imu_->first_lidar_time = first_lidar_time_;
      started_ = true;
      result.first_frame = true;
    }
    imu_->Process2(*measurements, state_, undistorted_);
    result.imu_initialized = options_.imu.enabled && !imu_->imu_need_init;
    result.imu_diagnostics = imu_->diagnostics_;
    if (options_.align_gravity) AlignGravity();
    propagated_ = state_;
    map_->state_ = state_;
    map_->feats_undistort_ = undistorted_;
    result.timestamp = measurements->last_lio_update_time;
    result.relative_time = result.timestamp - first_lidar_time_;
    result.prediction = state_;
    result.undistorted_cloud = undistorted_;

    switch (result.kind) {
      case UpdateKind::kLidarInertial:
      case UpdateKind::kLidarOnly:
        result.updated = ProcessLidar();
        break;
      case UpdateKind::kVisual:
        result.updated = ProcessVisual(*measurements, result.relative_time);
        break;
      default:
        break;
    }
    result.estimate = state_;
    if (result.updated) {
      result.world_cloud = world_;
      CaptureOutput(&result);
      // Consumption belongs to the frontend, even if the caller skips output.
      if (result.kind == UpdateKind::kVisual) world_.reset(new PointCloud);
    }
    return result;
  }

  void CaptureOutput(FrontendResult* result) {
    if (result->kind == UpdateKind::kVisual) {
      result->color_image = visual_->img_rgb;
      result->tracking_image = visual_->img_cp;
      const auto& pose = visual_->new_frame_->T_f_w_;
      result->camera_from_world_rotation = pose.unit_quaternion();
      result->camera_from_world_translation = pose.translation();
      result->debug_images = visual_->debug_images_;
      return;
    }
    if (options_.output_effective_points) {
      PointCloud::Ptr effective(new PointCloud(map_->ptpl_list_.size(), 1));
      for (std::size_t i = 0; i < map_->ptpl_list_.size(); ++i) {
        const auto& point = map_->ptpl_list_[i].point_w_;
        effective->points[i].x = point.x();
        effective->points[i].y = point.y();
        effective->points[i].z = point.z();
      }
      result->effective_cloud = effective;
    }
    if (options_.voxel_map.is_pub_plane_map_) {
      result->has_plane_visualization = true;
      result->planes = map_->GetVisualization();
    }
  }

  void AlignGravity() {
    if (imu_->imu_need_init || gravity_aligned_) return;
    const Eigen::Vector3d vertical(0, 0, -1);
    const Eigen::Quaterniond rotation =
        Eigen::Quaterniond::FromTwoVectors(state_.gravity, vertical);
    const Eigen::Matrix3d world_from_initial = rotation.toRotationMatrix();
    state_.pos_end = world_from_initial * state_.pos_end;
    state_.rot_end = world_from_initial * state_.rot_end;
    state_.vel_end = world_from_initial * state_.vel_end;
    state_.gravity = world_from_initial * state_.gravity;
    gravity_aligned_ = true;
  }

  void TransformCloud(const PointCloud::Ptr& input, PointCloud* output) {
    PointCloud().swap(*output);
    output->reserve(input->size());
    const auto& extrinsics = options_.calibration.imu_from_lidar;
    for (const auto& point : input->points) {
      Eigen::Vector3d position(point.x, point.y, point.z);
      position = (state_.rot_end * (extrinsics.rotation * position +
                                   extrinsics.translation) + state_.pos_end);
      Point transformed;
      transformed.x = position(0);
      transformed.y = position(1);
      transformed.z = position(2);
      transformed.intensity = point.intensity;
      output->points.push_back(transformed);
    }
  }

  bool ProcessLidar() {
    if (undistorted_->empty()) return false;
    filter_.setInputCloud(undistorted_);
    filter_.filter(*down_body_);
    map_->feats_down_body_ = down_body_;
    TransformCloud(down_body_, down_world_.get());
    map_->feats_down_world_ = down_world_;
    map_->feats_down_size_ = down_body_->points.size();
    if (!map_initialized_) {
      map_initialized_ = true;
      map_->BuildVoxelMap();
    }
    map_->StateEstimation(propagated_);
    state_ = map_->state_;
    points_ = map_->pv_list_;

    PointCloud::Ptr world_lidar(new PointCloud);
    TransformCloud(down_body_, world_lidar.get());
    const auto& extrinsics = options_.calibration.imu_from_lidar;
    for (std::size_t i = 0; i < world_lidar->points.size(); ++i) {
      const auto& point = world_lidar->points[i];
      map_->pv_list_[i].point_w << point.x, point.y, point.z;
      Eigen::Matrix3d cross = map_->cross_mat_list_[i];
      Eigen::Matrix3d variance = map_->body_cov_list_[i];
      variance = (state_.rot_end * extrinsics.rotation) * variance *
                     (state_.rot_end * extrinsics.rotation).transpose() +
                 (-cross) * state_.cov.block<3, 3>(0, 0) * (-cross).transpose() +
                 state_.cov.block<3, 3>(3, 3);
      map_->pv_list_[i].var = variance;
    }
    map_->UpdateVoxelMap(map_->pv_list_);
    points_ = map_->pv_list_;
    if (map_->config_setting_.map_sliding_en) map_->mapSliding();

    const PointCloud::Ptr input =
        options_.dense_cloud ? undistorted_ : down_body_;
    PointCloud::Ptr world(new PointCloud(input->size(), 1));
    for (std::size_t i = 0; i < input->size(); ++i) {
      const auto& point = input->points[i];
      const Eigen::Vector3d body(point.x, point.y, point.z);
      const Eigen::Vector3d global(
          state_.rot_end * (extrinsics.rotation * body +
                            extrinsics.translation) + state_.pos_end);
      auto& transformed = world->points[i];
      transformed.x = global(0);
      transformed.y = global(1);
      transformed.z = global(2);
      transformed.intensity = point.intensity;
    }
    world_ = world;
    return true;
  }

  bool ProcessVisual(SynchronizedMeasurements& measurements,
                     double relative_time) {
    if (world_->empty()) return false;
    visual_->plot_flag =
        std::fabs(relative_time - options_.visual.plot_time) <
        (options_.visual.plot_frame_count / 2 * 0.1);
    visual_->processFrame(measurements.measures.back().img, points_,
                          map_->voxel_map_, relative_time);
    return true;
  }

  FrontendOptions options_;
  State state_;
  State propagated_;
  std::unique_ptr<ImuProcess> imu_;
  std::unique_ptr<VoxelMapManager> map_;
  // Camera outlives VIO's non-owning pointer.
  std::unique_ptr<livo_atlas::Camera> camera_;
  std::unique_ptr<VIOManager> visual_;
  PointCloud::Ptr undistorted_{new PointCloud};
  PointCloud::Ptr down_body_{new PointCloud};
  PointCloud::Ptr down_world_{new PointCloud};
  PointCloud::Ptr world_{new PointCloud};
  std::vector<pointWithVar> points_;
  pcl::VoxelGrid<Point> filter_;
  bool started_ = false;
  bool map_initialized_ = false;
  bool gravity_aligned_ = false;
  double first_lidar_time_ = 0.0;
};

Frontend::Frontend(const FrontendOptions& options)
    : impl_(std::make_unique<Impl>(options)) {}
Frontend::~Frontend() = default;

FrontendResult Frontend::Process(SynchronizedMeasurements* measurements) {
  return impl_->Process(measurements);
}

const State& Frontend::state() const { return impl_->state_; }
bool Frontend::imu_needs_initialization() const {
  return impl_->imu_->imu_need_init;
}
double Frontend::imu_mean_acceleration_norm() const {
  if (!impl_->options_.imu.enabled || impl_->imu_->imu_need_init) return 0;
  return impl_->imu_->IMU_mean_acc_norm;
}
}  // namespace livo_atlas
