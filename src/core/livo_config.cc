#include "livo_atlas/core/livo_config.h"

#include <cstdlib>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <utility>

namespace livo_atlas {
namespace {
std::string DefaultDirectory() {
  const char* base = std::getenv("ROS_HOME");
  if (base != nullptr && *base != '\0')
    return (std::filesystem::path(base) / "livo_atlas").string();
  base = std::getenv("HOME");
  if (base != nullptr && *base != '\0')
    return (std::filesystem::path(base) / ".ros/livo_atlas").string();
  return (std::filesystem::current_path() / "livo_atlas_output").string();
}
RigidTransform Transform(ParameterReader& reader, const std::string& rotation,
                         const std::string& translation) {
  RigidTransform result;
  const auto rotation_values = reader.NumberArray(rotation);
  const auto translation_values = reader.NumberArray(translation);
  const auto error =
      ParseExtrinsic(rotation_values, translation_values, &result);
  if (!error.empty())
    throw std::invalid_argument(reader.Name(rotation) + " / " +
                                reader.Name(translation) + ": " + error);
  return result;
}
}  // namespace
LivoConfig LoadLivoConfig(ParameterReader& reader) {
  LivoConfig config;
  auto& options = config.system;
  auto& front = options.frontend;
  auto& visual = front.visual;
  auto& imu = front.imu;
  auto& pre = options.preprocess;
  auto& output = options.exporter;
  auto& processor = options.processor;
  config.lidar_topic = reader.String("common.lid_topic", config.lidar_topic);
  config.imu_topic = reader.String("common.imu_topic", config.imu_topic);
  options.driver_fix =
      reader.Boolean("common.ros_driver_bug_fix", options.driver_fix);
  options.image_enabled =
      reader.Integer("common.img_en", options.image_enabled ? 1 : 0) != 0;
  options.lidar_enabled =
      reader.Integer("common.lidar_en", options.lidar_enabled ? 1 : 0) != 0;
  config.image_topic = reader.String("common.img_topic", config.image_topic);
  visual.normal_enabled =
      reader.Boolean("vio.normal_en", visual.normal_enabled);
  visual.inverse_composition =
      reader.Boolean("vio.inverse_composition_en", visual.inverse_composition);
  visual.max_iterations =
      reader.Integer("vio.max_iterations", visual.max_iterations);
  visual.image_point_covariance =
      reader.Number("vio.img_point_cov", visual.image_point_covariance);
  visual.raycast = reader.Boolean("vio.raycast_en", visual.raycast);
  visual.estimate_exposure =
      reader.Boolean("vio.exposure_estimate_en", visual.estimate_exposure);
  imu.inverse_exposure_covariance =
      reader.Number("vio.inv_expo_cov", imu.inverse_exposure_covariance);
  visual.grid_size = reader.Integer("vio.grid_size", visual.grid_size);
  visual.grid_rows = reader.Integer("vio.grid_n_height", visual.grid_rows);
  visual.patch_pyramid_levels =
      reader.Integer("vio.patch_pyrimid_level", visual.patch_pyramid_levels);
  visual.patch_size = reader.Integer("vio.patch_size", visual.patch_size);
  visual.outlier_threshold =
      reader.Number("vio.outlier_threshold", visual.outlier_threshold);
  options.exposure_time =
      reader.Number("time_offset.exposure_time_init", options.exposure_time);
  config.image_time_offset =
      reader.Number("time_offset.img_time_offset", config.image_time_offset);
  options.imu_time_offset =
      reader.Number("time_offset.imu_time_offset", options.imu_time_offset);
  config.lidar_time_offset =
      reader.Number("time_offset.lidar_time_offset", config.lidar_time_offset);
  options.propagate_imu =
      reader.Boolean("uav.imu_rate_odom", options.propagate_imu);
  front.align_gravity =
      reader.Boolean("uav.gravity_align_en", front.align_gravity);
  output.sequence_name = reader.String("evo.seq_name", output.sequence_name);
  output.trajectory = reader.Boolean("evo.pose_output_en", output.trajectory);
  imu.gyroscope_covariance =
      reader.Number("imu.gyr_cov", imu.gyroscope_covariance);
  imu.accelerometer_covariance =
      reader.Number("imu.acc_cov", imu.accelerometer_covariance);
  imu.initialization_frames =
      reader.Integer("imu.imu_int_frame", imu.initialization_frames);
  imu.enabled = reader.Boolean("imu.imu_en", imu.enabled);
  imu.estimate_gravity =
      reader.Boolean("imu.gravity_est_en", imu.estimate_gravity);
  imu.estimate_bias = reader.Boolean("imu.ba_bg_est_en", imu.estimate_bias);
  pre.blind = reader.Number("preprocess.blind", pre.blind);
  front.surface_filter_size =
      reader.Number("preprocess.filter_size_surf", front.surface_filter_size);
  options.hilti = reader.Boolean("preprocess.hilti_en", options.hilti);
  pre.lidar_type = reader.Integer("preprocess.lidar_type", pre.lidar_type);
  pre.scan_lines = reader.Integer("preprocess.scan_line", pre.scan_lines);
  pre.point_filter =
      reader.Integer("preprocess.point_filter_num", pre.point_filter);
  pre.features =
      reader.Boolean("preprocess.feature_extract_enabled", pre.features);
  output.pcd_interval =
      reader.Integer("pcd_save.interval", output.pcd_interval);
  output.pcd = reader.Boolean("pcd_save.pcd_save_en", output.pcd);
  output.pcd_type = reader.Integer("pcd_save.type", output.pcd_type);
  output.images = reader.Boolean("image_save.img_save_en", output.images);
  output.image_interval =
      reader.Integer("image_save.interval", output.image_interval);
  output.colmap = reader.Boolean("pcd_save.colmap_output_en", output.colmap);
  output.voxel_size =
      reader.Number("pcd_save.filter_size_pcd", output.voxel_size);
  visual.plot_time = reader.Number("debug.plot_time", visual.plot_time);
  visual.plot_frame_count =
      reader.Integer("debug.frame_cnt", visual.plot_frame_count);
  processor.minimum_color_distance = reader.Number(
      "publish.blind_rgb_points", processor.minimum_color_distance);
  processor.scan_interval =
      reader.Integer("publish.pub_scan_num", processor.scan_interval);
  front.output_effective_points = reader.Boolean("publish.pub_effect_point_en",
                                                 front.output_effective_points);
  front.dense_cloud = reader.Boolean("publish.dense_map_en", front.dense_cloud);
  output.directory = reader.String("output.directory", DefaultDirectory());
  auto& calibration = front.calibration;
  calibration.imu_from_lidar =
      Transform(reader, "extrin_calib.extrinsic_R", "extrin_calib.extrinsic_T");
  calibration.camera_from_lidar =
      Transform(reader, "extrin_calib.Rcl", "extrin_calib.Pcl");
  auto& camera = calibration.camera;
  camera.model = reader.String("cam_model", camera.model);
  camera.width = reader.Integer("cam_width", camera.width);
  camera.height = reader.Integer("cam_height", camera.height);
  camera.fx = reader.Number("cam_fx", std::numeric_limits<double>::quiet_NaN());
  camera.fy = reader.Number("cam_fy", std::numeric_limits<double>::quiet_NaN());
  camera.cx = reader.Number("cam_cx", std::numeric_limits<double>::quiet_NaN());
  camera.cy = reader.Number("cam_cy", std::numeric_limits<double>::quiet_NaN());
  camera.scale = reader.Number("scale", camera.scale);
  for (int i = 0; i < 4; ++i) {
    const std::string name = camera.model == "Pinhole"
                                 ? "cam_d" + std::to_string(i)
                                 : "k" + std::to_string(i + 1);
    try {
      camera.distortion[i] = reader.Number(name, camera.distortion[i]);
    } catch (const std::invalid_argument& error) {
      throw std::invalid_argument(std::string("camera distortion: ") +
                                  error.what());
    }
  }
  auto& map = front.voxel_map;
  map.is_pub_plane_map_ =
      reader.Boolean("publish.pub_plane_en", map.is_pub_plane_map_);
  map.max_layer_ = reader.Integer("lio.max_layer", map.max_layer_);
  map.max_voxel_size_ = reader.Number("lio.voxel_size", map.max_voxel_size_);
  map.planner_threshold_ =
      reader.Number("lio.min_eigen_value", map.planner_threshold_);
  map.sigma_num_ = reader.Number("lio.sigma_num", map.sigma_num_);
  map.beam_err_ = reader.Number("lio.beam_err", map.beam_err_);
  map.dept_err_ = reader.Number("lio.dept_err", map.dept_err_);
  map.layer_init_num_ =
      reader.IntegerArray("lio.layer_init_num", map.layer_init_num_);
  map.max_points_num_ =
      reader.Integer("lio.max_points_num", map.max_points_num_);
  map.max_iterations_ =
      reader.Integer("lio.max_iterations", map.max_iterations_);
  map.map_sliding_en =
      reader.Boolean("local_map.map_sliding_en", map.map_sliding_en);
  map.half_map_size =
      reader.Integer("local_map.half_map_size", map.half_map_size);
  map.sliding_thresh =
      reader.Number("local_map.sliding_thresh", map.sliding_thresh);
  if (config.lidar_topic.empty())
    throw std::invalid_argument(reader.Name("common.lid_topic") +
                                " must not be empty");
  if (config.imu_topic.empty())
    throw std::invalid_argument(reader.Name("common.imu_topic") +
                                " must not be empty");
  if (config.image_topic.empty())
    throw std::invalid_argument(reader.Name("common.img_topic") +
                                " must not be empty");
  config.system = PrepareSystemOptions(std::move(config.system));
  return config;
}
}  // namespace livo_atlas
