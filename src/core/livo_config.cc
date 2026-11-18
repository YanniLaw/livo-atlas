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
  config.loop_closure_enabled = reader.Boolean("loop_closure.enabled", false);
  config.loop_closure_options_file =
      reader.String("loop_closure.options_file", "");
  auto& gaussian = config.gaussian;
  gaussian.enabled = reader.Boolean("gaussian.enabled", false);
  gaussian.persist_submaps = reader.Boolean("gaussian.persist_submaps", false);
  gaussian.finalize_on_shutdown =
      reader.Boolean("gaussian.finalize_on_shutdown", false);
  gaussian.joint_render_enabled =
      reader.Boolean("gaussian.joint_render_enabled", false);
  auto& gaussian_output = config.gaussian_output;
  gaussian_output.enabled = reader.Boolean("gaussian.publish_preview", false);
  gaussian_output.optical_frame = reader.String(
      "gaussian.preview_frame_id", gaussian_output.optical_frame);
  if (gaussian_output.enabled &&
      (!gaussian.enabled || !gaussian.persist_submaps ||
       !gaussian.joint_render_enabled))
    throw std::invalid_argument(
        "Gaussian preview publication requires persistent joint rendering");
  const auto render_policy =
      reader.String("gaussian.joint_render_policy", "temporal");
  if (render_policy == "spatial") {
    gaussian.joint_render_policy = GaussianRenderPolicy::kSpatial;
    if (!gaussian.enabled || !gaussian.persist_submaps ||
        !gaussian.joint_render_enabled)
      throw std::invalid_argument(
          "Spatial rendering requires Gaussian persistence and joint "
          "rendering");
  } else if (render_policy != "temporal") {
    throw std::invalid_argument("Unknown Gaussian joint render policy");
  }
  if (gaussian.joint_render_enabled &&
      (!gaussian.enabled || !gaussian.persist_submaps))
    throw std::invalid_argument(
        "Joint rendering requires Gaussian persistence");
  if (gaussian.finalize_on_shutdown &&
      (!gaussian.enabled || !gaussian.persist_submaps))
    throw std::invalid_argument("Gaussian finalization requires persistence");
  if (gaussian.persist_submaps && !gaussian.enabled)
    throw std::invalid_argument(
        "Gaussian persistence requires gaussian.enabled");
  if (gaussian.enabled) {
    gaussian.socket_path = reader.String("gaussian.socket_path", "");
    gaussian.output_directory = reader.String("gaussian.output_directory", "");
    gaussian.shutdown_timeout = std::chrono::milliseconds(
        reader.Integer("gaussian.shutdown_timeout_ms", 3000));
    gaussian.finalize_timeout = std::chrono::milliseconds(
        reader.Integer("gaussian.finalize_timeout_ms", 60000));
    if (gaussian.joint_render_enabled) {
      gaussian.joint_render_interval = std::chrono::milliseconds(
          reader.Integer("gaussian.joint_render_interval_ms", 1000));
      gaussian.joint_render_timeout = std::chrono::milliseconds(
          reader.Integer("gaussian.joint_render_timeout_ms", 5000));
    }
    ValidateOnlineGaussianOptions(gaussian);
  }
  auto& global = config.global_output;
  global.enabled = reader.Boolean("global_output.enabled", global.enabled);
  global.publish_tf =
      reader.Boolean("global_output.publish_tf", global.publish_tf);
  global.map_frame = reader.String("global_output.map_frame", global.map_frame);
  global.trajectory.enabled = reader.Boolean("global_output.trajectory.enabled",
                                             global.trajectory.enabled);
  global.trajectory.max_frequency_hz =
      reader.Number("global_output.trajectory.max_frequency_hz",
                    global.trajectory.max_frequency_hz);
  global.trajectory.max_keyframes =
      reader.Integer("global_output.trajectory.max_keyframes",
                     global.trajectory.max_keyframes);
  auto& geometry_output = global.geometry;
  geometry_output.enabled =
      reader.Boolean("global_output.geometry.enabled", geometry_output.enabled);
  geometry_output.max_frequency_hz =
      reader.Number("global_output.geometry.max_frequency_hz",
                    geometry_output.max_frequency_hz);
  geometry_output.max_keyframes = reader.Integer(
      "global_output.geometry.max_keyframes", geometry_output.max_keyframes);
  geometry_output.map.max_points = reader.Integer(
      "global_output.geometry.max_points", geometry_output.map.max_points);
  geometry_output.map.max_cached_points =
      reader.Integer("global_output.geometry.max_cached_points",
                     geometry_output.map.max_cached_points);
  geometry_output.map.max_message_bytes =
      reader.Integer("global_output.geometry.max_message_bytes",
                     geometry_output.map.max_message_bytes);
  ValidateGlobalOutputOptions(global);
  gaussian_output.minimum_interval = gaussian.joint_render_interval;
  ValidateGaussianRenderOutputOptions(gaussian_output, global.map_frame);
  auto& options = config.system;
  auto& front = options.frontend;
  auto& visual = front.visual;
  auto& imu = front.imu;
  auto& pre = options.preprocess;
  auto& output = options.exporter;
  auto& colorizer = options.colorizer;
  auto& keyframes = options.keyframes;
  keyframes.enabled = reader.Boolean("keyframes.enabled", keyframes.enabled);
  keyframes.translation_threshold = reader.Number(
      "keyframes.translation_threshold", keyframes.translation_threshold);
  keyframes.rotation_threshold_deg = reader.Number(
      "keyframes.rotation_threshold_deg", keyframes.rotation_threshold_deg);
  keyframes.time_threshold =
      reader.Number("keyframes.time_threshold", keyframes.time_threshold);
  auto& submaps = options.submaps;
  submaps.enabled = reader.Boolean("submaps.enabled", submaps.enabled);
  submaps.max_keyframes =
      reader.Integer("submaps.max_keyframes", submaps.max_keyframes);
  const auto submap_mode = reader.String("submaps.mode", "keyframe_groups");
  if (submap_mode == "online") {
    options.submap_mode = SubmapMode::kOnline;
    auto& online = options.online_submaps;
    online.translation_threshold = reader.Number(
        "submaps.online.translation_threshold", online.translation_threshold);
    online.rotation_threshold_deg = reader.Number(
        "submaps.online.rotation_threshold_deg", online.rotation_threshold_deg);
    online.time_threshold =
        reader.Number("submaps.online.time_threshold", online.time_threshold);
    online.support_keyframes = reader.Integer(
        "submaps.online.support_keyframes", online.support_keyframes);
  } else if (submap_mode != "keyframe_groups") {
    throw std::invalid_argument(
        "submaps.mode must be keyframe_groups or online");
  }
  auto& geometry = options.geometry;
  geometry.enabled =
      reader.Boolean("submaps.geometry.enabled", geometry.enabled);
  if (geometry.enabled || geometry_output.enabled) {
    geometry.voxel_size =
        reader.Number("submaps.geometry.voxel_size", geometry.voxel_size);
    geometry.min_points_per_voxel = reader.Integer(
        "submaps.geometry.min_points_per_voxel", geometry.min_points_per_voxel);
    geometry.max_plane_rms =
        reader.Number("submaps.geometry.max_plane_rms", geometry.max_plane_rms);
    geometry.max_plane_ratio = reader.Number("submaps.geometry.max_plane_ratio",
                                             geometry.max_plane_ratio);
    geometry.max_voxels_per_submap =
        reader.Integer("submaps.geometry.max_voxels_per_submap",
                       geometry.max_voxels_per_submap);
    geometry.max_points_per_keyframe =
        reader.Integer("submaps.geometry.max_points_per_keyframe",
                       geometry.max_points_per_keyframe);
    geometry.max_resident_submaps = reader.Integer(
        "submaps.geometry.max_resident_submaps", geometry.max_resident_submaps);
    ValidateGeometryOptions(geometry);
  }
  config.lidar_topic = reader.String("common.lid_topic", config.lidar_topic);
  config.imu_topic = reader.String("common.imu_topic", config.imu_topic);
  options.driver_fix =
      reader.Boolean("common.ros_driver_bug_fix", options.driver_fix);
  options.buffer_before_first_lidar = reader.Boolean(
      "common.buffer_before_first_lidar", options.buffer_before_first_lidar);
  options.image_enabled =
      reader.Integer("common.img_en", options.image_enabled ? 1 : 0) != 0;
  options.lidar_enabled =
      reader.Integer("common.lidar_en", options.lidar_enabled ? 1 : 0) != 0;
  config.image_topic = reader.String("common.img_topic", config.image_topic);
  config.image_queue_size =
      reader.Integer("publish.image_queue_size", config.image_queue_size);
  if (config.image_queue_size <= 0)
    throw std::invalid_argument(reader.Name("publish.image_queue_size") +
                                ": must be positive");
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
  colorizer.minimum_color_distance = reader.Number(
      "publish.blind_rgb_points", colorizer.minimum_color_distance);
  colorizer.scan_interval =
      reader.Integer("publish.pub_scan_num", colorizer.scan_interval);
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
  auto& voxel_options = front.voxel_map;
  voxel_options.is_pub_plane_map_ =
      reader.Boolean("publish.pub_plane_en", voxel_options.is_pub_plane_map_);
  voxel_options.max_layer_ =
      reader.Integer("lio.max_layer", voxel_options.max_layer_);
  voxel_options.max_voxel_size_ =
      reader.Number("lio.voxel_size", voxel_options.max_voxel_size_);
  voxel_options.planner_threshold_ =
      reader.Number("lio.min_eigen_value", voxel_options.planner_threshold_);
  voxel_options.sigma_num_ =
      reader.Number("lio.sigma_num", voxel_options.sigma_num_);
  voxel_options.beam_err_ =
      reader.Number("lio.beam_err", voxel_options.beam_err_);
  voxel_options.dept_err_ =
      reader.Number("lio.dept_err", voxel_options.dept_err_);
  voxel_options.layer_init_num_ =
      reader.IntegerArray("lio.layer_init_num", voxel_options.layer_init_num_);
  voxel_options.max_points_num_ =
      reader.Integer("lio.max_points_num", voxel_options.max_points_num_);
  voxel_options.max_iterations_ =
      reader.Integer("lio.max_iterations", voxel_options.max_iterations_);
  voxel_options.map_sliding_en =
      reader.Boolean("local_map.map_sliding_en", voxel_options.map_sliding_en);
  voxel_options.half_map_size =
      reader.Integer("local_map.half_map_size", voxel_options.half_map_size);
  voxel_options.sliding_thresh =
      reader.Number("local_map.sliding_thresh", voxel_options.sliding_thresh);
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
  if (config.loop_closure_enabled &&
      (!options.keyframes.enabled || !options.submaps.enabled ||
       options.submap_mode != SubmapMode::kOnline ||
       options.submaps.max_keyframes > static_cast<int>(kMaxGeometryKeyframes)))
    throw std::invalid_argument(
        "loop_closure.enabled requires keyframes and online submaps with at "
        "most 64 members");
  if (global.enabled &&
      (!options.keyframes.enabled || !options.submaps.enabled ||
       options.submap_mode != SubmapMode::kOnline))
    throw std::invalid_argument(
        "global_output.enabled requires keyframes and online submaps");
  if (gaussian.enabled &&
      (!options.keyframes.enabled || !options.submaps.enabled ||
       options.submap_mode != SubmapMode::kOnline))
    throw std::invalid_argument(
        "gaussian.enabled requires keyframes and online submaps");
  if (geometry_output.enabled &&
      options.submaps.max_keyframes > static_cast<int>(kMaxGeometryKeyframes))
    throw std::invalid_argument(
        "Geometry map output requires at most 64 members per submap");
  return config;
}
}  // namespace livo_atlas
