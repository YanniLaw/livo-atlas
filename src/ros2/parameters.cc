#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>
#include <stdexcept>
#include <utility>
#include <vector>

#include "livo_atlas/frontend/image_options.h"
#include "ros2/parameters.h"

namespace livo_atlas::ros2 {
namespace {
rclcpp::Parameter Declare(rclcpp::Node& node, const std::string& name,
                          const rclcpp::ParameterValue& value,
                          bool numeric = false) {
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  descriptor.dynamic_typing = numeric;
  node.declare_parameter(name, value, descriptor);
  return node.get_parameter(name);
}
double Number(rclcpp::Node& node, const std::string& name, double fallback) {
  const auto value =
      Declare(node, name, rclcpp::ParameterValue(fallback), true);
  double number;
  if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
    number = value.as_int();
  else if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    number = value.as_double();
  else
    throw std::invalid_argument(name + " must be numeric");
  if (!std::isfinite(number))
    throw std::invalid_argument(name + " must be finite");
  return number;
}
int Integer(rclcpp::Node& node, const std::string& name, int fallback) {
  const auto value =
      Declare(node, name, rclcpp::ParameterValue(fallback)).as_int();
  if (value < std::numeric_limits<int>::min() ||
      value > std::numeric_limits<int>::max()) {
    throw std::invalid_argument(name + " is outside the integer range");
  }
  return static_cast<int>(value);
}
bool Boolean(rclcpp::Node& node, const std::string& name, bool fallback) {
  return Declare(node, name, rclcpp::ParameterValue(fallback)).as_bool();
}
std::string String(rclcpp::Node& node, const std::string& name,
                   const std::string& fallback) {
  return Declare(node, name, rclcpp::ParameterValue(fallback)).as_string();
}
std::vector<double> Array(rclcpp::Node& node, const std::string& name) {
  const auto value =
      Declare(node, name, rclcpp::ParameterValue(std::vector<double>{}), true);
  if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
    return value.as_double_array();
  if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
    const auto& values = value.as_integer_array();
    return {values.begin(), values.end()};
  }
  throw std::invalid_argument(name + " must be a numeric array");
}
RigidTransform Transform(rclcpp::Node& node, const std::string& rotation,
                         const std::string& translation) {
  const auto r = Array(node, rotation);
  const auto t = Array(node, translation);
  RigidTransform result;
  const auto error = ParseExtrinsic(r, t, &result);
  if (!error.empty())
    throw std::invalid_argument(rotation + " / " + translation + ": " + error);
  return result;
}
std::string DefaultDirectory() {
  const char* base = std::getenv("ROS_HOME");
  if (base != nullptr && *base != '\0')
    return (std::filesystem::path(base) / "livo_atlas").string();
  base = std::getenv("HOME");
  if (base != nullptr && *base != '\0')
    return (std::filesystem::path(base) / ".ros/livo_atlas").string();
  return (std::filesystem::current_path() / "livo_atlas_output").string();
}
}  // namespace
NodeParameters ReadParameters(rclcpp::Node& node) {
  NodeParameters config;
  auto& options = config.session;
  auto& front = options.frontend;
  auto& visual = front.visual;
  auto& imu = front.imu;
  auto& pre = options.preprocess;
  auto& output = options.output;
  auto& processor = options.processor;
  config.lidar_topic = String(node, "common.lid_topic", "/livox/lidar");
  config.imu_topic = String(node, "common.imu_topic", "/livox/imu");
  options.driver_fix = Boolean(node, "common.ros_driver_bug_fix", false);
  options.image_enabled = Integer(node, "common.img_en", 1) != 0;
  options.lidar_enabled = Integer(node, "common.lidar_en", 1) != 0;
  config.image_topic = String(node, "common.img_topic", "/left_camera/image");
  // Resolve names before constructing the session or opening any outputs.
  for (const auto& topic :
       {config.lidar_topic, config.imu_topic, config.image_topic}) {
    node.get_node_topics_interface()->resolve_topic_name(topic);
  }
  visual.normal_enabled = Boolean(node, "vio.normal_en", true);
  visual.inverse_composition =
      Boolean(node, "vio.inverse_composition_en", false);
  visual.max_iterations = Integer(node, "vio.max_iterations", 5);
  visual.image_point_covariance = Number(node, "vio.img_point_cov", 100);
  visual.raycast = Boolean(node, "vio.raycast_en", false);
  visual.estimate_exposure = Boolean(node, "vio.exposure_estimate_en", true);
  imu.inverse_exposure_covariance = Number(node, "vio.inv_expo_cov", 0.2);
  visual.grid_size = Integer(node, "vio.grid_size", 5);
  visual.grid_rows = Integer(node, "vio.grid_n_height", 17);
  visual.patch_pyramid_levels = Integer(node, "vio.patch_pyrimid_level", 3);
  visual.patch_size = Integer(node, "vio.patch_size", 8);
  visual.outlier_threshold = Number(node, "vio.outlier_threshold", 1000);
  options.exposure_time = Number(node, "time_offset.exposure_time_init", 0.0);
  config.image_time_offset = Number(node, "time_offset.img_time_offset", 0.0);
  options.imu_time_offset = Number(node, "time_offset.imu_time_offset", 0.0);
  config.lidar_time_offset = Number(node, "time_offset.lidar_time_offset", 0.0);
  options.propagate_imu = Boolean(node, "uav.imu_rate_odom", false);
  front.align_gravity = Boolean(node, "uav.gravity_align_en", false);
  output.sequence_name = String(node, "evo.seq_name", "01");
  output.trajectory = Boolean(node, "evo.pose_output_en", false);
  imu.gyroscope_covariance = Number(node, "imu.gyr_cov", 1.0);
  imu.accelerometer_covariance = Number(node, "imu.acc_cov", 1.0);
  imu.initialization_frames = Integer(node, "imu.imu_int_frame", 3);
  imu.enabled = Boolean(node, "imu.imu_en", false);
  imu.estimate_gravity = Boolean(node, "imu.gravity_est_en", true);
  imu.estimate_bias = Boolean(node, "imu.ba_bg_est_en", true);
  pre.blind = Number(node, "preprocess.blind", 0.01);
  front.surface_filter_size = Number(node, "preprocess.filter_size_surf", 0.5);
  options.hilti = Boolean(node, "preprocess.hilti_en", false);
  pre.lidar_type = Integer(node, "preprocess.lidar_type", kLivox);
  pre.scan_lines = Integer(node, "preprocess.scan_line", 6);
  pre.point_filter = Integer(node, "preprocess.point_filter_num", 3);
  pre.features = Boolean(node, "preprocess.feature_extract_enabled", false);
  output.pcd_interval = Integer(node, "pcd_save.interval", -1);
  output.pcd = Boolean(node, "pcd_save.pcd_save_en", false);
  output.pcd_type = Integer(node, "pcd_save.type", 0);
  output.images = Boolean(node, "image_save.img_save_en", false);
  output.image_interval = Integer(node, "image_save.interval", 1);
  output.colmap = Boolean(node, "pcd_save.colmap_output_en", false);
  output.voxel_size = Number(node, "pcd_save.filter_size_pcd", 0.5);
  visual.plot_time = Number(node, "debug.plot_time", -10);
  visual.plot_frame_count = Integer(node, "debug.frame_cnt", 6);
  processor.minimum_color_distance =
      Number(node, "publish.blind_rgb_points", 0.01);
  processor.scan_interval = Integer(node, "publish.pub_scan_num", 1);
  front.output_effective_points =
      Boolean(node, "publish.pub_effect_point_en", false);
  front.dense_cloud = Boolean(node, "publish.dense_map_en", false);
  output.directory = String(node, "output.directory", DefaultDirectory());
  auto& calibration = front.calibration;
  calibration.imu_from_lidar =
      Transform(node, "extrin_calib.extrinsic_R", "extrin_calib.extrinsic_T");
  calibration.camera_from_lidar =
      Transform(node, "extrin_calib.Rcl", "extrin_calib.Pcl");
  auto& camera = calibration.camera;
  camera.model = String(node, "cam_model", "");
  camera.width = Integer(node, "cam_width", 0);
  camera.height = Integer(node, "cam_height", 0);
  camera.fx = Number(node, "cam_fx", 0);
  camera.fy = Number(node, "cam_fy", 0);
  camera.cx = Number(node, "cam_cx", 0);
  camera.cy = Number(node, "cam_cy", 0);
  camera.scale = Number(node, "scale", 1);
  for (int i = 0; i < 4; ++i) {
    const std::string name = camera.model == "Pinhole"
                                 ? "cam_d" + std::to_string(i)
                                 : "k" + std::to_string(i + 1);
    camera.distortion[i] = Number(node, name, 0);
  }
  if (visual.image_point_covariance <= 0 || imu.accelerometer_covariance < 0 ||
      imu.gyroscope_covariance < 0 || imu.inverse_exposure_covariance < 0) {
    throw std::invalid_argument(
        "vio.img_point_cov must be positive; IMU and exposure covariances must "
        "be nonnegative");
  }
  const auto error = ValidateImageOptions(camera, visual);
  if (!error.empty()) throw std::invalid_argument(error);
  auto& map = front.voxel_map;
  map.is_pub_plane_map_ = Boolean(node, "publish.pub_plane_en", false);
  map.max_layer_ = Integer(node, "lio.max_layer", 1);
  map.max_voxel_size_ = Number(node, "lio.voxel_size", 0.5);
  map.planner_threshold_ = Number(node, "lio.min_eigen_value", 0.01);
  map.sigma_num_ = Number(node, "lio.sigma_num", 3);
  map.beam_err_ = Number(node, "lio.beam_err", 0.02);
  map.dept_err_ = Number(node, "lio.dept_err", 0.05);
  const auto layers =
      Declare(node, "lio.layer_init_num",
              rclcpp::ParameterValue(std::vector<int64_t>{5, 5, 5, 5, 5}))
          .as_integer_array();
  map.layer_init_num_.clear();
  for (auto value : layers) {
    if (value <= 0 || value > std::numeric_limits<int>::max())
      throw std::invalid_argument(
          "lio.layer_init_num values must be positive integers");
    map.layer_init_num_.push_back(static_cast<int>(value));
  }
  map.max_points_num_ = Integer(node, "lio.max_points_num", 50);
  map.max_iterations_ = Integer(node, "lio.max_iterations", 5);
  map.map_sliding_en = Boolean(node, "local_map.map_sliding_en", false);
  map.half_map_size = Integer(node, "local_map.half_map_size", 100);
  map.sliding_thresh = Number(node, "local_map.sliding_thresh", 8);
  if (pre.scan_lines <= 0 || pre.scan_lines > 128 || pre.point_filter <= 0 ||
      pre.blind < 0 || pre.lidar_type < kLivox || pre.lidar_type > kRobosense) {
    throw std::invalid_argument(
        "invalid preprocess scan lines, point filter, blind distance or lidar "
        "type");
  }
  if (front.surface_filter_size <= 0 || imu.initialization_frames <= 0 ||
      map.max_layer_ < 0 ||
      static_cast<std::size_t>(map.max_layer_) >= layers.size() ||
      map.max_voxel_size_ <= 0 || map.max_points_num_ <= 0 ||
      map.max_iterations_ <= 0 || visual.max_iterations <= 0 ||
      map.half_map_size <= 0) {
    throw std::invalid_argument(
        "invalid frontend iteration, initialization, filter or map bounds");
  }
  config.reliability = String(node, "qos.reliability", "best_effort");
  config.imu_depth = Integer(node, "qos.imu_depth", 2000);
  config.lidar_depth = Integer(node, "qos.lidar_depth", 100);
  config.image_depth = Integer(node, "qos.image_depth", 20);
  if ((config.reliability != "best_effort" &&
       config.reliability != "reliable") ||
      config.imu_depth <= 0 || config.lidar_depth <= 0 ||
      config.image_depth <= 0) {
    throw std::invalid_argument(
        "qos reliability must be best_effort/reliable and depths must be "
        "positive");
  }
  output.calibration = calibration;
  output.mode = processor.mode = options.image_enabled && options.lidar_enabled
                                     ? FrontendMode::kLidarInertialVisual
                                 : imu.enabled ? FrontendMode::kLidarInertial
                                               : FrontendMode::kLidarOnly;
  processor.calibration = calibration;
  processor.colmap = output.colmap;
  ValidateOutputOptions(output);
  if (processor.scan_interval <= 0)
    throw std::invalid_argument("publish.pub_scan_num must be positive");
  return config;
}
}  // namespace livo_atlas::ros2
