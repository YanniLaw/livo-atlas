#include "livo_atlas/ros2/livo_node.h"

#include <atomic>
#include <chrono>
#include <exception>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>

#include "livo_atlas/common/logging.h"
#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/ros2/livo_publisher.h"
#include "livo_atlas/ros2/measurement_conversions.h"
#include "livo_atlas/ros2/node_options.h"
#include "ros_common/input_diagnostics.h"

namespace livo_atlas::ros2 {
namespace {

rclcpp::QoS SensorQos(const NodeOptions& parameters, int depth) {
  auto qos = rclcpp::SensorDataQoS().keep_last(depth);
  if (parameters.reliability == "reliable") qos.reliable();
  return qos;
}

}  // namespace

template <typename Callback>
void LivoNode::RunCallback(Callback callback) {
  try {
    callback();
  } catch (...) {
    {
      const std::lock_guard<std::mutex> lock(system_mutex_);
      if (callback_failure_) return;
      callback_failure_ = std::current_exception();
    }
    try {
      context_->shutdown("LIVO callback failed");
    } catch (const std::exception& error) {
      LOG(ERROR) << "ROS2 shutdown failed: " << error.what();
    } catch (...) {
      LOG(ERROR) << "ROS2 shutdown failed: unknown exception";
    }
  }
}

LivoNode::LivoNode(rclcpp::Node& node, const NodeOptions& parameters,
                   LivoSystem& system,
                   std::function<void()> poll_background_outputs)
    : parameters_(parameters),
      system_(system),
      poll_background_outputs_(std::move(poll_background_outputs)),
      publisher_(node, parameters_.livo.system.frontend_mode(),
                 parameters_.livo.image_queue_size,
                 parameters_.livo.global_output,
                 parameters_.livo.path_max_poses,
                 parameters_.livo.system.stage_timing),
      context_(node.get_node_base_interface()->get_context()) {
  // Keep message order within each subscription while allowing different
  // sensors to convert messages independently.
  constexpr auto kExclusive = rclcpp::CallbackGroupType::MutuallyExclusive;
  lidar_group_ = node.create_callback_group(kExclusive);
  imu_group_ = node.create_callback_group(kExclusive);
  image_group_ = node.create_callback_group(kExclusive);
  processing_group_ = node.create_callback_group(kExclusive);
  rclcpp::SubscriptionOptions lidar_options;
  lidar_options.callback_group = lidar_group_;
  rclcpp::SubscriptionOptions imu_options;
  imu_options.callback_group = imu_group_;
  rclcpp::SubscriptionOptions image_options;
  image_options.callback_group = image_group_;

  if (parameters_.livo.system.preprocess.lidar_type == kLivox) {
    livox_ = node.create_subscription<livox_ros_driver2::msg::CustomMsg>(
        parameters_.livo.lidar_topic,
        SensorQos(parameters_, parameters_.lidar_depth),
        [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message) {
          RunCallback([&] {
            lidar_count_.fetch_add(1, std::memory_order_relaxed);
            LidarScan scan;
            ros_detail::InputTiming timing(
                parameters_.livo.system.stage_timing, "lidar_conversion",
                "lidar_lock_wait");
            const auto error = ConvertLivox(*message, &scan);
            timing.Converted();
            if (!error.empty()) {
              LOG(WARNING) << error;
              return;
            }
            const std::lock_guard<std::mutex> lock(system_mutex_);
            timing.LockAcquired();
            if (!callback_failure_) system_.AddLidar(scan);
          });
        },
        lidar_options);
  } else {
    cloud_ = node.create_subscription<sensor_msgs::msg::PointCloud2>(
        parameters_.livo.lidar_topic,
        SensorQos(parameters_, parameters_.lidar_depth),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
          RunCallback([&] {
            lidar_count_.fetch_add(1, std::memory_order_relaxed);
            LidarScan scan;
            ros_detail::InputTiming timing(
                parameters_.livo.system.stage_timing, "lidar_conversion",
                "lidar_lock_wait");
            const auto error = ConvertPointCloud(
                *message, parameters_.livo.system.preprocess.lidar_type,
                parameters_.livo.lidar_time_offset, &scan);
            timing.Converted();
            if (!error.empty()) {
              LOG(WARNING) << error;
              return;
            }
            const std::lock_guard<std::mutex> lock(system_mutex_);
            timing.LockAcquired();
            if (!callback_failure_) system_.AddLidar(scan);
          });
        },
        lidar_options);
  }
  imu_ = node.create_subscription<sensor_msgs::msg::Imu>(
      parameters_.livo.imu_topic, SensorQos(parameters_, parameters_.imu_depth),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
        RunCallback([&] {
          imu_count_.fetch_add(1, std::memory_order_relaxed);
          try {
            ros_detail::InputTiming timing(parameters_.livo.system.stage_timing,
                                           "imu_conversion", "imu_lock_wait");
            auto measurement = ConvertImu(*message);
            timing.Converted();
            const std::lock_guard<std::mutex> lock(system_mutex_);
            timing.LockAcquired();
            if (!callback_failure_) system_.AddImu(std::move(measurement));
          } catch (const std::invalid_argument& error) {
            LOG(WARNING) << error.what();
          }
        });
      },
      imu_options);
  image_ = node.create_subscription<sensor_msgs::msg::Image>(
      parameters_.livo.image_topic,
      SensorQos(parameters_, parameters_.image_depth),
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        RunCallback([&] {
          image_count_.fetch_add(1, std::memory_order_relaxed);
          TimedImage image;
          ros_detail::InputTiming timing(parameters_.livo.system.stage_timing,
                                         "image_conversion", "image_lock_wait");
          const auto error = ConvertImage(
              *message, parameters_.livo.image_time_offset, &image);
          timing.Converted();
          if (!error.empty()) {
            LOG(WARNING) << error;
            return;
          }
          const std::lock_guard<std::mutex> lock(system_mutex_);
          timing.LockAcquired();
          if (!callback_failure_) system_.AddImage(std::move(image));
        });
      },
      image_options);
  processing_ = node.create_wall_timer(
      std::chrono::milliseconds(1),
      [this] {
        RunCallback([this] {
          while (ProcessNext() != SyncStatus::kWaiting) {
          }
        });
      },
      processing_group_);
  propagation_ = node.create_wall_timer(
      std::chrono::milliseconds(4),
      [this] {
        RunCallback([this] {
          const std::lock_guard<std::mutex> lock(system_mutex_);
          if (callback_failure_) return;
          const auto result = system_.PropagateLatest();
          if (result) {
            publisher_.PublishImuPropagation(result->state, result->timestamp);
            if (parameters_.livo.system.stage_timing) {
              ros_detail::LogPropagationInterval(&last_propagation_);
            }
          }
        });
      },
      processing_group_);
  LOG(INFO) << "livo_atlas frontend ready path_max_poses="
            << parameters_.livo.path_max_poses;
}

SyncStatus LivoNode::ProcessNext() {
  const std::lock_guard<std::mutex> lock(system_mutex_);
  if (callback_failure_) std::rethrow_exception(callback_failure_);
  const auto status = system_.ProcessNext(
      [this](const auto& result, const auto& colored_cloud, double elapsed) {
        std::optional<MapCorrection> correction;
        if (parameters_.livo.global_output.enabled && result.updated &&
            result.kind != UpdateKind::kVisual) {
          correction = system_.GetMapCorrection();
        }
        publisher_.Publish(result, colored_cloud, correction);
        ros_detail::LogFrontendUpdate(result.kind, elapsed);
      });
  if (poll_background_outputs_) poll_background_outputs_();
  return status;
}

void LivoNode::RethrowCallbackFailure() {
  const std::lock_guard<std::mutex> lock(system_mutex_);
  if (callback_failure_) std::rethrow_exception(callback_failure_);
}

void LivoNode::ReportInputs() {
  const std::lock_guard<std::mutex> lock(system_mutex_);
  ros_detail::LogInputDiagnostics(lidar_count_.load(std::memory_order_relaxed),
                                 imu_count_.load(std::memory_order_relaxed),
                                 image_count_.load(std::memory_order_relaxed),
                                 system_.GetStartupBufferStats());
}

LivoNode::~LivoNode() = default;

}  // namespace livo_atlas::ros2
