#include "livo_atlas/ros1/livo_node.h"

#include <atomic>
#include <exception>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>

#include "livo_atlas/common/logging.h"
#include "livo_atlas/ros1/livo_publisher.h"
#include "livo_atlas/ros1/measurement_conversions.h"
#include "ros/input_diagnostics.h"

namespace livo_atlas::ros1 {

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
    ros::requestShutdown();
  }
}

LivoNode::LivoNode(ros::NodeHandle& node, const NodeOptions& parameters,
                   LivoSystem& system,
                   std::function<void()> poll_background_outputs)
    : parameters_(parameters),
      system_(system),
      poll_background_outputs_(std::move(poll_background_outputs)),
      transport_(node),
      publisher_(parameters.livo.system.exporter.mode,
                 parameters.livo.image_queue_size,
                 parameters.livo.global_output) {
  publisher_.Initialize(node, transport_);
  const auto& input = parameters_.livo;
  if (input.system.preprocess.lidar_type == kLivox) {
    lidar_ = node.subscribe<livox_ros_driver::CustomMsg>(
        input.lidar_topic, 200000,
        [this](const livox_ros_driver::CustomMsg::ConstPtr& message) {
          RunCallback([&] {
            lidar_count_.fetch_add(1, std::memory_order_relaxed);
            LidarScan scan;
            const auto error = ConvertLivox(*message, &scan);
            if (!error.empty()) {
              LOG(WARNING) << error;
              return;
            }
            const std::lock_guard<std::mutex> lock(system_mutex_);
            if (!callback_failure_) system_.AddLidar(scan);
          });
        });
  } else {
    lidar_ = node.subscribe<sensor_msgs::PointCloud2>(
        input.lidar_topic, 200000,
        [this](const sensor_msgs::PointCloud2::ConstPtr& message) {
          RunCallback([&] {
            lidar_count_.fetch_add(1, std::memory_order_relaxed);
            LidarScan scan;
            const auto error = ConvertPointCloud(
                *message, parameters_.livo.system.preprocess.lidar_type,
                parameters_.livo.lidar_time_offset, &scan);
            if (!error.empty()) {
              LOG(WARNING) << error;
              return;
            }
            const std::lock_guard<std::mutex> lock(system_mutex_);
            if (!callback_failure_) system_.AddLidar(scan);
          });
        });
  }
  imu_ = node.subscribe<sensor_msgs::Imu>(
      input.imu_topic, 200000,
      [this](const sensor_msgs::Imu::ConstPtr& message) {
        RunCallback([&] {
          imu_count_.fetch_add(1, std::memory_order_relaxed);
          try {
            auto measurement = ConvertImu(*message);
            const std::lock_guard<std::mutex> lock(system_mutex_);
            if (!callback_failure_) system_.AddImu(std::move(measurement));
          } catch (const std::invalid_argument& error) {
            LOG(WARNING) << error.what();
          }
        });
      });
  image_ = node.subscribe<sensor_msgs::Image>(
      input.image_topic, 200000,
      [this](const sensor_msgs::Image::ConstPtr& message) {
        RunCallback([&] {
          image_count_.fetch_add(1, std::memory_order_relaxed);
          TimedImage image;
          const auto error = ConvertImage(
              *message, parameters_.livo.image_time_offset, &image);
          if (!error.empty()) {
            LOG(WARNING) << error;
            return;
          }
          const std::lock_guard<std::mutex> lock(system_mutex_);
          if (!callback_failure_) system_.AddImage(std::move(image));
        });
      });
  propagation_ =
      node.createTimer(ros::Duration(.004), [this](const ros::TimerEvent&) {
        RunCallback([this] {
          const std::lock_guard<std::mutex> lock(system_mutex_);
          if (callback_failure_) return;
          const auto result = system_.PropagateLatest();
          if (result) {
            ros::Time stamp;
            stamp.fromSec(result->timestamp);
            publisher_.PublishImuPropagation(result->state, stamp);
          }
        });
      });
  LOG(INFO) << "livo_atlas frontend ready";
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
        ros_detail::LogUpdateTime(result.kind, elapsed);
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

}  // namespace livo_atlas::ros1
