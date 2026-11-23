#include "livo_atlas/ros2/livo_node.h"

#include <chrono>
#include <exception>
#include <optional>
#include <rclcpp/qos_event.hpp>
#include <string>
#include <utility>

#include "livo_atlas/common/logging.h"
#include "livo_atlas/ros2/measurement_conversions.h"
#include "ros_common/input_diagnostics.h"

namespace livo_atlas::ros2 {
namespace {

using ros_detail::Sensor;
using ros_detail::SensorInput;

template <typename Callback>
void RunCallback(SensorInput& input, rclcpp::Context& context,
                 Callback callback) {
  try {
    callback();
  } catch (...) {
    input.Fail(std::current_exception());
    try {
      context.shutdown("LIVO callback failed");
    } catch (const std::exception& error) {
      LOG(ERROR) << "ROS2 shutdown failed: " << error.what();
    } catch (...) {
      LOG(ERROR) << "ROS2 shutdown failed: unknown exception";
    }
  }
}

template <typename Message, typename Sample, typename Converter>
typename rclcpp::Subscription<Message>::SharedPtr SubscribeSensor(
    rclcpp::Node& node, const std::string& topic, int depth,
    const std::string& reliability, rclcpp::CallbackGroup::SharedPtr group,
    SensorInput& input, Sensor sensor, Converter convert) {
  auto qos = rclcpp::SensorDataQoS().keep_last(depth);
  if (reliability == "reliable") qos.reliable();
  rclcpp::SubscriptionOptions options;
  options.callback_group = std::move(group);
  options.event_callbacks.message_lost_callback =
      [&input, sensor](rclcpp::QOSMessageLostInfo& info) {
        if (info.total_count_change > 0) {
          input.RecordTransportLoss(sensor, info.total_count_change);
        }
      };
  auto callback = [&input, sensor, convert,
                   context = node.get_node_base_interface()->get_context()](
                      typename Message::ConstSharedPtr message) {
    RunCallback(input, *context, [&] {
      ros_detail::ReceiveInput<Sample>(input, sensor, [&](Sample* sample) {
        return convert(*message, sample);
      });
    });
  };
  try {
    return node.create_subscription<Message>(topic, qos, callback, options);
  } catch (const rclcpp::UnsupportedEventTypeException&) {
    LOG(INFO) << "ROS2 message loss events unavailable for " << topic;
    options.event_callbacks.message_lost_callback = {};
    return node.create_subscription<Message>(topic, qos, callback, options);
  }
}

}  // namespace

LivoNode::LivoNode(rclcpp::Node& node, const NodeOptions& parameters,
                   LivoSystem& system,
                   std::function<void()> poll_background_outputs)
    : parameters_(parameters),
      system_(system),
      poll_background_outputs_(std::move(poll_background_outputs)),
      publisher_(node, parameters.livo.system.frontend_mode(),
                 parameters.livo.image_queue_size,
                 parameters.livo.global_output, parameters.livo.path_max_poses,
                 parameters.livo.system.stage_timing),
      context_(node.get_node_base_interface()->get_context()),
      input_(parameters.livo.system.sensor_buffer,
             parameters.livo.system.stage_timing) {
  const auto& config = parameters_.livo;
  const auto& options = config.system;
  const auto& depths = config.subscription_depths;
  // Distinct mutually exclusive groups preserve subscription order while
  // allowing conversion to run concurrently with the system.
  constexpr auto kExclusive = rclcpp::CallbackGroupType::MutuallyExclusive;
  processing_group_ = node.create_callback_group(kExclusive);
  if (options.lidar_enabled) {
    lidar_group_ = node.create_callback_group(kExclusive);
    if (options.preprocess.lidar_type == kLivox) {
      livox_ = SubscribeSensor<livox_ros_driver2::msg::CustomMsg, LidarScan>(
          node, config.lidar_topic, depths.lidar, parameters_.reliability,
          lidar_group_, input_, Sensor::kLidar, ConvertLivox);
    } else {
      cloud_ = SubscribeSensor<sensor_msgs::msg::PointCloud2, LidarScan>(
          node, config.lidar_topic, depths.lidar, parameters_.reliability,
          lidar_group_, input_, Sensor::kLidar,
          [type = options.preprocess.lidar_type,
           offset = config.lidar_time_offset](const auto& message,
                                              LidarScan* scan) {
            return ConvertPointCloud(message, type, offset, scan);
          });
    }
  }
  if (options.frontend.imu.enabled) {
    imu_group_ = node.create_callback_group(kExclusive);
    imu_ = SubscribeSensor<sensor_msgs::msg::Imu, Imu>(
        node, config.imu_topic, depths.imu, parameters_.reliability, imu_group_,
        input_, Sensor::kImu, [](const auto& message, Imu* imu) {
          *imu = ConvertImu(message);
          return std::string{};
        });
  }
  if (options.image_enabled) {
    image_group_ = node.create_callback_group(kExclusive);
    image_ = SubscribeSensor<sensor_msgs::msg::Image, TimedImage>(
        node, config.image_topic, depths.image, parameters_.reliability,
        image_group_, input_, Sensor::kImage,
        [offset = config.image_time_offset](const auto& message,
                                            TimedImage* image) {
          return ConvertImage(message, offset, image);
        });
  }
  processing_ = node.create_wall_timer(
      std::chrono::milliseconds(1),
      [this] { RunCallback(input_, *context_, [this] { ProcessNext(); }); },
      processing_group_);
  LOG(INFO) << "livo_atlas frontend ready path_max_poses="
            << config.path_max_poses;
}

SyncStatus LivoNode::ProcessNext() {
  const auto status = input_.ProcessNext(
      system_,
      [this](const auto& result, const auto& colored_cloud, double elapsed) {
        const bool publish = context_->is_valid();
        if (publish) {
          std::optional<MapCorrection> correction;
          if (parameters_.livo.global_output.enabled &&
              result.kind != UpdateKind::kVisual) {
            correction = system_.GetMapCorrection();
          }
          publisher_.Publish(result, colored_cloud, correction);
        }
        ros_detail::LogFrontendUpdate(result.kind, elapsed, publish);
      });
  if (parameters_.livo.system.propagate_imu && context_->is_valid()) {
    const auto now = std::chrono::steady_clock::now();
    // Share the owner turn with processing, including under a ready backlog.
    if (now >= next_propagation_) {
      next_propagation_ = now + std::chrono::milliseconds(4);
      if (const auto result = system_.PropagateLatest()) {
        publisher_.PublishImuPropagation(result->state, result->timestamp);
        if (parameters_.livo.system.stage_timing) {
          ros_detail::LogPropagationInterval(&last_propagation_);
        }
      }
    }
  }
  if (poll_background_outputs_) poll_background_outputs_();
  return status;
}

void LivoNode::RethrowCallbackFailure() { input_.RethrowFailure(); }

void LivoNode::DrainInput() { input_.Drain(system_); }

void LivoNode::ReportInputs() { input_.Report(system_); }

LivoNode::~LivoNode() = default;

}  // namespace livo_atlas::ros2
