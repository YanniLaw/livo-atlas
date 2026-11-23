#include "livo_atlas/ros1/livo_node.h"

#include <ros/message_event.h>

#include <cstdint>
#include <exception>
#include <optional>
#include <string>
#include <utility>

#include "livo_atlas/common/logging.h"
#include "livo_atlas/ros1/measurement_conversions.h"
#include "ros_common/input_diagnostics.h"

namespace livo_atlas::ros1 {
namespace {

using ros_detail::Sensor;
using ros_detail::SensorInput;

template <typename Message, typename Sample, typename Converter>
ros::Subscriber SubscribeSensor(ros::NodeHandle& node, const std::string& topic,
                                int depth, SensorInput& input, Sensor sensor,
                                Converter convert) {
  using Event = ros::MessageEvent<const Message>;
  return node.subscribe<Message, const Event&>(
      topic, depth,
      [&input, sensor, convert, connection = boost::shared_ptr<ros::M_string>(),
       sequence = std::uint32_t{0}](const Event& event) mutable {
        try {
          const auto& message = *event.getMessage();
          const auto& header = event.getConnectionHeaderPtr();
          const std::uint32_t delta = message.header.seq - sequence;
          // Only count forward gaps on the same publisher connection.
          if (header && header == connection && delta > 1 &&
              delta < 0x80000000u) {
            input.RecordTransportLoss(sensor, delta - 1);
          }
          connection = header;
          sequence = message.header.seq;
          ros_detail::ReceiveInput<Sample>(input, sensor, [&](Sample* sample) {
            return convert(message, sample);
          });
        } catch (...) {
          input.Fail(std::current_exception());
          ros::requestShutdown();
        }
      });
}

}  // namespace

LivoNode::LivoNode(ros::NodeHandle& node, const NodeOptions& parameters,
                   LivoSystem& system,
                   std::function<void()> poll_background_outputs)
    : parameters_(parameters),
      system_(system),
      poll_background_outputs_(std::move(poll_background_outputs)),
      transport_(node),
      publisher_(parameters.livo.system.frontend_mode(),
                 parameters.livo.image_queue_size,
                 parameters.livo.global_output, parameters.livo.path_max_poses,
                 parameters.livo.system.stage_timing),
      input_(parameters.livo.system.sensor_buffer,
             parameters.livo.system.stage_timing) {
  publisher_.Initialize(node, transport_);
  const auto& config = parameters_.livo;
  const auto& options = config.system;
  const auto& depths = config.subscription_depths;
  if (options.lidar_enabled) {
    if (options.preprocess.lidar_type == kLivox) {
      lidar_ = SubscribeSensor<livox_ros_driver::CustomMsg, LidarScan>(
          node, config.lidar_topic, depths.lidar, input_, Sensor::kLidar,
          ConvertLivox);
    } else {
      lidar_ = SubscribeSensor<sensor_msgs::PointCloud2, LidarScan>(
          node, config.lidar_topic, depths.lidar, input_, Sensor::kLidar,
          [type = options.preprocess.lidar_type,
           offset = config.lidar_time_offset](const auto& message,
                                              LidarScan* scan) {
            return ConvertPointCloud(message, type, offset, scan);
          });
    }
  }
  if (options.frontend.imu.enabled) {
    imu_ = SubscribeSensor<sensor_msgs::Imu, Imu>(
        node, config.imu_topic, depths.imu, input_, Sensor::kImu,
        [](const auto& message, Imu* imu) {
          *imu = ConvertImu(message);
          return std::string{};
        });
  }
  if (options.image_enabled) {
    image_ = SubscribeSensor<sensor_msgs::Image, TimedImage>(
        node, config.image_topic, depths.image, input_, Sensor::kImage,
        [offset = config.image_time_offset](const auto& message,
                                            TimedImage* image) {
          return ConvertImage(message, offset, image);
        });
  }
  if (options.propagate_imu) {
    propagation_ =
        node.createTimer(ros::Duration(.004), [this](const ros::TimerEvent&) {
          propagation_due_.store(true, std::memory_order_relaxed);
        });
  }
  LOG(INFO) << "livo_atlas frontend ready path_max_poses="
            << config.path_max_poses;
}

SyncStatus LivoNode::ProcessNext() {
  const auto status = input_.ProcessNext(
      system_,
      [this](const auto& result, const auto& colored_cloud, double elapsed) {
        const bool publish = ros::ok();
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
  if (propagation_due_.exchange(false, std::memory_order_relaxed) &&
      ros::ok()) {
    if (const auto result = system_.PropagateLatest()) {
      ros::Time stamp;
      stamp.fromSec(result->timestamp);
      publisher_.PublishImuPropagation(result->state, stamp);
      if (parameters_.livo.system.stage_timing) {
        ros_detail::LogPropagationInterval(&last_propagation_);
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

}  // namespace livo_atlas::ros1
