#include "livo_atlas/ros2/mapping_adapter.h"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <utility>

#include "livo_atlas/ros2/measurement_conversions.h"
#include "livo_atlas/ros2/parameters.h"
#include "livo_atlas/ros2/result_publisher.h"
#include "livo_atlas/runtime/mapping_session.h"

namespace livo_atlas::ros2 {
namespace {
rclcpp::QoS SensorQos(const NodeParameters& parameters, int depth) {
  auto qos = rclcpp::SensorDataQoS().keep_last(depth);
  if (parameters.reliability == "reliable") qos.reliable();
  return qos;
}
}  // namespace
class MappingAdapter::Impl {
 public:
  Impl(rclcpp::Node& node, const NodeParameters& parameters,
       MappingSession& session)
      : node_(node),
        parameters_(parameters),
        session_(session),
        publisher_(node, parameters_.mapping.session.output.mode) {
    if (parameters_.mapping.session.preprocess.lidar_type == kLivox) {
      livox_ = node.create_subscription<livox_ros_driver2::msg::CustomMsg>(
          parameters_.mapping.lidar_topic,
          SensorQos(parameters_, parameters_.lidar_depth),
          [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message) {
            ++lidar_count_;
            LidarScan scan;
            const auto error = ConvertLivox(*message, &scan);
            if (error.empty())
              session_.AddLidar(scan);
            else
              Warn(error);
          });
    } else {
      cloud_ = node.create_subscription<sensor_msgs::msg::PointCloud2>(
          parameters_.mapping.lidar_topic,
          SensorQos(parameters_, parameters_.lidar_depth),
          [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
            ++lidar_count_;
            LidarScan scan;
            const auto error = ConvertPointCloud(
                *message, parameters_.mapping.session.preprocess.lidar_type,
                parameters_.mapping.lidar_time_offset, &scan);
            if (error.empty())
              session_.AddLidar(scan);
            else
              Warn(error);
          });
    }
    imu_ = node.create_subscription<sensor_msgs::msg::Imu>(
        parameters_.mapping.imu_topic,
        SensorQos(parameters_, parameters_.imu_depth),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
          ++imu_count_;
          try {
            session_.AddImu(ConvertImu(*message));
          } catch (const std::exception& error) {
            Warn(error.what());
          }
        });
    image_ = node.create_subscription<sensor_msgs::msg::Image>(
        parameters_.mapping.image_topic,
        SensorQos(parameters_, parameters_.image_depth),
        [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
          ++image_count_;
          ImageMeasurement image;
          const auto error = ConvertImage(
              *message, parameters_.mapping.image_time_offset, &image);
          if (error.empty())
            session_.AddImage(std::move(image));
          else
            Warn(error);
        });
    processing_ = node.create_wall_timer(std::chrono::milliseconds(1), [this] {
      while (ProcessNext() != SynchronizationResult::kWaiting) {
      }
    });
    propagation_ = node.create_wall_timer(std::chrono::milliseconds(4), [this] {
      const auto result = session_.PropagateLatest();
      if (result)
        publisher_.PublishImuPropagation(result->state, result->timestamp);
    });
    RCLCPP_INFO(node.get_logger(), "livo_atlas frontend ready");
  }
  SynchronizationResult ProcessNext() {
    return session_.ProcessNext([this](const auto& result, const auto& prepared,
                                       double elapsed) {
      publisher_.Publish(result, prepared);
      const char* stage =
          result.kind == UpdateKind::kVisual ? "VIO Time" : "LIO Mapping Time";
      std::printf("%s\n| Current Total Time | %.6f |\n", stage, elapsed);
    });
  }
  void ReportInputs() {
    std::printf("livo_atlas inputs: lidar=%llu imu=%llu image=%llu\n",
                static_cast<unsigned long long>(lidar_count_),
                static_cast<unsigned long long>(imu_count_),
                static_cast<unsigned long long>(image_count_));
  }
  void Warn(const std::string& error) {
    RCLCPP_WARN(node_.get_logger(), "%s", error.c_str());
  }
  std::uint64_t lidar_count_ = 0, imu_count_ = 0, image_count_ = 0;
  rclcpp::Node& node_;
  NodeParameters parameters_;
  MappingSession& session_;
  ResultPublisher publisher_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_;
  rclcpp::TimerBase::SharedPtr processing_, propagation_;
};
MappingAdapter::MappingAdapter(rclcpp::Node& node,
                               const NodeParameters& parameters,
                               MappingSession& session)
    : impl_(std::make_unique<Impl>(node, parameters, session)) {}
MappingAdapter::~MappingAdapter() = default;
SynchronizationResult MappingAdapter::ProcessNext() {
  return impl_->ProcessNext();
}
void MappingAdapter::ReportInputs() { impl_->ReportInputs(); }
}  // namespace livo_atlas::ros2
