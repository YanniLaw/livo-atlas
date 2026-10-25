#include "livo_atlas/ros2/livo_node.h"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <utility>

#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/ros2/livo_publisher.h"
#include "livo_atlas/ros2/measurement_conversions.h"
#include "livo_atlas/ros2/node_options.h"

namespace livo_atlas::ros2 {
namespace {
rclcpp::QoS SensorQos(const NodeOptions& parameters, int depth) {
  auto qos = rclcpp::SensorDataQoS().keep_last(depth);
  if (parameters.reliability == "reliable") qos.reliable();
  return qos;
}
}  // namespace
class LivoNode::Impl {
 public:
  Impl(rclcpp::Node& node, const NodeOptions& parameters, LivoSystem& system)
      : node_(node),
        parameters_(parameters),
        system_(system),
        publisher_(node, parameters_.livo.system.exporter.mode) {
    if (parameters_.livo.system.preprocess.lidar_type == kLivox) {
      livox_ = node.create_subscription<livox_ros_driver2::msg::CustomMsg>(
          parameters_.livo.lidar_topic,
          SensorQos(parameters_, parameters_.lidar_depth),
          [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message) {
            ++lidar_count_;
            LidarScan scan;
            const auto error = ConvertLivox(*message, &scan);
            if (error.empty())
              system_.AddLidar(scan);
            else
              Warn(error);
          });
    } else {
      cloud_ = node.create_subscription<sensor_msgs::msg::PointCloud2>(
          parameters_.livo.lidar_topic,
          SensorQos(parameters_, parameters_.lidar_depth),
          [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
            ++lidar_count_;
            LidarScan scan;
            const auto error = ConvertPointCloud(
                *message, parameters_.livo.system.preprocess.lidar_type,
                parameters_.livo.lidar_time_offset, &scan);
            if (error.empty())
              system_.AddLidar(scan);
            else
              Warn(error);
          });
    }
    imu_ = node.create_subscription<sensor_msgs::msg::Imu>(
        parameters_.livo.imu_topic,
        SensorQos(parameters_, parameters_.imu_depth),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
          ++imu_count_;
          try {
            system_.AddImu(ConvertImu(*message));
          } catch (const std::exception& error) {
            Warn(error.what());
          }
        });
    image_ = node.create_subscription<sensor_msgs::msg::Image>(
        parameters_.livo.image_topic,
        SensorQos(parameters_, parameters_.image_depth),
        [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
          ++image_count_;
          TimedImage image;
          const auto error = ConvertImage(
              *message, parameters_.livo.image_time_offset, &image);
          if (error.empty())
            system_.AddImage(std::move(image));
          else
            Warn(error);
        });
    processing_ = node.create_wall_timer(std::chrono::milliseconds(1), [this] {
      while (ProcessNext() != SyncStatus::kWaiting) {
      }
    });
    propagation_ = node.create_wall_timer(std::chrono::milliseconds(4), [this] {
      const auto result = system_.PropagateLatest();
      if (result)
        publisher_.PublishImuPropagation(result->state, result->timestamp);
    });
    RCLCPP_INFO(node.get_logger(), "livo_atlas frontend ready");
  }
  SyncStatus ProcessNext() {
    return system_.ProcessNext(
        [this](const auto& result, const auto& colored_cloud, double elapsed) {
          publisher_.Publish(result, colored_cloud);
          const char* stage = result.kind == UpdateKind::kVisual
                                  ? "VIO Time"
                                  : "LIO Mapping Time";
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
  NodeOptions parameters_;
  LivoSystem& system_;
  LivoPublisher publisher_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_;
  rclcpp::TimerBase::SharedPtr processing_, propagation_;
};
LivoNode::LivoNode(rclcpp::Node& node, const NodeOptions& parameters,
                   LivoSystem& system)
    : impl_(std::make_unique<Impl>(node, parameters, system)) {}
LivoNode::~LivoNode() = default;
SyncStatus LivoNode::ProcessNext() { return impl_->ProcessNext(); }
void LivoNode::ReportInputs() { impl_->ReportInputs(); }
}  // namespace livo_atlas::ros2
