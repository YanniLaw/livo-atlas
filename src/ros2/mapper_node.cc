#include <chrono>
#include <cstdio>
#include <exception>
#include <utility>

#include "livo_atlas/ros2/measurement_conversions.h"
#include "ros2/mapper_node.h"
#include "ros2/mapping_session.h"
#include "ros2/parameters.h"
#include "ros2/result_publisher.h"

namespace livo_atlas::ros2 {
namespace {
rclcpp::QoS SensorQos(const NodeParameters& parameters, int depth) {
  auto qos = rclcpp::SensorDataQoS().keep_last(depth);
  if (parameters.reliability == "reliable") qos.reliable();
  return qos;
}
}  // namespace
class MapperNode::Impl {
 public:
  explicit Impl(MapperNode& node)
      : node_(node),
        parameters_(ReadParameters(node)),
        session_(parameters_.session),
        publisher_(node, parameters_.session.output.mode) {
    if (parameters_.session.preprocess.lidar_type == kLivox) {
      livox_ = node.create_subscription<livox_ros_driver2::msg::CustomMsg>(
          parameters_.lidar_topic,
          SensorQos(parameters_, parameters_.lidar_depth),
          [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message) {
            LidarScan scan;
            const auto error = ConvertLivox(*message, &scan);
            if (error.empty())
              session_.AddLidar(scan);
            else
              Warn(error);
          });
    } else {
      cloud_ = node.create_subscription<sensor_msgs::msg::PointCloud2>(
          parameters_.lidar_topic,
          SensorQos(parameters_, parameters_.lidar_depth),
          [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
            LidarScan scan;
            const auto error = ConvertPointCloud(
                *message, parameters_.session.preprocess.lidar_type,
                parameters_.lidar_time_offset, &scan);
            if (error.empty())
              session_.AddLidar(scan);
            else
              Warn(error);
          });
    }
    imu_ = node.create_subscription<sensor_msgs::msg::Imu>(
        parameters_.imu_topic, SensorQos(parameters_, parameters_.imu_depth),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
          try {
            session_.AddImu(ConvertImu(*message));
          } catch (const std::exception& error) {
            Warn(error.what());
          }
        });
    image_ = node.create_subscription<sensor_msgs::msg::Image>(
        parameters_.image_topic,
        SensorQos(parameters_, parameters_.image_depth),
        [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
          ImageMeasurement image;
          const auto error =
              ConvertImage(*message, parameters_.image_time_offset, &image);
          if (error.empty())
            session_.AddImage(std::move(image));
          else
            Warn(error);
        });
    processing_ = node.create_wall_timer(std::chrono::milliseconds(1), [this] {
      session_.ProcessAvailable(
          [this](const auto& result, const auto& prepared, double elapsed) {
            publisher_.Publish(result, prepared);
            const char* stage = result.kind == UpdateKind::kVisual
                                    ? "VIO Time"
                                    : "LIO Mapping Time";
            std::printf("%s\n| Current Total Time | %.6f |\n", stage, elapsed);
          });
    });
    propagation_ = node.create_wall_timer(std::chrono::milliseconds(4), [this] {
      const auto result = session_.PropagateLatest();
      if (result)
        publisher_.PublishImuPropagation(result->state, result->timestamp);
    });
    RCLCPP_INFO(node.get_logger(), "livo_atlas frontend ready");
  }
  void Warn(const std::string& error) {
    RCLCPP_WARN(node_.get_logger(), "%s", error.c_str());
  }
  MapperNode& node_;
  NodeParameters parameters_;
  MappingSession session_;
  ResultPublisher publisher_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_;
  rclcpp::TimerBase::SharedPtr processing_, propagation_;
};
MapperNode::MapperNode(const rclcpp::NodeOptions& options)
    : Node("laserMapping", options), impl_(std::make_unique<Impl>(*this)) {}
MapperNode::~MapperNode() = default;
void MapperNode::Finish() { impl_->session_.Finish(); }
}  // namespace livo_atlas::ros2
