#include "livo_atlas/ros1/livo_node.h"

#include <image_transport/image_transport.h>

#include <cstdint>
#include <cstdio>
#include <exception>
#include <utility>

#include "livo_atlas/ros1/measurement_conversions.h"
#include "livo_atlas/ros1/livo_publisher.h"
namespace livo_atlas::ros1 {
class LivoNode::Impl {
 public:
  Impl(ros::NodeHandle& node, const NodeOptions& parameters,
       LivoSystem& system)
      : parameters_(parameters),
        system_(system),
        transport_(node),
        publisher_(parameters.livo.system.exporter.mode) {
    publisher_.Initialize(node, transport_);
    const auto& input = parameters_.livo;
    if (input.system.preprocess.lidar_type == kLivox) {
      lidar_ = node.subscribe<livox_ros_driver::CustomMsg>(
          input.lidar_topic, 200000,
          [this](const livox_ros_driver::CustomMsg::ConstPtr& message) {
            ++lidar_count_;
            LidarScan scan;
            const auto error = ConvertLivox(*message, &scan);
            if (error.empty())
              system_.AddLidar(scan);
            else
              Warn(error);
          });
    } else {
      lidar_ = node.subscribe<sensor_msgs::PointCloud2>(
          input.lidar_topic, 200000,
          [this](const sensor_msgs::PointCloud2::ConstPtr& message) {
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
    imu_ = node.subscribe<sensor_msgs::Imu>(
        input.imu_topic, 200000,
        [this](const sensor_msgs::Imu::ConstPtr& message) {
          ++imu_count_;
          try {
            system_.AddImu(ConvertImu(*message));
          } catch (const std::exception& error) {
            Warn(error.what());
          }
        });
    image_ = node.subscribe<sensor_msgs::Image>(
        input.image_topic, 200000,
        [this](const sensor_msgs::Image::ConstPtr& message) {
          ++image_count_;
          TimedImage image;
          const auto error = ConvertImage(
              *message, parameters_.livo.image_time_offset, &image);
          if (error.empty())
            system_.AddImage(std::move(image));
          else
            Warn(error);
        });
    propagation_ =
        node.createTimer(ros::Duration(.004), [this](const ros::TimerEvent&) {
          const auto result = system_.PropagateLatest();
          if (result) {
            ros::Time stamp;
            stamp.fromSec(result->timestamp);
            publisher_.PublishImuPropagation(result->state, stamp);
          }
        });
    ROS_INFO("livo_atlas frontend ready");
  }
  SynchronizationResult ProcessNext() {
    return system_.ProcessNext([this](const auto& result, const auto& prepared,
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
  void Warn(const std::string& error) { ROS_WARN_STREAM(error); }
  NodeOptions parameters_;
  LivoSystem& system_;
  image_transport::ImageTransport transport_;
  LivoPublisher publisher_;
  ros::Subscriber lidar_, imu_, image_;
  ros::Timer propagation_;
  std::uint64_t lidar_count_ = 0, imu_count_ = 0, image_count_ = 0;
};
LivoNode::LivoNode(ros::NodeHandle& node,
                               const NodeOptions& parameters,
                               LivoSystem& system)
    : impl_(std::make_unique<Impl>(node, parameters, system)) {}
LivoNode::~LivoNode() = default;
SynchronizationResult LivoNode::ProcessNext() {
  return impl_->ProcessNext();
}
void LivoNode::ReportInputs() { impl_->ReportInputs(); }
}  // namespace livo_atlas::ros1
