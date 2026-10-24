#include "livo_atlas/ros1/mapping_adapter.h"

#include <image_transport/image_transport.h>

#include <cstdint>
#include <cstdio>
#include <exception>
#include <utility>

#include "livo_atlas/ros1/measurement_conversions.h"
#include "livo_atlas/ros1/result_publisher.h"
namespace livo_atlas::ros1 {
class MappingAdapter::Impl {
 public:
  Impl(ros::NodeHandle& node, const NodeParameters& parameters,
       MappingSession& session)
      : parameters_(parameters),
        session_(session),
        transport_(node),
        publisher_(parameters.mapping.session.output.mode) {
    publisher_.Initialize(node, transport_);
    const auto& input = parameters_.mapping;
    if (input.session.preprocess.lidar_type == kLivox) {
      lidar_ = node.subscribe<livox_ros_driver::CustomMsg>(
          input.lidar_topic, 200000,
          [this](const livox_ros_driver::CustomMsg::ConstPtr& message) {
            ++lidar_count_;
            LidarScan scan;
            const auto error = ConvertLivox(*message, &scan);
            if (error.empty())
              session_.AddLidar(scan);
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
                *message, parameters_.mapping.session.preprocess.lidar_type,
                parameters_.mapping.lidar_time_offset, &scan);
            if (error.empty())
              session_.AddLidar(scan);
            else
              Warn(error);
          });
    }
    imu_ = node.subscribe<sensor_msgs::Imu>(
        input.imu_topic, 200000,
        [this](const sensor_msgs::Imu::ConstPtr& message) {
          ++imu_count_;
          try {
            session_.AddImu(ConvertImu(*message));
          } catch (const std::exception& error) {
            Warn(error.what());
          }
        });
    image_ = node.subscribe<sensor_msgs::Image>(
        input.image_topic, 200000,
        [this](const sensor_msgs::Image::ConstPtr& message) {
          ++image_count_;
          ImageMeasurement image;
          const auto error = ConvertImage(
              *message, parameters_.mapping.image_time_offset, &image);
          if (error.empty())
            session_.AddImage(std::move(image));
          else
            Warn(error);
        });
    propagation_ =
        node.createTimer(ros::Duration(.004), [this](const ros::TimerEvent&) {
          const auto result = session_.PropagateLatest();
          if (result) {
            ros::Time stamp;
            stamp.fromSec(result->timestamp);
            publisher_.PublishImuPropagation(result->state, stamp);
          }
        });
    ROS_INFO("livo_atlas frontend ready");
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
  void Warn(const std::string& error) { ROS_WARN_STREAM(error); }
  NodeParameters parameters_;
  MappingSession& session_;
  image_transport::ImageTransport transport_;
  ResultPublisher publisher_;
  ros::Subscriber lidar_, imu_, image_;
  ros::Timer propagation_;
  std::uint64_t lidar_count_ = 0, imu_count_ = 0, image_count_ = 0;
};
MappingAdapter::MappingAdapter(ros::NodeHandle& node,
                               const NodeParameters& parameters,
                               MappingSession& session)
    : impl_(std::make_unique<Impl>(node, parameters, session)) {}
MappingAdapter::~MappingAdapter() = default;
SynchronizationResult MappingAdapter::ProcessNext() {
  return impl_->ProcessNext();
}
void MappingAdapter::ReportInputs() { impl_->ReportInputs(); }
}  // namespace livo_atlas::ros1
