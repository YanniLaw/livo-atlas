#ifndef LIVO_ATLAS_ROS1_RESULT_PUBLISHER_H_
#define LIVO_ATLAS_ROS1_RESULT_PUBLISHER_H_

#include <memory>

#include <ros/time.h>

#include "livo_atlas/output/output_processor.h"

namespace ros {
class NodeHandle;
}  // namespace ros
namespace image_transport {
class ImageTransport;
}  // namespace image_transport

namespace livo_atlas {
namespace ros1 {

class ResultPublisher {
 public:
  explicit ResultPublisher(FrontendMode mode);
  ~ResultPublisher();
  ResultPublisher(const ResultPublisher&) = delete;
  ResultPublisher& operator=(const ResultPublisher&) = delete;

  void Initialize(ros::NodeHandle& node,
                  image_transport::ImageTransport& transport);
  void Publish(const FrontendResult& result, const PreparedOutput& prepared);
  void PublishImuPropagation(const State& state, const ros::Time& timestamp);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ros1
}  // namespace livo_atlas

#endif  // LIVO_ATLAS_ROS1_RESULT_PUBLISHER_H_
