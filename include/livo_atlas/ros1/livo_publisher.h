#ifndef LIVO_ATLAS_ROS1_LIVO_PUBLISHER_H_
#define LIVO_ATLAS_ROS1_LIVO_PUBLISHER_H_

#include <ros/time.h>

#include <memory>

#include "livo_atlas/output/output_processor.h"

namespace ros {
class NodeHandle;
}  // namespace ros
namespace image_transport {
class ImageTransport;
}  // namespace image_transport

namespace livo_atlas {
namespace ros1 {

class LivoPublisher {
 public:
  explicit LivoPublisher(FrontendMode mode);
  ~LivoPublisher();
  LivoPublisher(const LivoPublisher&) = delete;
  LivoPublisher& operator=(const LivoPublisher&) = delete;

  void Initialize(ros::NodeHandle& node,
                  image_transport::ImageTransport& transport);
  void Publish(const FrontendResult& result, const PreparedOutput& prepared);
  void PublishImuPropagation(const FilterState& state,
                             const ros::Time& timestamp);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ros1
}  // namespace livo_atlas

#endif  // LIVO_ATLAS_ROS1_LIVO_PUBLISHER_H_
