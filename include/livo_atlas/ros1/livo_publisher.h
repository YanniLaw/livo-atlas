#ifndef LIVO_ATLAS_ROS1_LIVO_PUBLISHER_H_
#define LIVO_ATLAS_ROS1_LIVO_PUBLISHER_H_

#include <image_transport/publisher.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <vector>

#include "livo_atlas/common/filter_state.h"
#include "livo_atlas/frontend/frontend_result.h"
#include "livo_atlas/frontend/synchronized_measurements.h"
#include "livo_atlas/sensor/point_types.h"

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
  explicit LivoPublisher(FrontendMode mode, int image_queue_size = 1);
  ~LivoPublisher();
  LivoPublisher(const LivoPublisher&) = delete;
  LivoPublisher& operator=(const LivoPublisher&) = delete;

  void Initialize(ros::NodeHandle& node,
                  image_transport::ImageTransport& transport);
  void Publish(const FrontendResult& result,
               const ColoredCloud::ConstPtr& colored_cloud);
  void PublishImuPropagation(const FilterState& state,
                             const ros::Time& timestamp);

 private:
  void PublishCloud(const FrontendResult& result,
                    const ColoredCloud::ConstPtr& colored_cloud);

  FrontendMode mode_;
  int image_queue_size_;
  ros::Publisher cloud_, effect_, odometry_, path_publisher_, mavros_;
  ros::Publisher propagation_, planes_;
  image_transport::Publisher image_;
  std::vector<ros::Publisher> compatibility_;
  tf::TransformBroadcaster broadcaster_;
  nav_msgs::Path path_;
};

}  // namespace ros1
}  // namespace livo_atlas

#endif  // LIVO_ATLAS_ROS1_LIVO_PUBLISHER_H_
