#ifndef LIVO_ATLAS_ROS1_LIVO_NODE_H_
#define LIVO_ATLAS_ROS1_LIVO_NODE_H_

#include <atomic>
#include <chrono>
#include <functional>

#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/ros1/livo_publisher.h"
#include "livo_atlas/ros1/node_options.h"
#include "livo_atlas/ros_common/sensor_input.h"

namespace livo_atlas::ros1 {

class LivoNode {
 public:
  // The main loop owns the system while the spinner receives ROS messages.
  LivoNode(ros::NodeHandle& node, const NodeOptions& parameters,
           LivoSystem& system,
           std::function<void()> poll_background_outputs = {});
  ~LivoNode();
  LivoNode(const LivoNode&) = delete;
  LivoNode& operator=(const LivoNode&) = delete;
  SyncStatus ProcessNext();
  void RethrowCallbackFailure();
  // Call once the spinner has stopped.
  void DrainInput();
  void ReportInputs();

 private:
  NodeOptions parameters_;
  LivoSystem& system_;
  std::function<void()> poll_background_outputs_;
  image_transport::ImageTransport transport_;
  LivoPublisher publisher_;
  ros_detail::SensorInput input_;
  std::atomic<bool> propagation_due_{false};
  std::chrono::steady_clock::time_point last_propagation_;
  // Destroy callback sources before the state they access.
  ros::Subscriber lidar_, imu_, image_;
  ros::Timer propagation_;
};

}  // namespace livo_atlas::ros1

#endif  // LIVO_ATLAS_ROS1_LIVO_NODE_H_
