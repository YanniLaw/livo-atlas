#ifndef LIVO_ATLAS_ROS1_LIVO_NODE_H_
#define LIVO_ATLAS_ROS1_LIVO_NODE_H_

#include <atomic>
#include <cstdint>
#include <exception>
#include <functional>
#include <mutex>

#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/ros1/livo_publisher.h"
#include "livo_atlas/ros1/node_options.h"

namespace livo_atlas::ros1 {

class LivoNode {
 public:
  // Stop the spinner before using system directly.
  LivoNode(ros::NodeHandle& node, const NodeOptions& parameters,
           LivoSystem& system, std::function<void()> after_process = {});
  ~LivoNode();
  LivoNode(const LivoNode&) = delete;
  LivoNode& operator=(const LivoNode&) = delete;
  SyncStatus ProcessNext();
  void RethrowCallbackFailure();
  void ReportInputs();

 private:
  template <typename Callback>
  void RunCallback(Callback callback);

  NodeOptions parameters_;
  LivoSystem& system_;
  std::function<void()> after_process_;
  image_transport::ImageTransport transport_;
  LivoPublisher publisher_;
  // Serialize access to system_ and its processing hook across ROS callbacks.
  std::mutex system_mutex_;
  std::exception_ptr callback_failure_;
  std::atomic<std::uint64_t> lidar_count_{0};
  std::atomic<std::uint64_t> imu_count_{0};
  std::atomic<std::uint64_t> image_count_{0};
  // Destroy callback sources before the state they access.
  ros::Subscriber lidar_, imu_, image_;
  ros::Timer propagation_;
};

}  // namespace livo_atlas::ros1

#endif  // LIVO_ATLAS_ROS1_LIVO_NODE_H_
