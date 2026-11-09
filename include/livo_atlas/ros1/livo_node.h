#ifndef LIVO_ATLAS_ROS1_LIVO_NODE_H_
#define LIVO_ATLAS_ROS1_LIVO_NODE_H_

#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <cstdint>
#include <string>

#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/ros1/livo_publisher.h"
#include "livo_atlas/ros1/node_options.h"

namespace livo_atlas::ros1 {

class LivoNode {
 public:
  LivoNode(ros::NodeHandle& node, const NodeOptions& parameters,
           LivoSystem& system);
  ~LivoNode();
  LivoNode(const LivoNode&) = delete;
  LivoNode& operator=(const LivoNode&) = delete;
  SyncStatus ProcessNext();
  void ReportInputs();

 private:
  void Warn(const std::string& error);

  NodeOptions parameters_;
  LivoSystem& system_;
  image_transport::ImageTransport transport_;
  LivoPublisher publisher_;
  std::uint64_t lidar_count_ = 0, imu_count_ = 0, image_count_ = 0;
  // Destroy callback sources before the state they access.
  ros::Subscriber lidar_, imu_, image_;
  ros::Timer propagation_;
};

}  // namespace livo_atlas::ros1

#endif  // LIVO_ATLAS_ROS1_LIVO_NODE_H_
