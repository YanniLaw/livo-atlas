#ifndef LIVO_ATLAS_ROS2_LIVO_NODE_H_
#define LIVO_ATLAS_ROS2_LIVO_NODE_H_
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/ros2/node_options.h"
namespace livo_atlas::ros2 {
class LivoNode {
 public:
  LivoNode(rclcpp::Node& node, const NodeOptions& parameters,
           LivoSystem& system);
  ~LivoNode();
  LivoNode(const LivoNode&) = delete;
  LivoNode& operator=(const LivoNode&) = delete;
  SyncStatus ProcessNext();
  void ReportInputs();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace livo_atlas::ros2
#endif  // LIVO_ATLAS_ROS2_LIVO_NODE_H_
