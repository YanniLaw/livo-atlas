#ifndef LIVO_ATLAS_ROS1_LIVO_NODE_H_
#define LIVO_ATLAS_ROS1_LIVO_NODE_H_
#include <ros/ros.h>

#include <memory>

#include "livo_atlas/core/livo_system.h"
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
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace livo_atlas::ros1
#endif  // LIVO_ATLAS_ROS1_LIVO_NODE_H_
