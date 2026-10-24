#ifndef LIVO_ATLAS_ROS1_MAPPING_ADAPTER_H_
#define LIVO_ATLAS_ROS1_MAPPING_ADAPTER_H_
#include <ros/ros.h>

#include <memory>

#include "livo_atlas/ros1/parameters.h"
#include "livo_atlas/runtime/mapping_session.h"
namespace livo_atlas::ros1 {
class MappingAdapter {
 public:
  MappingAdapter(ros::NodeHandle& node, const NodeParameters& parameters,
                 MappingSession& session);
  ~MappingAdapter();
  MappingAdapter(const MappingAdapter&) = delete;
  MappingAdapter& operator=(const MappingAdapter&) = delete;
  SynchronizationResult ProcessNext();
  void ReportInputs();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace livo_atlas::ros1
#endif  // LIVO_ATLAS_ROS1_MAPPING_ADAPTER_H_
