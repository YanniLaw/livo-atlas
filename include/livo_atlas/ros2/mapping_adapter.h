#ifndef LIVO_ATLAS_ROS2_MAPPING_ADAPTER_H_
#define LIVO_ATLAS_ROS2_MAPPING_ADAPTER_H_
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "livo_atlas/ros2/parameters.h"
#include "livo_atlas/runtime/mapping_session.h"
namespace livo_atlas::ros2 {
class MappingAdapter {
 public:
  MappingAdapter(rclcpp::Node& node, const NodeParameters& parameters,
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
}  // namespace livo_atlas::ros2
#endif  // LIVO_ATLAS_ROS2_MAPPING_ADAPTER_H_
