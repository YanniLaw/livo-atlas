#ifndef LIVO_ATLAS_ROS2_LIVO_PUBLISHER_H_
#define LIVO_ATLAS_ROS2_LIVO_PUBLISHER_H_
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "livo_atlas/common/filter_state.h"
#include "livo_atlas/frontend/frontend_result.h"
#include "livo_atlas/sensor/point_types.h"
namespace livo_atlas::ros2 {
class LivoPublisher {
 public:
  LivoPublisher(rclcpp::Node& node, FrontendMode mode);
  ~LivoPublisher();
  LivoPublisher(const LivoPublisher&) = delete;
  LivoPublisher& operator=(const LivoPublisher&) = delete;
  void Publish(const FrontendResult& result,
               const ColoredCloud::ConstPtr& colored_cloud);
  void PublishImuPropagation(const FilterState& state, double timestamp);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace livo_atlas::ros2
#endif  // LIVO_ATLAS_ROS2_LIVO_PUBLISHER_H_
