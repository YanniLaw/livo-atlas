#ifndef LIVO_ATLAS_ROS2_RESULT_PUBLISHER_H_
#define LIVO_ATLAS_ROS2_RESULT_PUBLISHER_H_
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "livo_atlas/output/output_processor.h"
namespace livo_atlas::ros2 {
class ResultPublisher {
 public:
  ResultPublisher(rclcpp::Node& node, FrontendMode mode);
  ~ResultPublisher();
  ResultPublisher(const ResultPublisher&) = delete;
  ResultPublisher& operator=(const ResultPublisher&) = delete;
  void Publish(const FrontendResult& result, const PreparedOutput& prepared);
  void PublishImuPropagation(const State& state, double timestamp);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace livo_atlas::ros2
#endif  // LIVO_ATLAS_ROS2_RESULT_PUBLISHER_H_
