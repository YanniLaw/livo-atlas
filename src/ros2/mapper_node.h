#ifndef LIVO_ATLAS_ROS2_MAPPER_NODE_H_
#define LIVO_ATLAS_ROS2_MAPPER_NODE_H_
#include <memory>
#include <rclcpp/rclcpp.hpp>
namespace livo_atlas::ros2 {
class MapperNode : public rclcpp::Node {
 public:
  explicit MapperNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~MapperNode() override;
  void Finish();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace livo_atlas::ros2
#endif  // LIVO_ATLAS_ROS2_MAPPER_NODE_H_
