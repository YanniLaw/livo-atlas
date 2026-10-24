#include <cstdio>
#include <exception>

#include "ros2/mapper_node.h"
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<livo_atlas::ros2::MapperNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    node->Finish();
  } catch (const std::exception& error) {
    std::fprintf(stderr, "livo_atlas: %s\n", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
