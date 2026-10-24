#include <cstdio>
#include <exception>
#include <memory>

#if LIVO_ATLAS_ROS_VERSION == 1
#include "LIVMapper.h"
#elif LIVO_ATLAS_ROS_VERSION == 2
#include "ros2/mapper_node.h"
#else
#error "The mapping node requires ROS1 or ROS2"
#endif

int main(int argc, char** argv) {
#if LIVO_ATLAS_ROS_VERSION == 1
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle node_handle;
  image_transport::ImageTransport image_transport(node_handle);
#else
  rclcpp::init(argc, argv);
#endif

  int result = 0;
  try {
#if LIVO_ATLAS_ROS_VERSION == 1
    LIVMapper mapper(node_handle);
    mapper.initializeSubscribersAndPublishers(node_handle, image_transport);
    mapper.run();
#else
    auto node = std::make_shared<livo_atlas::ros2::MapperNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    node->Finish();
#endif
  } catch (const std::exception& error) {
#if LIVO_ATLAS_ROS_VERSION == 1
    ROS_FATAL_STREAM(
        "LIVO initialization or processing failed: " << error.what());
#else
    std::fprintf(stderr, "livo_atlas: %s\n", error.what());
#endif
    result = 1;
  }
#if LIVO_ATLAS_ROS_VERSION == 2
  rclcpp::shutdown();
#endif
  return result;
}
