#include <cstdio>
#include <exception>
#include <memory>

#include "livo_atlas/core/livo_system.h"
#if LIVO_ATLAS_ROS_VERSION == 1
#include "livo_atlas/ros1/livo_node.h"
namespace ros_integration = livo_atlas::ros1;
#elif LIVO_ATLAS_ROS_VERSION == 2
#include "livo_atlas/ros2/livo_node.h"
namespace ros_integration = livo_atlas::ros2;
#else
#error "The mapping node requires ROS1 or ROS2"
#endif
int main(int argc, char** argv) {
#if LIVO_ATLAS_ROS_VERSION == 1
  ros::init(argc, argv, "laserMapping");
  auto node = std::make_shared<ros::NodeHandle>();
#else
  rclcpp::init(argc, argv);
#endif
  int result = 0;
  try {
#if LIVO_ATLAS_ROS_VERSION == 2
    auto node = std::make_shared<rclcpp::Node>("laserMapping");
#endif
    const auto parameters = ros_integration::ReadNodeOptions(*node);
    livo_atlas::LivoSystem system(parameters.livo.system);
    ros_integration::LivoNode ros_node(*node, parameters, system);
#if LIVO_ATLAS_ROS_VERSION == 1
    ros::Rate rate(5000);
    while (ros::ok()) {
      ros::spinOnce();
      if (ros_node.ProcessNext() != livo_atlas::SynchronizationResult::kReady)
        rate.sleep();
    }
#else
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
#endif
    system.Finish();
    ros_node.ReportInputs();
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
