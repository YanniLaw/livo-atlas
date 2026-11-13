#include <cstdio>
#include <exception>
#include <functional>
#include <memory>
#include <stdexcept>
#include <utility>

#include "livo_atlas/core/livo_system.h"
#if LIVO_ATLAS_ONLINE_BACKEND
#include "livo_atlas/core/loop_backend.h"
#endif
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
#if LIVO_ATLAS_ONLINE_BACKEND
    livo_atlas::LoopAdmissionOptions loop_options;
    if (parameters.livo.loop_closure_enabled &&
        !parameters.livo.loop_closure_options_file.empty())
      loop_options = livo_atlas::ReadLoopAdmissionOptions(
          parameters.livo.loop_closure_options_file);
#else
    if (parameters.livo.loop_closure_enabled)
      throw std::invalid_argument(
          "Online loops require LIVO_ATLAS_WITH_GTSAM and "
          "LIVO_ATLAS_WITH_LOOP_CLOSURE");
#endif
    livo_atlas::LivoSystem system(parameters.livo.system);
    std::function<void()> after_process;
#if LIVO_ATLAS_ONLINE_BACKEND
    // Declaration order keeps callback sources before backend before system
    // during destruction, including exceptional shutdown.
    std::unique_ptr<livo_atlas::LoopBackend> backend;
    if (parameters.livo.loop_closure_enabled) {
      backend = std::make_unique<livo_atlas::LoopBackend>(system, loop_options);
      system.SetOnlineSubmapCallback(
          [&](const auto& update) { backend->Observe(update); });
      after_process = [&] { backend->Poll(); };
    }
#endif
    ros_integration::LivoNode ros_node(*node, parameters, system,
                                       std::move(after_process));
#if LIVO_ATLAS_ROS_VERSION == 1
    ros::Rate rate(5000);
    ros::WallRate backend_rate(5000);
    while (ros::ok()) {
      ros::spinOnce();
      if (ros_node.ProcessNext() != livo_atlas::SyncStatus::kReady) {
        // A paused simulation clock must not prevent taking worker results.
        if (parameters.livo.loop_closure_enabled) {
          backend_rate.sleep();
        } else {
          rate.sleep();
        }
      }
    }
#else
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
#endif
#if LIVO_ATLAS_ONLINE_BACKEND
    if (backend) {
      backend->Finish();
      const auto s = backend->GetStats();
      std::printf(
          "livo_atlas loops: frames=%llu pose_version=%llu submitted=%llu "
          "completed=%llu rejected=%llu accepted=%llu stale=%llu skipped=%llu "
          "resource_limited=%llu peak_work_payloads=%llu peak_pending=%llu\n",
          static_cast<unsigned long long>(s.frame_count),
          static_cast<unsigned long long>(s.pose_version),
          static_cast<unsigned long long>(s.submitted),
          static_cast<unsigned long long>(s.completed),
          static_cast<unsigned long long>(s.rejected),
          static_cast<unsigned long long>(s.accepted),
          static_cast<unsigned long long>(s.stale),
          static_cast<unsigned long long>(s.skipped),
          static_cast<unsigned long long>(s.resource_limited),
          static_cast<unsigned long long>(s.peak_work_payloads),
          static_cast<unsigned long long>(s.peak_pending));
    } else
#endif
    {
      system.Finish();
    }
    ros_node.ReportInputs();
  } catch (const std::exception& error) {
#if LIVO_ATLAS_ROS_VERSION == 1
    ROS_FATAL_STREAM(
        "LIVO initialization or processing failed: " << error.what());
#else
    std::fprintf(stderr, "livo_atlas: %s\n", error.what());
#endif
    result = 1;
  } catch (...) {
    std::fprintf(
        stderr, "livo_atlas: unknown initialization or processing exception\n");
    result = 1;
  }
#if LIVO_ATLAS_ROS_VERSION == 2
  rclcpp::shutdown();
#endif
  return result;
}
