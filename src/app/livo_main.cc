#include <exception>
#include <functional>
#include <iomanip>
#include <memory>
#include <utility>

#include "livo_atlas/common/logging.h"
#include "livo_atlas/core/geometry_map_output.h"
#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/core/trajectory_output.h"
#include "online_gaussian.h"
#if LIVO_ATLAS_ROS_VERSION == 1
#include <ros/spinner.h>

#include "livo_atlas/ros1/geometry_map_publisher.h"
#include "livo_atlas/ros1/keyframe_trajectory_publisher.h"
#include "livo_atlas/ros1/livo_node.h"
namespace ros_integration = livo_atlas::ros1;
#elif LIVO_ATLAS_ROS_VERSION == 2
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "livo_atlas/ros2/geometry_map_publisher.h"
#include "livo_atlas/ros2/keyframe_trajectory_publisher.h"
#include "livo_atlas/ros2/livo_node.h"
namespace ros_integration = livo_atlas::ros2;
#else
#error "The mapping node requires ROS1 or ROS2"
#endif

int main(int argc, char** argv) {
  livo_atlas::InitializeLogging(argv[0]);
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
    livo_atlas::app::OnlineGaussian::ValidateConfig(parameters.livo);
    livo_atlas::LivoSystem system(parameters.livo.system);
    std::function<void()> poll_background_outputs;
    livo_atlas::app::OnlineGaussian gaussian(*node, system, parameters.livo);
    // The worker joins before its captured publisher handles are destroyed.
    std::unique_ptr<ros_integration::KeyframeTrajectoryPublisher>
        trajectory_publisher;
    std::unique_ptr<livo_atlas::TrajectoryOutput> trajectory;
    if (parameters.livo.global_output.trajectory.enabled) {
      trajectory_publisher =
          std::make_unique<ros_integration::KeyframeTrajectoryPublisher>(
              *node, parameters.livo.global_output.map_frame);
      trajectory = std::make_unique<livo_atlas::TrajectoryOutput>(
          system, parameters.livo.global_output.trajectory,
          [publisher = trajectory_publisher.get()](const auto& snapshot) {
            publisher->Publish(snapshot);
          });
    }
    std::unique_ptr<ros_integration::GeometryMapPublisher> geometry_publisher;
    std::unique_ptr<livo_atlas::GeometryMapOutput> geometry;
    if (parameters.livo.global_output.geometry.enabled) {
      geometry_publisher =
          std::make_unique<ros_integration::GeometryMapPublisher>(
              *node, parameters.livo.global_output.map_frame);
      geometry = std::make_unique<livo_atlas::GeometryMapOutput>(
          system, parameters.livo.global_output.geometry,
          parameters.livo.system.geometry,
          [publisher = geometry_publisher.get()](const auto& snapshot) {
            publisher->Publish(snapshot);
          });
    }
    if (trajectory || geometry || parameters.livo.gaussian.enabled) {
      system.SetOnlineSubmapCallback([&](const auto& update) {
        if (trajectory) trajectory->Observe(update);
        if (geometry) geometry->Observe(update);
        gaussian.Observe(update);
      });
      // Outputs can have pending work while sensor synchronization is waiting.
      poll_background_outputs = [&] {
        if (trajectory) trajectory->Poll();
        if (geometry) geometry->Poll();
        gaussian.Poll();
      };
    }
    if (trajectory) trajectory->Poll();
    if (geometry) geometry->Poll();
    gaussian.Poll();
    ros_integration::LivoNode ros_node(*node, parameters, system,
                                       std::move(poll_background_outputs));
#if LIVO_ATLAS_ROS_VERSION == 1
    ros::AsyncSpinner spinner(parameters.livo.executor_threads);
    spinner.start();
    ros::WallRate rate(5000);
    while (ros::ok()) {
      if (ros_node.ProcessNext() != livo_atlas::SyncStatus::kReady) {
        rate.sleep();
      }
    }
    spinner.stop();
#else
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), parameters.livo.executor_threads);
    executor.add_node(node);
    executor.spin();
#endif
    ros_node.RethrowCallbackFailure();
    // Withdraw preview output before terminal backend work can block.
    gaussian.FinishPublication();
    system.Finish();
    if (const auto stats = system.GetLoopStats()) {
      const auto& s = *stats;
      LOG(INFO) << "livo_atlas loops: frames=" << s.frame_count
                << " pose_version=" << s.pose_version
                << " submitted=" << s.submitted << " completed=" << s.completed
                << " rejected=" << s.rejected << " accepted=" << s.accepted
                << " stale=" << s.stale << " skipped=" << s.skipped
                << " resource_limited=" << s.resource_limited
                << " peak_work_payloads=" << s.peak_work_payloads
                << " peak_pending=" << s.peak_pending;
      if (!parameters.livo.system.loop_closure->pose_graph_enabled) {
        LOG(INFO) << "livo_atlas loop detection: detected=" << s.detected;
      }
    }
    if (!gaussian.Finish(parameters.livo.gaussian)) result = 1;
    if (trajectory) {
      trajectory->Finish();
      const auto s = trajectory->GetStats();
      // Delivery means the SDK consumer returned, including after ROS stopped.
      LOG(INFO) << std::setprecision(9)
                << "livo_atlas trajectory: observed=" << s.observed
                << " submitted=" << s.submitted << " coalesced=" << s.coalesced
                << " delivered=" << s.delivered
                << " resource_limited=" << s.resource_limited
                << " failed=" << s.failed << " revision=" << s.last_revision
                << " pose_version=" << s.last_pose_version
                << " frames=" << s.last_frame_count
                << " peak_work_payloads=" << s.peak_work_payloads
                << " peak_pending=" << s.peak_pending
                << " timestamp_reads=" << s.timestamp_reads
                << " max_observe_seconds=" << s.max_observe_seconds
                << " max_poll_seconds=" << s.max_poll_seconds
                << " max_worker_seconds=" << s.max_worker_seconds
                << " last_latency_seconds=" << s.last_latency_seconds;
    }
    if (geometry) {
      geometry->Finish();
      const auto s = geometry->GetStats();
      // Delivery means the consumer returned; ROS may already be shut down.
      LOG(INFO) << std::setprecision(9)
                << "livo_atlas geometry map: observed=" << s.observed
                << " submitted=" << s.submitted << " coalesced=" << s.coalesced
                << " delivered=" << s.delivered
                << " resource_limited=" << s.resource_limited
                << " failed=" << s.failed << " revision=" << s.last_revision
                << " pose_version=" << s.last_pose_version
                << " frames=" << s.last_frame_count
                << " optimized_frames=" << s.last_optimized_frame_count
                << " submaps=" << s.last_submap_count
                << " source_voxels=" << s.last_source_voxel_count
                << " points=" << s.last_point_count
                << " peak_work_payloads=" << s.peak_work_payloads
                << " peak_pending=" << s.peak_pending
                << " cache_hits=" << s.geometry.cache_hits
                << " cache_evictions=" << s.geometry.cache_evictions
                << " cache_invalidations=" << s.geometry.cache_invalidations
                << " peak_cached_points=" << s.geometry.peak_cached_points
                << " full_builds=" << s.geometry.geometry.full_builds
                << " cloud_reads=" << s.geometry.geometry.cloud_reads
                << " timestamp_reads=" << s.timestamp_reads
                << " max_observe_seconds=" << s.max_observe_seconds
                << " max_poll_seconds=" << s.max_poll_seconds
                << " max_worker_seconds=" << s.max_worker_seconds
                << " last_latency_seconds=" << s.last_latency_seconds;
    }
    ros_node.ReportInputs();
  } catch (const std::exception& error) {
    LOG(ERROR) << "LIVO initialization or processing failed: " << error.what();
    result = 1;
  } catch (...) {
    LOG(ERROR) << "LIVO initialization or processing failed: unknown exception";
    result = 1;
  }
#if LIVO_ATLAS_ROS_VERSION == 2
  rclcpp::shutdown();
#endif
  return result;
}
