#include <cstdio>
#include <exception>
#include <functional>
#include <memory>
#include <stdexcept>
#include <utility>

#include "livo_atlas/core/geometry_map_output.h"
#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/core/trajectory_output.h"
#if LIVO_ATLAS_ONLINE_GAUSSIAN
#include "livo_atlas/gaussian/online_gaussian_mapper.h"
#endif
#if LIVO_ATLAS_ONLINE_BACKEND
#include "livo_atlas/core/loop_backend.h"
#endif
#if LIVO_ATLAS_ROS_VERSION == 1
#include "livo_atlas/ros1/geometry_map_publisher.h"
#include "livo_atlas/ros1/keyframe_trajectory_publisher.h"
#include "livo_atlas/ros1/livo_node.h"
namespace ros_integration = livo_atlas::ros1;
#elif LIVO_ATLAS_ROS_VERSION == 2
#include "livo_atlas/ros2/geometry_map_publisher.h"
#include "livo_atlas/ros2/keyframe_trajectory_publisher.h"
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
    if (parameters.livo.gaussian.enabled) {
#if LIVO_ATLAS_ONLINE_GAUSSIAN
      livo_atlas::ValidateOnlineGaussianPaths(
          parameters.livo.gaussian,
          parameters.livo.system.exporter.directory / "keyframes");
#else
      throw std::invalid_argument(
          "Online Gaussian requires LIVO_ATLAS_WITH_ONLINE_GAUSSIAN");
#endif
    }
    livo_atlas::LivoSystem system(parameters.livo.system);
    std::function<void()> after_process;
#if LIVO_ATLAS_ONLINE_BACKEND
    // Declaration order keeps callback sources before backend before system
    // during destruction, including exceptional shutdown.
    std::unique_ptr<livo_atlas::LoopBackend> backend;
    if (parameters.livo.loop_closure_enabled) {
      backend = std::make_unique<livo_atlas::LoopBackend>(system, loop_options);
    }
#endif
#if LIVO_ATLAS_ONLINE_GAUSSIAN
    std::unique_ptr<livo_atlas::OnlineGaussianMapper> gaussian;
    if (parameters.livo.gaussian.enabled) {
      gaussian = std::make_unique<livo_atlas::OnlineGaussianMapper>(
          system, parameters.livo.system.frontend.calibration.camera,
          parameters.livo.gaussian);
    }
#endif
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
    if (parameters.livo.loop_closure_enabled || trajectory || geometry ||
        parameters.livo.gaussian.enabled) {
      system.SetOnlineSubmapCallback([&](const auto& update) {
#if LIVO_ATLAS_ONLINE_BACKEND
        if (backend) backend->Observe(update);
#endif
        if (trajectory) trajectory->Observe(update);
        if (geometry) geometry->Observe(update);
#if LIVO_ATLAS_ONLINE_GAUSSIAN
        if (gaussian) gaussian->Observe(update);
#endif
      });
      after_process = [&] {
#if LIVO_ATLAS_ONLINE_BACKEND
        if (backend) backend->Poll();
#endif
        if (trajectory) trajectory->Poll();
        if (geometry) geometry->Poll();
#if LIVO_ATLAS_ONLINE_GAUSSIAN
        if (gaussian) gaussian->Poll();
#endif
      };
    }
    if (trajectory) trajectory->Poll();
    if (geometry) geometry->Poll();
#if LIVO_ATLAS_ONLINE_GAUSSIAN
    if (gaussian) gaussian->Poll();
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
        if (parameters.livo.loop_closure_enabled || trajectory || geometry ||
            parameters.livo.gaussian.enabled) {
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
#if LIVO_ATLAS_ONLINE_GAUSSIAN
    if (gaussian) {
      gaussian->Finish();
      const auto s = gaussian->GetStats();
      std::printf(
          "livo_atlas gaussian: observed=%llu submitted=%llu coalesced=%llu "
          "no_image=%llu no_depth=%llu resource_limited=%llu completed=%llu "
          "accepted=%llu stale=%llu resets=%llu failed=%llu pose_version=%llu "
          "generation=%llu peak_pending=%llu peak_in_flight=%llu "
          "max_observe_seconds=%.9g max_poll_seconds=%.9g "
          "last_latency_seconds=%.9g max_latency_seconds=%.9g "
          "mean_latency_seconds=%.9g\n",
          static_cast<unsigned long long>(s.observed),
          static_cast<unsigned long long>(s.submitted),
          static_cast<unsigned long long>(s.coalesced),
          static_cast<unsigned long long>(s.skipped_no_image),
          static_cast<unsigned long long>(s.skipped_no_depth),
          static_cast<unsigned long long>(s.resource_limited),
          static_cast<unsigned long long>(s.completed),
          static_cast<unsigned long long>(s.accepted),
          static_cast<unsigned long long>(s.stale),
          static_cast<unsigned long long>(s.resets),
          static_cast<unsigned long long>(s.failed),
          static_cast<unsigned long long>(s.pose_version),
          static_cast<unsigned long long>(s.generation),
          static_cast<unsigned long long>(s.peak_pending),
          static_cast<unsigned long long>(s.peak_in_flight),
          s.max_observe_seconds, s.max_poll_seconds, s.last_latency_seconds,
          s.max_latency_seconds,
          s.accepted ? s.total_latency_seconds / s.accepted : 0.0);
      std::printf(
          "livo_atlas gaussian latency buckets: le_0p1=%llu "
          "gt_0p1_le_0p25=%llu gt_0p25_le_0p5=%llu gt_0p5_le_1=%llu "
          "gt_1_le_2=%llu gt_2_le_5=%llu gt_5_le_10=%llu gt_10=%llu\n",
          static_cast<unsigned long long>(s.latency_buckets[0]),
          static_cast<unsigned long long>(s.latency_buckets[1]),
          static_cast<unsigned long long>(s.latency_buckets[2]),
          static_cast<unsigned long long>(s.latency_buckets[3]),
          static_cast<unsigned long long>(s.latency_buckets[4]),
          static_cast<unsigned long long>(s.latency_buckets[5]),
          static_cast<unsigned long long>(s.latency_buckets[6]),
          static_cast<unsigned long long>(s.latency_buckets[7]));
      if (parameters.livo.gaussian.persist_submaps) {
        std::printf(
            "livo_atlas gaussian submaps: checkpoints=%llu loads=%llu "
            "rebinds=%llu "
            "rebuilds=%llu prunes=%llu map_sequence=%llu indexed=%llu "
            "missing=%llu "
            "needs_rebuild=%llu dirty=%llu asset_bytes=%llu incomplete=%d\n",
            static_cast<unsigned long long>(s.checkpoints),
            static_cast<unsigned long long>(s.loads),
            static_cast<unsigned long long>(s.rebinds),
            static_cast<unsigned long long>(s.rebuilds),
            static_cast<unsigned long long>(s.prunes),
            static_cast<unsigned long long>(s.map_sequence),
            static_cast<unsigned long long>(s.indexed),
            static_cast<unsigned long long>(s.missing),
            static_cast<unsigned long long>(s.needs_rebuild),
            static_cast<unsigned long long>(s.dirty),
            static_cast<unsigned long long>(s.asset_bytes),
            static_cast<int>(s.incomplete_shutdown));
      }
      if (!s.error.empty())
        std::fprintf(stderr, "livo_atlas gaussian: %s\n", s.error.c_str());
    }
#endif
    if (trajectory) {
      trajectory->Finish();
      const auto s = trajectory->GetStats();
      // Delivery means the SDK consumer returned, including after ROS stopped.
      std::printf(
          "livo_atlas trajectory: observed=%llu submitted=%llu coalesced=%llu "
          "delivered=%llu resource_limited=%llu failed=%llu revision=%llu "
          "pose_version=%llu frames=%llu peak_work_payloads=%llu "
          "peak_pending=%llu "
          "timestamp_reads=%llu max_observe_seconds=%.9g max_poll_seconds=%.9g "
          "max_worker_seconds=%.9g last_latency_seconds=%.9g\n",
          static_cast<unsigned long long>(s.observed),
          static_cast<unsigned long long>(s.submitted),
          static_cast<unsigned long long>(s.coalesced),
          static_cast<unsigned long long>(s.delivered),
          static_cast<unsigned long long>(s.resource_limited),
          static_cast<unsigned long long>(s.failed),
          static_cast<unsigned long long>(s.last_revision),
          static_cast<unsigned long long>(s.last_pose_version),
          static_cast<unsigned long long>(s.last_frame_count),
          static_cast<unsigned long long>(s.peak_work_payloads),
          static_cast<unsigned long long>(s.peak_pending),
          static_cast<unsigned long long>(s.timestamp_reads),
          s.max_observe_seconds, s.max_poll_seconds, s.max_worker_seconds,
          s.last_latency_seconds);
    }
    if (geometry) {
      geometry->Finish();
      const auto s = geometry->GetStats();
      // Delivery means the consumer returned; ROS may already be shut down.
      std::printf(
          "livo_atlas geometry map: observed=%llu submitted=%llu "
          "coalesced=%llu delivered=%llu resource_limited=%llu failed=%llu "
          "revision=%llu pose_version=%llu frames=%llu optimized_frames=%llu "
          "submaps=%llu source_voxels=%llu points=%llu peak_work_payloads=%llu "
          "peak_pending=%llu cache_hits=%llu cache_evictions=%llu "
          "cache_invalidations=%llu peak_cached_points=%llu full_builds=%llu "
          "cloud_reads=%llu timestamp_reads=%llu max_observe_seconds=%.9g "
          "max_poll_seconds=%.9g max_worker_seconds=%.9g "
          "last_latency_seconds=%.9g\n",
          static_cast<unsigned long long>(s.observed),
          static_cast<unsigned long long>(s.submitted),
          static_cast<unsigned long long>(s.coalesced),
          static_cast<unsigned long long>(s.delivered),
          static_cast<unsigned long long>(s.resource_limited),
          static_cast<unsigned long long>(s.failed),
          static_cast<unsigned long long>(s.last_revision),
          static_cast<unsigned long long>(s.last_pose_version),
          static_cast<unsigned long long>(s.last_frame_count),
          static_cast<unsigned long long>(s.last_optimized_frame_count),
          static_cast<unsigned long long>(s.last_submap_count),
          static_cast<unsigned long long>(s.last_source_voxel_count),
          static_cast<unsigned long long>(s.last_point_count),
          static_cast<unsigned long long>(s.peak_work_payloads),
          static_cast<unsigned long long>(s.peak_pending),
          static_cast<unsigned long long>(s.geometry.cache_hits),
          static_cast<unsigned long long>(s.geometry.cache_evictions),
          static_cast<unsigned long long>(s.geometry.cache_invalidations),
          static_cast<unsigned long long>(s.geometry.peak_cached_points),
          static_cast<unsigned long long>(s.geometry.geometry.full_builds),
          static_cast<unsigned long long>(s.geometry.geometry.cloud_reads),
          static_cast<unsigned long long>(s.timestamp_reads),
          s.max_observe_seconds, s.max_poll_seconds, s.max_worker_seconds,
          s.last_latency_seconds);
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
