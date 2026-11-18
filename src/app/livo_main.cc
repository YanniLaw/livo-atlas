#include <cstdint>
#include <cstdio>
#include <exception>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "livo_atlas/core/geometry_map_output.h"
#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/core/trajectory_output.h"
#if LIVO_ATLAS_ONLINE_GAUSSIAN
#include "livo_atlas/gaussian/gaussian_render_output.h"
#include "livo_atlas/gaussian/online_gaussian_mapper.h"
#endif
#if LIVO_ATLAS_ONLINE_BACKEND
#include "livo_atlas/core/loop_backend.h"
#endif
#if LIVO_ATLAS_ROS_VERSION == 1
#if LIVO_ATLAS_ONLINE_GAUSSIAN
#include "livo_atlas/ros1/gaussian_render_publisher.h"
#endif
#include "livo_atlas/ros1/geometry_map_publisher.h"
#include "livo_atlas/ros1/keyframe_trajectory_publisher.h"
#include "livo_atlas/ros1/livo_node.h"
namespace ros_integration = livo_atlas::ros1;
#elif LIVO_ATLAS_ROS_VERSION == 2
#if LIVO_ATLAS_ONLINE_GAUSSIAN
#include "livo_atlas/ros2/gaussian_render_publisher.h"
#endif
#include "livo_atlas/ros2/geometry_map_publisher.h"
#include "livo_atlas/ros2/keyframe_trajectory_publisher.h"
#include "livo_atlas/ros2/livo_node.h"
namespace ros_integration = livo_atlas::ros2;
#else
#error "The mapping node requires ROS1 or ROS2"
#endif

#if LIVO_ATLAS_ONLINE_GAUSSIAN
namespace {

void ReportJointRender(const livo_atlas::GaussianRender& r) {
  const auto& q = r.request;
  const auto identity = livo_atlas::GaussianRenderSourceIdentity(q);
  std::printf(
      "livo_atlas gaussian joint accepted: session=%s job=%llu "
      "map_sequence=%llu pose_version=%llu frame_count=%llu "
      "optimized_frame_count=%llu generation=%llu query=%llu owner=%llu "
      "blocks=%zu changed_pixels=%d accepted_at_steady_seconds=%.9f "
      "latency_seconds=%.9f worker_seconds=%.9f policy=%s "
      "selected_first=%llu selected_last=%llu candidate_count=%llu "
      "candidate_pixels=%llu projection_uncertain=%d selection_seconds=%.9f "
      "source_identity=%s\n",
      q.session_id.c_str(), static_cast<unsigned long long>(q.job_id),
      static_cast<unsigned long long>(q.map_sequence),
      static_cast<unsigned long long>(q.pose_version),
      static_cast<unsigned long long>(q.frame_count),
      static_cast<unsigned long long>(q.optimized_frame_count),
      static_cast<unsigned long long>(q.generation),
      static_cast<unsigned long long>(q.query_keyframe_id),
      static_cast<unsigned long long>(q.query_submap_id), q.blocks.size(),
      r.changed_pixels, r.accepted_at_steady_seconds, r.latency_seconds,
      r.worker_seconds,
      q.policy == livo_atlas::GaussianRenderPolicy::kSpatial ? "spatial"
                                                             : "temporal",
      static_cast<unsigned long long>(q.blocks.front().target.group.primary.id),
      static_cast<unsigned long long>(q.blocks.back().target.group.primary.id),
      static_cast<unsigned long long>(q.candidate_count),
      static_cast<unsigned long long>(q.candidate_pixels),
      static_cast<int>(q.projection_uncertain), q.selection_seconds,
      identity.c_str());
  std::fflush(stdout);
}

void PollGaussian(livo_atlas::OnlineGaussianMapper& mapper,
                  std::uint64_t* reported_job, std::uint64_t* reported_wait,
                  livo_atlas::GaussianRenderOutput* publication) {
  mapper.Poll();
  const auto stats = mapper.GetRenderStats();
  if (stats.wait_changes != *reported_wait) {
    std::printf(
        "livo_atlas gaussian joint waiting: reason=%s map_sequence=%llu "
        "changes=%llu\n",
        stats.wait_reason.c_str(),
        static_cast<unsigned long long>(stats.wait_map_sequence),
        static_cast<unsigned long long>(stats.wait_changes));
    *reported_wait = stats.wait_changes;
    std::fflush(stdout);
  }
  const auto render = mapper.GetLatestRender();
  if (render && render->request.job_id != *reported_job) {
    ReportJointRender(*render);
    *reported_job = render->request.job_id;
  }
  if (publication) publication->Observe(mapper.GetRenderSnapshot());
}

void ReportFinalization(const livo_atlas::GaussianFinalizationResult& r) {
  const char* status = "failed";
  if (r.status == livo_atlas::GaussianFinalizationStatus::kSucceeded)
    status = "succeeded";
  else if (r.status == livo_atlas::GaussianFinalizationStatus::kTimedOut)
    status = "timed_out";
  std::ostringstream line;
  line << std::setprecision(17)
       << "livo_atlas gaussian finalization: status=" << status
       << " session_id=" << r.session_id << " pose_version=" << r.pose_version
       << " frame_count=" << r.frame_count
       << " optimized_frame_count=" << r.optimized_frame_count
       << " parent_frame="
       << (r.parent_frame == livo_atlas::PoseFrame::kMap ? "map" : "odom")
       << " map_sequence=" << r.map_sequence
       << " target_valid=" << r.target_valid
       << " eligible_set_frozen=" << r.eligible_set_frozen;
  const auto ids = [&](const char* key, const auto& values) {
    line << ' ' << key << '=';
    if (values.empty()) line << '-';
    for (std::size_t i = 0; i < values.size(); ++i) {
      if (i) line << ',';
      line << values[i];
    }
  };
  ids("eligible_submap_ids", r.eligible_submap_ids);
  ids("asset_ids", r.asset_ids);
  line << " eligible=" << r.eligible_submap_ids.size()
       << " current=" << r.current << " needs_rebuild=" << r.needs_rebuild
       << " missing=" << r.missing << " dirty=" << r.dirty
       << " durable=" << r.durable << " worker_stopped=" << r.worker_stopped
       << " report_persisted=" << r.report_persisted
       << " repair_attempts=" << r.repair_attempts;
  const auto counts = [&](const char* stage, const auto& value) {
    line << ' ' << stage << "_accepted=" << value.accepted << ' ' << stage
         << "_effective_training=" << value.effective_training << ' ' << stage
         << "_rebuilds=" << value.rebuilds << ' ' << stage
         << "_checkpoints=" << value.checkpoints;
  };
  counts("before", r.before);
  counts("after", r.after);
  line << " elapsed_seconds=" << r.elapsed_seconds
       << " repair_seconds=" << r.repair_seconds
       << " shutdown_seconds=" << r.shutdown_seconds
       << " total_seconds=" << r.total_seconds;
  std::printf("%s\n", line.str().c_str());
  // Diagnostics are separate from the machine-readable identity/counter row.
  if (!r.reason.empty() || !r.report_error.empty())
    std::fprintf(stderr,
                 "Gaussian finalization reason: %.1024s; report: %.1024s\n",
                 r.reason.c_str(), r.report_error.c_str());
}

}  // namespace
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
    std::uint64_t reported_render_job = 0;
    std::uint64_t reported_render_wait = 0;
    if (parameters.livo.gaussian.enabled) {
      gaussian = std::make_unique<livo_atlas::OnlineGaussianMapper>(
          system, parameters.livo.system.frontend.calibration.camera,
          parameters.livo.gaussian);
    }
    // The publisher outlives the CPU output worker, including stack unwinding.
    std::unique_ptr<ros_integration::GaussianRenderPublisher>
        gaussian_publisher;
    std::unique_ptr<livo_atlas::GaussianRenderOutput> gaussian_publication;
    if (parameters.livo.gaussian_output.enabled) {
      gaussian_publisher =
          std::make_unique<ros_integration::GaussianRenderPublisher>(
              *node, parameters.livo.gaussian_output.optical_frame,
              parameters.livo.global_output.map_frame);
      gaussian_publication = std::make_unique<livo_atlas::GaussianRenderOutput>(
          parameters.livo.gaussian_output,
          [publisher = gaussian_publisher.get()](const auto& snapshot) {
            return publisher->Publish(snapshot);
          });
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
        if (gaussian)
          PollGaussian(*gaussian, &reported_render_job, &reported_render_wait,
                       gaussian_publication.get());
#endif
      };
    }
    if (trajectory) trajectory->Poll();
    if (geometry) geometry->Poll();
#if LIVO_ATLAS_ONLINE_GAUSSIAN
    if (gaussian)
      PollGaussian(*gaussian, &reported_render_job, &reported_render_wait,
                   gaussian_publication.get());
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
#if LIVO_ATLAS_ONLINE_GAUSSIAN
    // Withdraw derived output before terminal backend/Gaussian work can block.
    // A closed ROS context is counted as unavailable, never remote delivery.
    if (gaussian_publication) {
      gaussian_publication->Finish();
      const auto s = gaussian_publication->GetStats();
      std::printf(
          "livo_atlas gaussian publication: observed=%llu submitted=%llu "
          "coalesced=%llu published=%llu unavailable=%llu failed=%llu "
          "pending=%llu in_flight=%llu peak_pending=%llu peak_in_flight=%llu "
          "last_revision=%llu max_worker_seconds=%.9f finished=%d\n",
          static_cast<unsigned long long>(s.observed),
          static_cast<unsigned long long>(s.submitted),
          static_cast<unsigned long long>(s.coalesced),
          static_cast<unsigned long long>(s.published),
          static_cast<unsigned long long>(s.unavailable),
          static_cast<unsigned long long>(s.failed),
          static_cast<unsigned long long>(s.pending),
          static_cast<unsigned long long>(s.in_flight),
          static_cast<unsigned long long>(s.peak_pending),
          static_cast<unsigned long long>(s.peak_in_flight),
          static_cast<unsigned long long>(s.last_revision),
          s.max_worker_seconds, static_cast<int>(s.finished));
      std::fflush(stdout);
      if (!s.error.empty())
        std::fprintf(stderr, "Gaussian publication: %.64s\n", s.error.c_str());
    }
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
      if (parameters.livo.gaussian.finalize_on_shutdown) {
        const auto finalization =
            gaussian->Finalize(parameters.livo.gaussian.finalize_timeout);
        ReportFinalization(finalization);
        if (finalization.status !=
                livo_atlas::GaussianFinalizationStatus::kSucceeded ||
            !finalization.report_persisted)
          result = 1;
      } else {
        gaussian->Finish();
      }
      if (parameters.livo.gaussian.joint_render_enabled) {
        const auto r = gaussian->GetRenderStats();
        std::printf(
            "livo_atlas gaussian joint: submitted=%llu accepted=%llu "
            "dual_accepted=%llu stale=%llu coalesced=%llu skipped=%llu "
            "failed=%llu saved=%llu peak_pending=%llu peak_in_flight=%llu "
            "pending=%llu in_flight=%llu pending_storage=%llu "
            "disabled=%d max_latency_seconds=%.9f "
            "total_latency_seconds=%.9f\n",
            static_cast<unsigned long long>(r.submitted),
            static_cast<unsigned long long>(r.accepted),
            static_cast<unsigned long long>(r.dual_accepted),
            static_cast<unsigned long long>(r.stale),
            static_cast<unsigned long long>(r.coalesced),
            static_cast<unsigned long long>(r.skipped),
            static_cast<unsigned long long>(r.failed),
            static_cast<unsigned long long>(r.saved),
            static_cast<unsigned long long>(r.peak_pending),
            static_cast<unsigned long long>(r.peak_in_flight),
            static_cast<unsigned long long>(r.pending),
            static_cast<unsigned long long>(r.in_flight),
            static_cast<unsigned long long>(r.pending_storage),
            static_cast<int>(r.disabled), r.max_latency_seconds,
            r.total_latency_seconds);
        std::fflush(stdout);
        if (!r.error.empty())
          std::fprintf(stderr, "Joint render: %.4096s\n", r.error.c_str());
      }
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
            "needs_rebuild=%llu dirty=%llu asset_bytes=%llu incomplete=%d "
            "effective_training=%llu\n",
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
            static_cast<int>(s.incomplete_shutdown),
            static_cast<unsigned long long>(s.effective_training));
      }
      // Keep merged stdout/stderr logs parseable on an incomplete shutdown.
      std::fflush(stdout);
      if (s.incomplete_shutdown) {
        // Keep the last confirmed identities available in the node log even
        // when the latest asynchronous catalogue could not be committed.
        const auto map = gaussian->GetMapSnapshot();
        if (map) {
          for (const auto& entry : map->entries) {
            if (!entry.content) continue;
            const auto& content = *entry.content;
            std::fprintf(
                stderr,
                "livo_atlas gaussian confirmation: session=%s submap=%llu "
                "source_generation=%llu content_job_id=%llu "
                "binding_job_id=%llu "
                "asset_id=%llu volatile=%d catalog_pending=%d\n",
                content.session_id.c_str(),
                static_cast<unsigned long long>(content.submap_id),
                static_cast<unsigned long long>(content.source_generation),
                static_cast<unsigned long long>(content.content_job_id),
                static_cast<unsigned long long>(entry.binding_job_id),
                static_cast<unsigned long long>(
                    entry.asset ? entry.asset->asset_id : 0),
                static_cast<int>(entry.dirty),
                static_cast<int>(map->map_sequence > s.map_sequence));
          }
        }
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
