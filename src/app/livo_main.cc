#include <cstdint>
#include <exception>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "livo_atlas/common/logging.h"
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
  const std::string_view source_identity(identity);
  // Keep each fragment below glog's per-message limit.
  constexpr std::size_t kIdentityChunkSize = 16 * 1024;
  LOG(INFO) << std::fixed << std::setprecision(9)
            << "livo_atlas gaussian joint accepted: session=" << q.session_id
            << " job=" << q.job_id << " map_sequence=" << q.map_sequence
            << " pose_version=" << q.pose_version
            << " frame_count=" << q.frame_count
            << " optimized_frame_count=" << q.optimized_frame_count
            << " generation=" << q.generation
            << " query=" << q.query_keyframe_id
            << " owner=" << q.query_submap_id << " blocks=" << q.blocks.size()
            << " changed_pixels=" << r.changed_pixels
            << " accepted_at_steady_seconds=" << r.accepted_at_steady_seconds
            << " latency_seconds=" << r.latency_seconds
            << " worker_seconds=" << r.worker_seconds << " policy="
            << (q.policy == livo_atlas::GaussianRenderPolicy::kSpatial
                    ? "spatial"
                    : "temporal")
            << " selected_first=" << q.blocks.front().target.group.primary.id
            << " selected_last=" << q.blocks.back().target.group.primary.id
            << " candidate_count=" << q.candidate_count
            << " candidate_pixels=" << q.candidate_pixels
            << " projection_uncertain=" << q.projection_uncertain
            << " selection_seconds=" << q.selection_seconds
            << " source_identity_size=" << source_identity.size()
            << " source_identity="
            << source_identity.substr(0, kIdentityChunkSize);
  for (std::size_t offset = kIdentityChunkSize; offset < source_identity.size();
       offset += kIdentityChunkSize) {
    LOG(INFO) << "livo_atlas gaussian joint source: job=" << q.job_id
              << " offset=" << offset << " source_identity="
              << source_identity.substr(offset, kIdentityChunkSize);
  }
}

void PollGaussian(livo_atlas::OnlineGaussianMapper& mapper,
                  std::uint64_t* reported_job, std::uint64_t* reported_wait,
                  livo_atlas::GaussianRenderOutput* publication) {
  mapper.Poll();
  const auto stats = mapper.GetRenderStats();
  if (stats.wait_changes != *reported_wait) {
    LOG(INFO) << "livo_atlas gaussian joint waiting: reason="
              << stats.wait_reason
              << " map_sequence=" << stats.wait_map_sequence
              << " changes=" << stats.wait_changes;
    *reported_wait = stats.wait_changes;
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
  LOG(INFO) << line.str();
  // Diagnostics are separate from the machine-readable identity/counter row.
  if (!r.reason.empty() || !r.report_error.empty()) {
    LOG(WARNING) << "Gaussian finalization reason: " << r.reason.substr(0, 1024)
                 << "; report: " << r.report_error.substr(0, 1024);
  }
}

}  // namespace
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
      LOG(INFO) << std::fixed << std::setprecision(9)
                << "livo_atlas gaussian publication: observed=" << s.observed
                << " submitted=" << s.submitted << " coalesced=" << s.coalesced
                << " published=" << s.published
                << " unavailable=" << s.unavailable << " failed=" << s.failed
                << " pending=" << s.pending << " in_flight=" << s.in_flight
                << " peak_pending=" << s.peak_pending
                << " peak_in_flight=" << s.peak_in_flight
                << " last_revision=" << s.last_revision
                << " max_worker_seconds=" << s.max_worker_seconds
                << " finished=" << s.finished;
      if (!s.error.empty()) {
        LOG(WARNING) << "Gaussian publication: " << s.error.substr(0, 64);
      }
    }
#endif
#if LIVO_ATLAS_ONLINE_BACKEND
    if (backend) {
      backend->Finish();
      const auto s = backend->GetStats();
      LOG(INFO) << "livo_atlas loops: frames=" << s.frame_count
                << " pose_version=" << s.pose_version
                << " submitted=" << s.submitted << " completed=" << s.completed
                << " rejected=" << s.rejected << " accepted=" << s.accepted
                << " stale=" << s.stale << " skipped=" << s.skipped
                << " resource_limited=" << s.resource_limited
                << " peak_work_payloads=" << s.peak_work_payloads
                << " peak_pending=" << s.peak_pending;
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
        LOG(INFO) << std::fixed << std::setprecision(9)
                  << "livo_atlas gaussian joint: submitted=" << r.submitted
                  << " accepted=" << r.accepted
                  << " dual_accepted=" << r.dual_accepted
                  << " stale=" << r.stale
                  << " coalesced=" << r.coalesced << " skipped=" << r.skipped
                  << " failed=" << r.failed << " saved=" << r.saved
                  << " peak_pending=" << r.peak_pending
                  << " peak_in_flight=" << r.peak_in_flight
                  << " pending=" << r.pending << " in_flight=" << r.in_flight
                  << " pending_storage=" << r.pending_storage
                  << " disabled=" << r.disabled
                  << " max_latency_seconds=" << r.max_latency_seconds
                  << " total_latency_seconds=" << r.total_latency_seconds;
        if (!r.error.empty()) {
          LOG(WARNING) << "Joint render: " << r.error.substr(0, 4096);
        }
      }
      const auto s = gaussian->GetStats();
      LOG(INFO) << std::setprecision(9)
                << "livo_atlas gaussian: observed=" << s.observed
                << " submitted=" << s.submitted << " coalesced=" << s.coalesced
                << " no_image=" << s.skipped_no_image
                << " no_depth=" << s.skipped_no_depth
                << " resource_limited=" << s.resource_limited
                << " completed=" << s.completed << " accepted=" << s.accepted
                << " stale=" << s.stale << " resets=" << s.resets
                << " failed=" << s.failed << " pose_version=" << s.pose_version
                << " generation=" << s.generation
                << " peak_pending=" << s.peak_pending
                << " peak_in_flight=" << s.peak_in_flight
                << " max_observe_seconds=" << s.max_observe_seconds
                << " max_poll_seconds=" << s.max_poll_seconds
                << " last_latency_seconds=" << s.last_latency_seconds
                << " max_latency_seconds=" << s.max_latency_seconds
                << " mean_latency_seconds="
                << (s.accepted ? s.total_latency_seconds / s.accepted : 0.0);
      LOG(INFO) << "livo_atlas gaussian latency buckets: le_0p1="
                << s.latency_buckets[0]
                << " gt_0p1_le_0p25=" << s.latency_buckets[1]
                << " gt_0p25_le_0p5=" << s.latency_buckets[2]
                << " gt_0p5_le_1=" << s.latency_buckets[3]
                << " gt_1_le_2=" << s.latency_buckets[4]
                << " gt_2_le_5=" << s.latency_buckets[5]
                << " gt_5_le_10=" << s.latency_buckets[6]
                << " gt_10=" << s.latency_buckets[7];
      if (parameters.livo.gaussian.persist_submaps) {
        LOG(INFO) << "livo_atlas gaussian submaps: checkpoints="
                  << s.checkpoints
                  << " loads=" << s.loads << " rebinds=" << s.rebinds
                  << " rebuilds=" << s.rebuilds << " prunes=" << s.prunes
                  << " map_sequence=" << s.map_sequence
                  << " indexed=" << s.indexed << " missing=" << s.missing
                  << " needs_rebuild=" << s.needs_rebuild
                  << " dirty=" << s.dirty << " asset_bytes=" << s.asset_bytes
                  << " incomplete=" << s.incomplete_shutdown
                  << " effective_training=" << s.effective_training;
      }
      if (s.incomplete_shutdown) {
        // Keep the last confirmed identities available in the node log even
        // when the latest asynchronous catalogue could not be committed.
        const auto map = gaussian->GetMapSnapshot();
        if (map) {
          for (const auto& entry : map->entries) {
            if (!entry.content) continue;
            const auto& content = *entry.content;
            LOG(WARNING) << "livo_atlas gaussian confirmation: session="
                         << content.session_id
                         << " submap=" << content.submap_id
                         << " source_generation=" << content.source_generation
                         << " content_job_id=" << content.content_job_id
                         << " binding_job_id=" << entry.binding_job_id
                         << " asset_id="
                         << (entry.asset ? entry.asset->asset_id : 0)
                         << " volatile=" << entry.dirty
                         << " catalog_pending="
                         << (map->map_sequence > s.map_sequence);
          }
        }
      }
      if (!s.error.empty()) {
        LOG(WARNING) << "livo_atlas gaussian: " << s.error;
      }
    }
#endif
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
