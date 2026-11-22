#include "online_gaussian.h"

#include <stdexcept>

#include "livo_atlas/core/livo_config.h"

#if LIVO_ATLAS_ONLINE_GAUSSIAN
#include <cstddef>
#include <iomanip>
#include <sstream>
#include <string_view>

#include "livo_atlas/common/logging.h"
#include "livo_atlas/gaussian/gaussian_render_output.h"
#include "livo_atlas/gaussian/online_gaussian_mapper.h"
#if LIVO_ATLAS_ROS_VERSION == 1
#include "livo_atlas/ros1/gaussian_render_publisher.h"
#elif LIVO_ATLAS_ROS_VERSION == 2
#include "livo_atlas/ros2/gaussian_render_publisher.h"
#endif
#endif

namespace livo_atlas::app {

#if LIVO_ATLAS_ONLINE_GAUSSIAN
namespace {

void ReportJointRender(const GaussianRender& r) {
  const auto& q = r.request;
  const auto identity = GaussianRenderSourceIdentity(q);
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
            << (q.policy == GaussianRenderPolicy::kSpatial
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

void ReportFinalization(const GaussianFinalizationResult& r) {
  const char* status = "failed";
  if (r.status == GaussianFinalizationStatus::kSucceeded)
    status = "succeeded";
  else if (r.status == GaussianFinalizationStatus::kTimedOut)
    status = "timed_out";
  std::ostringstream line;
  line << std::setprecision(17)
       << "livo_atlas gaussian finalization: status=" << status
       << " session_id=" << r.session_id << " pose_version=" << r.pose_version
       << " frame_count=" << r.frame_count
       << " optimized_frame_count=" << r.optimized_frame_count
       << " parent_frame="
       << (r.parent_frame == PoseFrame::kMap ? "map" : "odom")
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

void OnlineGaussian::ValidateConfig(const LivoConfig& config) {
  if (config.gaussian.enabled) {
    ValidateOnlineGaussianPaths(config.gaussian,
                                config.system.exporter.directory / "keyframes");
  }
}

OnlineGaussian::OnlineGaussian(RosNode& node, LivoSystem& system,
                               const LivoConfig& config) {
  if (!config.gaussian.enabled) return;

  mapper_ = std::make_unique<OnlineGaussianMapper>(
      system, config.system.frontend.calibration.camera, config.gaussian);
  if (config.gaussian_output.enabled) {
    publisher_ = std::make_unique<GaussianRenderPublisher>(
        node, config.gaussian_output.optical_frame,
        config.global_output.map_frame);
    publication_ = std::make_unique<GaussianRenderOutput>(
        config.gaussian_output,
        [publisher = publisher_.get()](const auto& snapshot) {
          return publisher->Publish(snapshot);
        });
  }
}

void OnlineGaussian::Observe(const OnlineSubmapUpdate& update) {
  if (mapper_) mapper_->Observe(update);
}

void OnlineGaussian::Poll() {
  if (!mapper_) return;

  mapper_->Poll();
  const auto stats = mapper_->GetRenderStats();
  if (stats.wait_changes != reported_render_wait_) {
    LOG(INFO) << "livo_atlas gaussian joint waiting: reason="
              << stats.wait_reason
              << " map_sequence=" << stats.wait_map_sequence
              << " changes=" << stats.wait_changes;
    reported_render_wait_ = stats.wait_changes;
  }
  const auto render = mapper_->GetLatestRender();
  if (render && render->request.job_id != reported_render_job_) {
    ReportJointRender(*render);
    reported_render_job_ = render->request.job_id;
  }
  if (publication_) publication_->Observe(mapper_->GetRenderSnapshot());
}

void OnlineGaussian::FinishPublication() {
  if (!publication_) return;

  // A closed ROS context is counted as unavailable.
  publication_->Finish();
  const auto s = publication_->GetStats();
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

bool OnlineGaussian::Finish(const OnlineGaussianOptions& options) {
  if (!mapper_) return true;

  bool succeeded = true;
  if (options.finalize_on_shutdown) {
    const auto finalization =
        mapper_->Finalize(options.finalize_timeout);
    ReportFinalization(finalization);
    succeeded = finalization.status == GaussianFinalizationStatus::kSucceeded &&
                finalization.report_persisted;
  } else {
    mapper_->Finish();
  }
  if (options.joint_render_enabled) {
    const auto r = mapper_->GetRenderStats();
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
  const auto s = mapper_->GetStats();
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
  if (options.persist_submaps) {
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
    const auto map = mapper_->GetMapSnapshot();
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
  return succeeded;
}
#else

void OnlineGaussian::ValidateConfig(const LivoConfig& config) {
  if (config.gaussian.enabled) {
    throw std::invalid_argument("gaussian.enabled requires a Linux build");
  }
}

OnlineGaussian::OnlineGaussian(RosNode&, LivoSystem&, const LivoConfig&) {}

void OnlineGaussian::Observe(const OnlineSubmapUpdate&) {}

void OnlineGaussian::Poll() {}

void OnlineGaussian::FinishPublication() {}

bool OnlineGaussian::Finish(const OnlineGaussianOptions&) { return true; }

#endif

OnlineGaussian::~OnlineGaussian() = default;

}  // namespace livo_atlas::app
