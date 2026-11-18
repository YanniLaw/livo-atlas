#include "livo_atlas/gaussian/online_gaussian_mapper.h"

#include <algorithm>
#include <chrono>
#include <string>
#include <utility>

namespace livo_atlas {

void OnlineGaussianMapper::RefreshRenderDemand() {
  if (!options_.joint_render_enabled || failed_ || finished_ || finishing_ ||
      finalization_phase_ != FinalizationPhase::kNone || render_stats_.disabled)
    return;
  const auto map = map_index_.Snapshot();
  if (map.map_sequence == render_sequence_) return;
  render_sequence_ = map.map_sequence;
  latest_render_.reset();
  if (render_pending_) ++render_stats_.coalesced;
  render_pending_ = true;
  render_stats_.pending = render_stats_.peak_pending = 1;
}

bool OnlineGaussianMapper::DispatchRender(const GaussianMapSnapshot& map) {
  if (!options_.joint_render_enabled || !render_pending_ ||
      render_stats_.disabled || finishing_ || failed_ || finished_ ||
      finalization_phase_ != FinalizationPhase::kNone || !poses_)
    return false;
  // An unavailable source cannot change without advancing this owner index.
  // Keep the demand, but avoid revalidating every canonical pose on each Poll.
  if (render_skip_sequence_ == map.map_sequence &&
      !render_unavailable_reason_.empty()) {
    SetRenderWait(render_unavailable_reason_, map.map_sequence);
    return false;
  }
  if (unsaved_) {
    SetRenderWait("storage", map.map_sequence);
    return false;
  }
  if (Clock::now() - render_dispatched_at_ < options_.joint_render_interval) {
    SetRenderWait("interval", map.map_sequence);
    return false;
  }
  const bool resident = std::any_of(
      map.entries.begin(), map.entries.end(),
      [](const auto& entry) { return entry.resident; });
  const bool content = pending_ || map_index_.NextRepair() ||
      (resident && (!remote_target_ ||
                    remote_target_->pose_version != poses_->version));
  std::string reason;
  auto request = SelectGaussianRender(map, *poses_, generation_, &reason);
  if (!request) {
    render_unavailable_reason_ = reason;
    SetRenderWait(reason, map.map_sequence);
    if (render_skip_sequence_ != map.map_sequence) {
      ++render_stats_.skipped;
      render_skip_sequence_ = map.map_sequence;
    }
    return false;
  }
  if (content && render_attempts_ < 2) {
    SetRenderWait("content", map.map_sequence);
    return false;
  }
  Command command;
  // Admission always rebuilds the references from this exact map. A pending
  // request owns no asset references while training/checkpoint/prune runs.
  if (map.map_sequence != acknowledged_map_) {
    SetRenderWait("synchronization", map.map_sequence);
    if (render_sync_attempted_ && content) {
      // One missed synchronization spends this opportunity. Keep the demand,
      // but let foreground/repair advance even if every acknowledgement ages.
      render_attempts_ = 0;
      render_sync_attempted_ = false;
      return false;
    }
    command.kind = CommandKind::kWriteMap;
    command.map = std::make_shared<GaussianMapSnapshot>(map);
    if (!QueueSubmapCommand(std::move(command))) return false;
    render_sync_attempted_ = true;
    return true;
  }
  command.kind = CommandKind::kRender;
  command.render_request = std::move(request);
  if (!QueueSubmapCommand(std::move(command))) return false;
  SetRenderWait("rendering", map.map_sequence);
  render_sync_attempted_ = false;
  return true;
}

void OnlineGaussianMapper::HandleRenderReply(Reply reply) {
  render_stats_.in_flight = 0;
  if (reply.kind == ReplyKind::kRenderError) {
    ++render_stats_.failed;
    render_stats_.disabled = true;
    render_stats_.error = std::move(reply.detail);
    unsaved_render_.reset();
    render_stats_.pending_storage = 0;
    StopRendering();
    return;
  }
  if (reply.kind == ReplyKind::kRenderSaved) {
    ++render_stats_.saved;
    if (unsaved_render_ && unsaved_render_->request.job_id ==
                               reply.command.render->request.job_id) {
      unsaved_render_.reset();
      render_stats_.pending_storage = 0;
    }
    return;
  }
  if (failed_ || finished_ || finishing_ ||
      finalization_phase_ != FinalizationPhase::kNone || !poses_ ||
      !IsCurrentGaussianRender(reply.render->request, map_index_.Snapshot(),
                                *poses_)) {
    ++render_stats_.stale;
    return;
  }
  auto& render = *reply.render;
  const auto now = Clock::now();
  render.accepted_at_steady_seconds =
      std::chrono::duration<double>(now.time_since_epoch()).count();
  render.accepted_at_unix_seconds = std::chrono::duration<double>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  render.latency_seconds =
      std::chrono::duration<double>(now - render_dispatched_at_).count();
  ++render_stats_.accepted;
  render_stats_.dual_accepted += render.request.blocks.size() == 2;
  render_stats_.last_latency_seconds = render.latency_seconds;
  render_stats_.max_latency_seconds =
      std::max(render_stats_.max_latency_seconds, render.latency_seconds);
  render_stats_.total_latency_seconds += render.latency_seconds;
  latest_render_ = std::move(reply.render);
  unsaved_render_ = latest_render_;
  render_stats_.pending_storage = 1;
}

void OnlineGaussianMapper::SetRenderWait(const std::string& reason,
                                         std::uint64_t map_sequence) {
  if (render_stats_.wait_reason == reason &&
      render_stats_.wait_map_sequence == map_sequence)
    return;
  render_stats_.wait_reason = reason;
  render_stats_.wait_map_sequence = map_sequence;
  ++render_stats_.wait_changes;
}

void OnlineGaussianMapper::StopRendering() {
  latest_render_.reset();
  render_pending_ = false;
  render_stats_.pending = 0;
}

std::shared_ptr<const GaussianRender> OnlineGaussianMapper::GetLatestRender()
    const {
  return failed_ || finished_ || finishing_ ||
                 finalization_phase_ != FinalizationPhase::kNone
             ? nullptr : latest_render_;
}

GaussianRenderStats OnlineGaussianMapper::GetRenderStats() const {
  return render_stats_;
}

}  // namespace livo_atlas
