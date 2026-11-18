#include "livo_atlas/gaussian/online_gaussian_mapper.h"

#include <algorithm>
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
      finalization_phase_ != FinalizationPhase::kNone || !poses_ ||
      unsaved_ || Clock::now() - render_dispatched_at_ <
                      options_.joint_render_interval)
    return false;
  const bool content = pending_ || map_index_.NextRepair() ||
      (remote_target_ && remote_target_->pose_version != poses_->version);
  if (content && render_attempts_ < 2) return false;
  std::string reason;
  auto request = SelectGaussianRender(map, *poses_, generation_, &reason);
  if (!request) {
    if (render_skip_sequence_ != map.map_sequence) {
      ++render_stats_.skipped;
      render_skip_sequence_ = map.map_sequence;
    }
    return false;
  }
  Command command;
  // Admission always rebuilds the references from this exact map. A pending
  // request owns no asset references while training/checkpoint/prune runs.
  if (map.map_sequence != acknowledged_map_) {
    command.kind = CommandKind::kWriteMap;
    command.map = std::make_shared<GaussianMapSnapshot>(map);
  } else {
    command.kind = CommandKind::kRender;
    command.render_request = std::move(request);
  }
  return QueueSubmapCommand(std::move(command));
}

void OnlineGaussianMapper::HandleRenderReply(Reply reply) {
  render_stats_.in_flight = 0;
  if (reply.kind == ReplyKind::kRenderError) {
    ++render_stats_.failed;
    render_stats_.disabled = true;
    render_stats_.error = std::move(reply.detail);
    StopRendering();
    return;
  }
  if (reply.kind == ReplyKind::kRenderSaved) {
    ++render_stats_.saved;
    if (unsaved_render_ && unsaved_render_->request.job_id ==
                               reply.command.render->request.job_id)
      unsaved_render_.reset();
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
