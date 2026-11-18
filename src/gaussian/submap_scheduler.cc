#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

#include "livo_atlas/gaussian/online_gaussian_mapper.h"
#include "mapping/pose_math.h"

namespace livo_atlas {
namespace {

std::vector<std::uint64_t> AssetIds(const GaussianMapSnapshot& map) {
  std::vector<std::uint64_t> result;
  for (const auto& entry : map.entries)
    if (entry.asset) result.push_back(entry.asset->asset_id);
  std::sort(result.begin(), result.end());
  return result;
}

}  // namespace

void OnlineGaussianMapper::ObserveSubmaps(const OnlineSubmapUpdate& update) {
  map_index_.Observe(update);
  const auto map = map_index_.Snapshot();
  poses_ = update.poses;
  if (!map.entries.empty()) submap_ = map.entries.back().group;
  if (update.keyframe) {
    ++stats_.observed;
    if (pending_) ++stats_.coalesced;
    pending_ = Source{update.keyframe, poses_, *submap_, 0, 0, Clock::now()};
    stats_.pending = stats_.peak_pending = 1;
  }
  if (ready_ && (!submap_ || ready_->submap_id != submap_->primary.id ||
                 ready_->pose_version != poses_->version ||
                 map.entries[ready_->submap_id].availability !=
                     GaussianAvailability::kCurrent))
    Invalidate();
  stats_.has_submap = submap_.has_value();
  stats_.submap_id = submap_ ? submap_->primary.id : 0;
  stats_.pose_version = poses_->version;
  ++state_sequence_;
  RefreshRenderDemand();
}

bool OnlineGaussianMapper::CurrentTarget(const GaussianTarget& target) const {
  if (!poses_ || target.pose_version != poses_->version ||
      target.frame_count > poses_->frame_count)
    return false;
  const auto current =
      map_index_.MakeTarget(target.group.primary.id, target.generation);
  if (target.submap_from_body.size() > current.submap_from_body.size())
    return false;
  for (std::size_t i = 0; i < target.submap_from_body.size(); ++i)
    if (!pose_math::Equal(target.submap_from_body[i],
                          current.submap_from_body[i]))
      return false;
  return pose_math::Equal(target.parent_from_submap,
                          current.parent_from_submap);
}

bool OnlineGaussianMapper::QueueSubmapCommand(Command command) {
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock || command_ || reply_) return false;
  // Target preparation and validation may outlive the earlier dispatch check.
  // Recheck at mailbox admission, before accounting or queuing new work.
  if (finalization_phase_ == FinalizationPhase::kRepairing &&
      Clock::now() >= finalization_deadline_) {
    if (command.kind == CommandKind::kActivate ||
        command.kind == CommandKind::kView)
      return false;
    if (command.kind == CommandKind::kDecision && command.accept) {
      command.accept = false;
      ++stats_.stale;
    }
  }
  if (command.kind != CommandKind::kDecision &&
      command.kind != CommandKind::kSave &&
      command.kind != CommandKind::kSaveRender &&
      command.kind != CommandKind::kWriteMap &&
      command.kind != CommandKind::kStatus) {
    if (next_job_ == std::numeric_limits<std::uint64_t>::max())
      throw std::overflow_error("Gaussian job ID overflow");
    command.job_id = next_job_++;
  }
  if (command.kind == CommandKind::kView ||
      command.kind == CommandKind::kActivate) {
    if (command.kind == CommandKind::kView || command.content) {
      ++stats_.submitted;
      stats_.in_flight = stats_.peak_in_flight = 1;
      published_since_work_ = false;
    }
    if (submap_work_->kind == CommandKind::kView) {
      // An activation is part of a foreground attempt even if it becomes
      // stale. Its following view belongs to that same attempt.
      if (command.kind == CommandKind::kActivate || !foreground_started_)
        foreground_attempts_ = std::min(2, foreground_attempts_ + 1);
      foreground_started_ = true;
    } else {
      foreground_attempts_ = 0;
    }
  }
  if (finalization_phase_ == FinalizationPhase::kRepairing &&
      command.kind == CommandKind::kActivate &&
      command.operation == GaussianOperation::kRebuild)
    ++finalization_result_->repair_attempts;
  if (command.kind == CommandKind::kRender) {
    command.render_request->job_id = command.job_id;
    ++render_stats_.submitted;
    render_stats_.in_flight = render_stats_.peak_in_flight = 1;
    render_stats_.pending = 0;
    render_pending_ = false;
    render_attempts_ = 0;
    render_dispatched_at_ = Clock::now();
  }
  if (render_pending_ && command.map &&
      (command.kind == CommandKind::kWriteMap ||
       command.kind == CommandKind::kMapCommitted))
    SetRenderWait("synchronization", command.map->map_sequence);
  command_ = std::move(command);
  busy_ = true;
  lock.unlock();
  condition_.notify_one();
  return true;
}

void OnlineGaussianMapper::EndSubmapWork(bool retry) {
  if (retry && !finishing_ && finalization_phase_ == FinalizationPhase::kNone &&
      !pending_ && submap_work_ && submap_work_->kind == CommandKind::kView) {
    pending_ = submap_work_->source;
    stats_.pending = stats_.peak_pending = 1;
  }
  if (submap_work_ && options_.joint_render_enabled) {
    render_attempts_ = std::min(2, render_attempts_ + 1);
    render_sync_attempted_ = false;
  }
  submap_work_.reset();
  foreground_started_ = false;
  stats_.in_flight = 0;
}

void OnlineGaussianMapper::DispatchSubmaps() {
  Command command;
  if (awaiting_decision_) {
    command = *candidate_command_;
    command.kind = CommandKind::kDecision;
    command.preview = candidate_;
    command.accept = CanAcceptSubmapCandidate(command);
    if (!command.accept) ++stats_.stale;
    QueueSubmapCommand(std::move(command));
    return;
  }
  if (map_to_ack_) {
    command.kind = CommandKind::kMapCommitted;
    command.map = map_to_ack_;
    command.asset_ids = AssetIds(*map_to_ack_);
    command.generation = generation_;
    QueueSubmapCommand(std::move(command));
    return;
  }
  if (prune_asset_ && !checkpoint_needs_map_) {
    command.kind = CommandKind::kPrune;
    command.asset_id = *prune_asset_;
    command.state_sequence = acknowledged_map_;
    command.generation = generation_;
    command.pose_version = poses_ ? poses_->version : 0;
    QueueSubmapCommand(std::move(command));
    return;
  }

  const auto map = map_index_.Snapshot();
  const auto resident =
      std::find_if(map.entries.begin(), map.entries.end(),
                   [](const auto& entry) { return entry.resident; });
  // Do not reserve the next content attempt while publishing the previous
  // confirmation. Otherwise continuous input can hide every render boundary.
  if (options_.joint_render_enabled && !submap_work_ && unsaved_) {
    command.kind = CommandKind::kSave;
    command.preview = unsaved_;
    QueueSubmapCommand(std::move(command));
    return;
  }
  if (!submap_work_ && DispatchRender(map)) return;
  const bool quiescing = finalization_phase_ == FinalizationPhase::kQuiescing;
  const bool repairing = finalization_phase_ == FinalizationPhase::kRepairing;
  if ((finishing_ || quiescing) && !busy_) EndSubmapWork(false);
  if (!finishing_ && !quiescing && !submap_work_) {
    auto repair = map_index_.NextRepair();
    if (repair && (repairing || !pending_ || foreground_attempts_ >= 2)) {
      if (repairing &&
          std::find(finalization_result_->eligible_submap_ids.begin(),
                    finalization_result_->eligible_submap_ids.end(),
                    *repair) == finalization_result_->eligible_submap_ids.end())
        throw std::runtime_error("Finalization content set changed");
      command.kind = CommandKind::kActivate;
      command.target = map_index_.MakeTarget(*repair, 0);
      command.source.observed_at = Clock::now();
      map_index_.MarkRepairAttempt(*repair);
      submap_work_ = command;
    } else if (!repairing && pending_) {
      if (!map_index_.CanTrain(pending_->submap.primary.id)) {
        ++stats_.resource_limited;
        pending_.reset();
        stats_.pending = 0;
      } else {
        command.kind = CommandKind::kView;
        command.source = std::move(*pending_);
        command.target =
            map_index_.MakeTarget(command.source.submap.primary.id, 0);
        submap_work_ = command;
        pending_.reset();
        stats_.pending = 0;
      }
    } else if (!repairing && resident != map.entries.end() &&
               (!remote_target_ ||
                remote_target_->pose_version != poses_->version)) {
      command.kind = CommandKind::kActivate;
      command.target = map_index_.MakeTarget(resident->group.primary.id, 0);
      command.source.observed_at = Clock::now();
      submap_work_ = command;
    }
  }

  bool activate = false;
  if (submap_work_) {
    const auto id = submap_work_->target.group.primary.id;
    activate = submap_work_->kind == CommandKind::kActivate ||
               !remote_target_ || remote_target_->group.primary.id != id ||
               remote_target_->pose_version != poses_->version;
  }
  // A stale empty activation has no candidate/decision to end the work. Yield
  // at this safe boundary too, before another prerequisite can consume time.
  if (!finishing_ && submap_work_ && submap_work_->kind == CommandKind::kView &&
      activate && foreground_attempts_ >= 2 && map_index_.NextRepair()) {
    EndSubmapWork(true);
    return;
  }
  const bool leaving =
      submap_work_ && activate && resident != map.entries.end() &&
      (resident->group.primary.id != submap_work_->target.group.primary.id ||
       map.entries[submap_work_->target.group.primary.id].availability ==
           GaussianAvailability::kNeedsRebuild);
  if (resident != map.entries.end() && resident->dirty &&
      (leaving || finishing_)) {
    command = {};
    command.kind = CommandKind::kCheckpoint;
    command.target = resident->content_target;
    command.target.generation = generation_;
    command.content = resident->content;
    command.asset_id = next_asset_;
    command.frame_ids = resident->frame_ids;
    command.gaussian_count = resident->gaussian_count;
    if (QueueSubmapCommand(std::move(command))) ++next_asset_;
    return;
  }
  if (unsaved_) {
    command = {};
    command.kind = CommandKind::kSave;
    command.preview = unsaved_;
    QueueSubmapCommand(std::move(command));
    return;
  }
  if (map.map_sequence > acknowledged_map_ &&
      (checkpoint_needs_map_ || !submap_work_ || !published_since_work_)) {
    command = {};
    command.kind = CommandKind::kWriteMap;
    command.map = std::make_shared<GaussianMapSnapshot>(map);
    QueueSubmapCommand(std::move(command));
    return;
  }
  if (state_sequence_ > saved_status_sequence_ &&
      (!submap_work_ || !published_since_work_)) {
    command = {};
    command.kind = CommandKind::kStatus;
    command.generation = generation_;
    command.pose_version = poses_ ? poses_->version : 0;
    command.state_sequence = state_sequence_;
    if (submap_) command.submap_id = submap_->primary.id;
    command.ready = ready_ && !saved_preview_name_.empty();
    command.preview_name = command.ready ? saved_preview_name_ : "";
    QueueSubmapCommand(std::move(command));
    return;
  }
  if (unsaved_render_) {
    command = {};
    command.kind = CommandKind::kSaveRender;
    command.render = unsaved_render_;
    QueueSubmapCommand(std::move(command));
    return;
  }
  if (!submap_work_ || finishing_ || quiescing) return;
  if (repairing && Clock::now() >= finalization_deadline_) return;

  command = *submap_work_;
  const auto id = command.target.group.primary.id;
  const auto& entry = map.entries[id];
  command.target = map_index_.MakeTarget(
      id, remote_target_ ? remote_target_->generation : 0);
  if (activate) {
    if (generation_ == std::numeric_limits<std::uint64_t>::max())
      throw std::overflow_error("Gaussian generation overflow");
    command.target.generation = ++generation_;
    stats_.generation = generation_;
    latest_render_.reset();
    command.kind = CommandKind::kActivate;
    command.content = entry.content;
    command.operation =
        entry.availability == GaussianAvailability::kNeedsRebuild
            ? GaussianOperation::kRebuild
            : GaussianOperation::kReuse;
    command.frame_ids = entry.frame_ids;
    command.gaussian_count = entry.gaussian_count;
    if (entry.content && !entry.resident) {
      if (!entry.asset || entry.asset->content != *entry.content)
        throw std::runtime_error(
            "Nonresident Gaussian content lacks its asset");
      command.asset = entry.asset;
    }
  } else {
    command.kind = CommandKind::kView;
    command.operation = GaussianOperation::kView;
    command.source.poses = poses_;
    command.source.submap = entry.group;
    command.frame_ids = entry.frame_ids;
    if (std::find(command.frame_ids.begin(), command.frame_ids.end(),
                  command.source.keyframe->id) != command.frame_ids.end()) {
      EndSubmapWork(false);
      return;
    }
    command.frame_ids.push_back(command.source.keyframe->id);
    if (command.frame_ids.size() > 10)
      command.frame_ids.erase(command.frame_ids.begin());
  }
  QueueSubmapCommand(std::move(command));
}

void OnlineGaussianMapper::ConfirmPreview(
    const std::shared_ptr<GaussianPreview>& preview,
    Clock::time_point observed_at) {
  preview->latency_seconds =
      std::chrono::duration<double>(Clock::now() - observed_at).count();
  preview->confirmed_at_unix_seconds =
      std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  ++stats_.accepted;
  if (preview->operation != GaussianOperation::kReuse && preview->steps > 0 &&
      preview->nonzero_gradient && preview->parameter_updated &&
      preview->supervised_depth_pixels > 0)
    ++stats_.effective_training;
  stats_.last_latency_seconds = preview->latency_seconds;
  stats_.total_latency_seconds += preview->latency_seconds;
  stats_.max_latency_seconds =
      std::max(stats_.max_latency_seconds, preview->latency_seconds);
  constexpr std::array<double, 7> kBounds = {.1, .25, .5, 1, 2, 5, 10};
  const auto bucket = std::lower_bound(kBounds.begin(), kBounds.end(),
                                       preview->latency_seconds);
  ++stats_.latency_buckets[std::distance(kBounds.begin(), bucket)];
}

void OnlineGaussianMapper::HandleSubmapReply(Reply reply) {
  busy_ = false;
  const auto& command = reply.command;
  switch (reply.kind) {
    case ReplyKind::kConnected:
      connected_ = true;
      break;
    case ReplyKind::kActivated:
      map_index_.RecordResidentReleased();
      remote_target_ = command.target;
      if (options_.joint_render_enabled && !CurrentTarget(command.target))
        EndSubmapWork(true);
      break;
    case ReplyKind::kCandidate:
      ++stats_.completed;
      candidate_ = std::move(reply.preview);
      candidate_command_ = command;
      awaiting_decision_ = true;
      break;
    case ReplyKind::kConfirmed:
      awaiting_decision_ = false;
      if (command.accept) {
        map_index_.RecordConfirmed(command.target, command.operation,
                                   command.job_id, candidate_->frame_ids,
                                   candidate_->gaussian_count);
        remote_target_ = command.target;
        if (command.asset) ++stats_.loads;
        if (command.operation == GaussianOperation::kReuse) ++stats_.rebinds;
        if (command.operation == GaussianOperation::kRebuild) ++stats_.rebuilds;
        ConfirmPreview(candidate_, submap_work_->source.observed_at);
        if (CurrentTarget(command.target) && submap_ &&
            command.target.group.primary.id == submap_->primary.id) {
          ready_ = candidate_;
          unsaved_ = ready_;
        } else {
          ++stats_.stale;
        }
        if (command.operation == GaussianOperation::kView ||
            submap_work_->kind == CommandKind::kActivate)
          EndSubmapWork(false);
      } else {
        remote_target_.reset();
        EndSubmapWork(true);
      }
      candidate_.reset();
      candidate_command_.reset();
      stats_.in_flight = 0;
      ++state_sequence_;
      break;
    case ReplyKind::kCheckpoint: {
      const auto before = map_index_.Snapshot();
      const auto& old = before.entries[reply.asset->content.submap_id].asset;
      if (!map_index_.RecordCheckpoint(*reply.asset))
        throw std::runtime_error(
            "Checkpoint no longer matches confirmed content");
      if (old) prune_asset_ = old->asset_id;
      ++stats_.checkpoints;
      checkpoint_needs_map_ = true;
      break;
    }
    case ReplyKind::kMapWritten:
      map_to_ack_ = command.map;
      published_since_work_ = true;
      break;
    case ReplyKind::kMapCommitted:
      acknowledged_map_ = command.map->map_sequence;
      map_to_ack_.reset();
      checkpoint_needs_map_ = false;
      break;
    case ReplyKind::kPruned:
      ++stats_.prunes;
      prune_asset_.reset();
      break;
    case ReplyKind::kSaved:
      if (ready_ && ready_->job_id == command.preview->job_id)
        saved_preview_name_ = reply.detail;
      if (unsaved_ && unsaved_->job_id == command.preview->job_id)
        unsaved_.reset();
      ++state_sequence_;
      break;
    case ReplyKind::kStatus:
      saved_status_sequence_ = command.state_sequence;
      published_since_work_ = true;
      break;
    case ReplyKind::kSkipped:
      if (reply.detail == "no_image")
        ++stats_.skipped_no_image;
      else if (reply.detail == "no_depth")
        ++stats_.skipped_no_depth;
      else
        ++stats_.resource_limited;
      EndSubmapWork(false);
      break;
    case ReplyKind::kRendered:
    case ReplyKind::kRenderError:
    case ReplyKind::kRenderSaved:
      HandleRenderReply(std::move(reply));
      break;
    case ReplyKind::kError:
      Fail(reply.detail);
      break;
    default:
      throw std::runtime_error("Unexpected Gaussian submap reply");
  }
  RefreshRenderDemand();
}

bool OnlineGaussianMapper::SubmapsDrained() const {
  if (!connected_ || busy_ || awaiting_decision_ || submap_work_ ||
      map_to_ack_ || prune_asset_ || unsaved_ || unsaved_render_ ||
      checkpoint_needs_map_ ||
      state_sequence_ != saved_status_sequence_)
    return false;
  const auto map = map_index_.Snapshot();
  return map.map_sequence == acknowledged_map_ &&
         std::none_of(map.entries.begin(), map.entries.end(),
                      [](const auto& entry) { return entry.dirty; });
}

}  // namespace livo_atlas
