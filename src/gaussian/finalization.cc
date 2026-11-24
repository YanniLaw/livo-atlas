#include <algorithm>
#include <stdexcept>
#include <thread>

#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/gaussian/online_gaussian_mapper.h"
#include "livo_atlas/io/gaussian_map_writer.h"
#include "mapping/pose_math.h"

namespace livo_atlas {
namespace {

bool SamePoses(const PoseSnapshot& a, const PoseSnapshot& b) {
  if (a.version != b.version || a.base_version != b.base_version ||
      a.frame_count != b.frame_count ||
      a.optimized_frame_count != b.optimized_frame_count ||
      a.parent_frame != b.parent_frame || a.tail_policy != b.tail_policy ||
      a.poses.size() != b.poses.size())
    return false;
  for (std::size_t i = 0; i < a.poses.size(); ++i)
    if (a.poses[i].id != b.poses[i].id ||
        !pose_math::Equal(a.poses[i].parent_from_body(),
                          b.poses[i].parent_from_body()))
      return false;
  return true;
}

GaussianFinalizationCounters Counters(const OnlineGaussianStats& stats) {
  return {stats.accepted, stats.effective_training, stats.rebuilds,
          stats.checkpoints};
}

}  // namespace

bool OnlineGaussianMapper::CanAcceptSubmapCandidate(
    const Command& command) const {
  if (finishing_ || finalization_phase_ == FinalizationPhase::kQuiescing)
    return false;
  if (finalization_phase_ == FinalizationPhase::kRepairing) {
    const auto& ids = finalization_result_->eligible_submap_ids;
    if (!system_.IsFinished() ||
        command.operation != GaussianOperation::kRebuild ||
        std::find(ids.begin(), ids.end(), command.target.group.primary.id) ==
            ids.end())
      return false;
  }
  return CurrentTarget(command.target) &&
         (finalization_phase_ != FinalizationPhase::kRepairing ||
          Clock::now() < finalization_deadline_);
}

bool OnlineGaussianMapper::FinalizationQuiescent() const {
  // Dirty content is allowed here: it is saved before switching or at the end.
  // Waiting for SubmapsDrained would require a checkpoint before E can exist.
  return connected_ && !busy_ && !awaiting_decision_ && !submap_work_ &&
         !map_to_ack_ && !prune_asset_ && !checkpoint_needs_map_;
}

bool OnlineGaussianMapper::FinalizationRepairsComplete() const {
  if (awaiting_decision_ || submap_work_) return false;
  const auto map = map_index_.Snapshot();
  std::vector<std::uint64_t> content;
  for (const auto& entry : map.entries) {
    if (!entry.content) continue;
    content.push_back(entry.group.primary.id);
    if (entry.availability != GaussianAvailability::kCurrent) return false;
  }
  if (content != finalization_result_->eligible_submap_ids)
    throw std::runtime_error("Finalization content set changed");
  // A metadata write may still be in flight. Finish owns that durable tail.
  return true;
}

GaussianFinalizationResult OnlineGaussianMapper::Finalize(
    std::chrono::milliseconds timeout) noexcept {
  if (finalization_result_ && finished_) return *finalization_result_;
  GaussianFinalizationResult rejected;
  rejected.session_id = session_id_;
  if (finished_) {
    rejected.reason = "already_finished";
    return rejected;
  }
  if (finalization_result_) {
    rejected.reason = "finalization_in_progress";
    return rejected;
  }
  if (!options_.enabled || !options_.persist_submaps) {
    rejected.reason = "persistence_required";
    return rejected;
  }
  if (timeout.count() < 1 || timeout.count() > 600000) {
    rejected.reason = "invalid_budget";
    return rejected;
  }
  PoseFrontier final_frontier;
  try {
    if (!system_.IsFinished()) {
      rejected.reason = "source_not_finished";
      return rejected;
    }
    const auto final = system_.GetPoseSnapshot();
    const auto map = map_index_.Snapshot();
    if (!SamePoses(*final, *map_index_.GetPoseSnapshot()) ||
        map.frame_count != final->frame_count ||
        map.pose_version != final->version) {
      rejected.reason = "source_mismatch";
      return rejected;
    }
    std::uint64_t next = 0;
    for (const auto& entry : map.entries) {
      if (entry.group.state != OnlineSubmapState::kSealed ||
          entry.group.primary.anchor_keyframe_id != next) {
        rejected.reason = "source_groups_not_sealed";
        return rejected;
      }
      next = entry.group.primary.keyframe_ids.back() + 1;
    }
    if (next != final->frame_count) {
      rejected.reason = "source_frontier_incomplete";
      return rejected;
    }
    final_frontier = final->frontier();
  } catch (const std::exception& error) {
    rejected.reason = std::string(error.what()).substr(0, 1024);
    return rejected;
  } catch (...) {
    rejected.reason = "source_validation_failed";
    return rejected;
  }

  const auto started = Clock::now();
  finalization_deadline_ = started + timeout;
  finalization_result_ = rejected;
  auto& result = *finalization_result_;
  result.reason.clear();
  result.target_valid = true;
  result.pose_version = final_frontier.version;
  result.frame_count = final_frontier.frame_count;
  result.optimized_frame_count = final_frontier.optimized_frame_count;
  result.parent_frame = final_frontier.parent_frame;
  result.before = Counters(stats_);
  pending_.reset();
  stats_.pending = 0;
  StopRendering();
  finalization_phase_ = FinalizationPhase::kQuiescing;
  bool repaired = false;
  bool timed_out = false;
  try {
    for (;;) {
      if (failed_) break;
      if (!system_.IsFinished() || !(final_frontier == map_index_.frontier()))
        throw std::runtime_error("Finalization source changed");
      if (Clock::now() >= finalization_deadline_) {
        timed_out = true;
        break;
      }
      if (finalization_phase_ == FinalizationPhase::kQuiescing &&
          FinalizationQuiescent()) {
        const auto map = map_index_.Snapshot();
        for (const auto& entry : map.entries)
          if (entry.content)
            result.eligible_submap_ids.push_back(entry.group.primary.id);
        result.eligible_set_frozen = true;
        finalization_phase_ = FinalizationPhase::kRepairing;
      }
      if (finalization_phase_ == FinalizationPhase::kRepairing &&
          FinalizationRepairsComplete()) {
        // Snapshot/completion checking can cross the deadline. Only the tail
        // admitted before it expires gets the independent cleanup budget.
        timed_out = Clock::now() >= finalization_deadline_;
        repaired = !timed_out;
        break;
      }
      Poll();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  } catch (const std::exception& error) {
    Fail(error.what());
  } catch (...) {
    Fail("Unknown finalization failure");
  }
  const auto repair_end = Clock::now();
  result.repair_seconds =
      std::chrono::duration<double>(repair_end - started).count();
  // From here only existing confirmation/catalogue/stop work may complete.
  // The deadline does not turn successful slow cleanup into repair timeout.
  finalization_phase_ = FinalizationPhase::kNone;
  Finish();
  const auto stopped = Clock::now();
  result.shutdown_seconds =
      std::chrono::duration<double>(stopped - repair_end).count();
  result.elapsed_seconds =
      std::chrono::duration<double>(stopped - started).count();
  result.after = Counters(stats_);
  result.worker_stopped = stop_acknowledged_;
  try {
    const auto map = map_index_.Snapshot();
    result.map_sequence = acknowledged_map_;
    result.durable = map.map_sequence == acknowledged_map_ &&
                     !checkpoint_needs_map_ && !map_to_ack_ && !prune_asset_;
    for (const auto& entry : map.entries) {
      if (!entry.content) {
        ++result.missing;
        continue;
      }
      result.current += entry.availability == GaussianAvailability::kCurrent;
      result.needs_rebuild +=
          entry.availability == GaussianAvailability::kNeedsRebuild;
      result.dirty += entry.dirty;
      result.durable = result.durable && !entry.dirty && entry.asset &&
                       entry.asset->content == *entry.content;
      if (entry.asset) result.asset_ids.push_back(entry.asset->asset_id);
      if (repaired && (!entry.asset_current ||
                       entry.availability != GaussianAvailability::kCurrent))
        repaired = false;
    }
    std::sort(result.asset_ids.begin(), result.asset_ids.end());
    if (failed_ || stats_.incomplete_shutdown || !result.worker_stopped ||
        !result.durable) {
      result.status = GaussianFinalizationStatus::kFailed;
      result.reason =
          stats_.error.empty() ? "incomplete_shutdown" : stats_.error;
    } else if (timed_out) {
      result.status = GaussianFinalizationStatus::kTimedOut;
      result.reason = "deadline";
    } else if (repaired && result.eligible_set_frozen &&
               result.current == result.eligible_submap_ids.size()) {
      result.status = GaussianFinalizationStatus::kSucceeded;
    } else {
      result.reason = "content_not_current";
    }
    WriteGaussianFinalizationResult(options_.output_directory, result);
    result.report_persisted = true;
  } catch (const GaussianMapCommitUncertainError&) {
    result.report_error = "commit_uncertain";
  } catch (const std::exception& error) {
    result.report_error = std::string(error.what()).substr(0, 1024);
  } catch (...) {
    result.report_error = "report_publication_failed";
  }
  result.total_seconds =
      std::chrono::duration<double>(Clock::now() - started).count();
  return result;
}

}  // namespace livo_atlas
