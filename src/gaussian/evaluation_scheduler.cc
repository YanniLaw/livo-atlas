#include <algorithm>
#include <chrono>
#include <filesystem>
#include <stdexcept>

#include "gaussian/mapper_io.h"
#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/gaussian/online_gaussian_mapper.h"

namespace livo_atlas {
namespace {

double UnixNow() {
  return std::chrono::duration<double>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

bool Pending(const GaussianEvaluationStatus& status) {
  return status.status == "waiting_seal" || status.status == "pending";
}

}  // namespace

std::vector<GaussianEvaluationStatus>
OnlineGaussianMapper::GetEvaluationStatus() const {
  return evaluation_status_;
}

void OnlineGaussianMapper::RefreshEvaluationDemand() {
  if (!options_.online_evaluation_enabled || finishing_ || finished_ ||
      failed_ || finalization_phase_ != FinalizationPhase::kNone)
    return;
  const auto map = map_index_.Snapshot();
  for (auto& state : evaluation_status_) {
    if (state.status != "waiting_seal" ||
        state.query.submap_id >= map.entries.size())
      continue;
    if (map.entries[state.query.submap_id].group.state ==
        OnlineSubmapState::kSealed) {
      state.status = "pending";
      state.sealed_at_unix_seconds = UnixNow();
      ++evaluation_sequence_;
    }
  }
}

bool OnlineGaussianMapper::DispatchEvaluation(const GaussianMapSnapshot& map) {
  if (!options_.online_evaluation_enabled || finishing_ || failed_ ||
      finished_ || submap_work_ ||
      finalization_phase_ != FinalizationPhase::kNone)
    return false;
  const bool content =
      pending_ || map_index_.NextRepair() ||
      std::any_of(map.entries.begin(), map.entries.end(),
                  [&](const auto& entry) {
                    return entry.resident &&
                           (!remote_target_ ||
                            remote_target_->pose_version != map.pose_version);
                  });
  if (content && evaluation_content_attempts_ < 2) return false;
  for (auto& state : evaluation_status_) {
    if (state.status != "pending" ||
        state.query.submap_id >= map.entries.size())
      continue;
    const auto& entry = map.entries[state.query.submap_id];
    if (!entry.content) {
      if (pending_ && pending_->submap.primary.id == state.query.submap_id)
        continue;
      state.status = "skipped";
      state.reason = "missing_model";
      ++evaluation_sequence_;
      continue;
    }
    std::string reason;
    auto selected = SelectGaussianEvaluation(
        map, *map_index_.GetPoseSnapshot(), generation_, state.query.camera,
        options_.joint_render_policy, &reason);
    if (!selected) continue;
    Command command;
    if (map.map_sequence != acknowledged_map_) {
      if (evaluation_sync_attempted_ && content) {
        evaluation_content_attempts_ = 0;
        evaluation_sync_attempted_ = false;
        return false;
      }
      command.kind = CommandKind::kWriteMap;
      command.map = std::make_shared<GaussianMapSnapshot>(map);
      if (QueueSubmapCommand(std::move(command)))
        evaluation_sync_attempted_ = true;
      return true;
    }
    command.kind = CommandKind::kEvaluate;
    command.evaluation = GaussianEvaluationRequest{
        *selected, state.query.camera, evaluation_split_->split_id(),
        state.sealed_at_unix_seconds, UnixNow()};
    command.source.poses = map_index_.GetPoseSnapshot();
    command.keyframes = system_.GetKeyframeReadView();
    if (QueueSubmapCommand(std::move(command)))
      evaluation_sync_attempted_ = false;
    // Busy mailbox retains this reserved opportunity, as with live preview.
    return true;
  }
  return false;
}

void OnlineGaussianMapper::HandleEvaluationReply(const Reply& reply) {
  if (reply.kind == ReplyKind::kEvaluationStatus) {
    evaluation_saved_sequence_ = reply.command.state_sequence;
    return;
  }
  const auto& captured = *reply.command.evaluation;
  auto it = std::find_if(evaluation_status_.begin(), evaluation_status_.end(),
                         [&](const auto& state) {
                           return state.query.camera.keyframe_id ==
                                  captured.camera.keyframe_id;
                         });
  if (it == evaluation_status_.end() || it->status != "in_flight" ||
      it->job_id != captured.sources.job_id)
    throw std::runtime_error(
        "Evaluation result has no matching admitted query");
  // This is a historical measurement. New frontend/pose data does not erase
  // the worker's already captured and verified source identity.
  it->status = reply.kind == ReplyKind::kEvaluated ? "succeeded" : "failed";
  it->reason = reply.detail.substr(0, 128);
  it->error_code = reply.evaluation_error_code;
  it->report_sha256 = reply.evaluation_report_sha256;
  ++evaluation_sequence_;
}

void OnlineGaussianMapper::StopEvaluation() {
  if (!options_.online_evaluation_enabled) return;
  const auto frontier = map_index_.frontier().frame_count;
  for (auto& state : evaluation_status_) {
    if (!Pending(state)) continue;
    if (failed_) {
      state.status = "failed";
      state.reason = stats_.error.substr(0, 128);
      state.error_code = "session";
    } else if (state.query.camera.keyframe_id >= frontier) {
      state.status = "skipped";
      state.reason = "not_in_scope";
    } else {
      state.reason = state.status == "waiting_seal" ? "shutdown_before_seal"
                                                    : "shutdown_pending";
      state.status = "cancelled";
    }
    ++evaluation_sequence_;
  }
}

void OnlineGaussianMapper::WriteEvaluationStatus(
    const std::vector<GaussianEvaluationStatus>& states,
    const std::string& session_error) const {
  const auto pending =
      std::count_if(states.begin(), states.end(), [](const auto& state) {
        return Pending(state) || state.status == "in_flight";
      });
  const auto failed =
      std::count_if(states.begin(), states.end(),
                    [](const auto& state) { return state.status == "failed"; });
  const auto json = gaussian_mapper_io::Json([&](auto& out) {
    out << "schema" << 1 << "kind"
        << "livo_atlas.gaussian_online_evaluation_status"
        << "phase" << "online_snapshot" << "session_id" << session_id_
        << "split_id" << evaluation_split_->split_id() << "status"
        << (!session_error.empty() ? "failed"
            : pending              ? "running"
            : failed               ? "failed"
                                   : "complete")
        << "query_count" << static_cast<int>(states.size()) << "pending_count"
        << static_cast<int>(pending) << "failed_count"
        << static_cast<int>(failed) << "session_error"
        << session_error.substr(0, 1024) << "queries" << "[";
    for (const auto& state : states) {
      out << "{";
      gaussian_mapper_io::Id(out, "keyframe_id",
                             state.query.camera.keyframe_id);
      gaussian_mapper_io::Id(out, "submap_id", state.query.submap_id);
      gaussian_mapper_io::Id(out, "job_id", state.job_id);
      gaussian_mapper_io::Id(out, "map_sequence", state.map_sequence);
      out << "phase" << "online_snapshot" << "status" << state.status
          << "reason" << state.reason.substr(0, 128) << "error_code"
          << state.error_code << "sealed_at_unix_seconds"
          << state.sealed_at_unix_seconds << "report_sha256"
          << state.report_sha256 << "}";
    }
    out << "]";
  });
  if (json.size() > 128 * 1024)
    throw std::runtime_error("Evaluation status exceeds its metadata budget");
  const auto root = options_.output_directory / "evaluations";
  std::filesystem::create_directories(root);
  const auto temporary = root / ".online-status.pending.json";
  gaussian_mapper_io::WriteFile(temporary, json);
  std::filesystem::rename(temporary, root / "online-status.json");
}

}  // namespace livo_atlas
