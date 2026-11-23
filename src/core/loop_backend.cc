#include "livo_atlas/core/loop_backend.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "common/timing.h"
#include "core/loop_commit.h"
#include "io/loop_admission_io.h"
#include "livo_atlas/core/livo_system.h"

namespace livo_atlas {
namespace {
void Require(bool condition, const char* message) {
  if (!condition) throw std::invalid_argument(message);
}
}  // namespace

LoopAdmissionOptions LoopBackend::ValidateSession(
    LivoSystem& system, LoopAdmissionOptions options) {
  ValidateLoopAdmissionOptions(options);
  system.CheckInputOpen();
  const auto& config = system.options_;
  Require(config.keyframes.enabled && config.submaps.enabled &&
              config.submap_mode == SubmapMode::kOnline &&
              config.submaps.max_keyframes <=
                  static_cast<int>(kMaxGeometryKeyframes),
          "Online loops require keyframes and online submaps with at most 64 "
          "members");
  if (system.loop_pose_updates_claimed_ || system.next_keyframe_id_ ||
      system.GetKeyframeReadView().frame_count() ||
      system.GetPoseSnapshot()->version)
    throw std::logic_error("Loop backend requires a new unclaimed session");
  return options;
}

LoopBackend::LoopBackend(LivoSystem& system, LoopAdmissionOptions options)
    : LoopBackend(system, options, [options](const auto& task) {
        return RunLoopTask(task, options);
      }) {}

LoopBackend::LoopBackend(LivoSystem& system, LoopAdmissionOptions options,
                         LoopTaskRunner runner)
    : system_(system),
      options_(ValidateSession(system, std::move(options))),
      store_(system.GetKeyframeReadView(), options_),
      worker_(std::move(runner)),
      poses_(system.GetPoseSnapshot()) {
  // All allocations, files and the worker are ready before the sticky claim.
  system_.ClaimLoopPoseUpdates();
}

LoopBackend::~LoopBackend() = default;

void LoopBackend::QueueQuery(std::uint64_t query_id) {
  if (budget_stopped_) return;
  if (pending_ && *pending_ != query_id) {
    if (!coalesced_) coalesced_ = Coalesced{*pending_, *pending_, 0};
    coalesced_->last = *pending_;
    ++coalesced_->count;
    ++stats_.skipped;
  }
  pending_ = query_id;
  stats_.peak_pending = 1;
}

void LoopBackend::RecordMembership(const OnlineSubmap& group) {
  ValidateSubmap(group.primary);
  const auto id = group.primary.id;
  Require(id <= submaps_.size() &&
              group.primary.keyframe_ids.size() <= kMaxGeometryKeyframes &&
              group.primary.keyframe_ids.back() < stats_.frame_count &&
              group.support_keyframe_ids.size() <= kMaxGeometryKeyframes &&
              (group.state == OnlineSubmapState::kActive ||
               group.state == OnlineSubmapState::kSealed),
          "Invalid observed online membership");
  if (id == submaps_.size()) {
    Require(group.membership_revision == 1 &&
                group.state == OnlineSubmapState::kActive &&
                group.primary.keyframe_ids.size() == 1 &&
                (submaps_.empty()
                     ? group.primary.keyframe_ids.front() == 0
                     : submaps_.back().state == OnlineSubmapState::kSealed &&
                           group.primary.keyframe_ids.front() ==
                               submaps_.back().primary.keyframe_ids.back() + 1),
            "Observed new submap does not extend the sealed prefix");
    submaps_.push_back(group);
  } else {
    auto& previous = submaps_[id];
    Require(id + 1 == submaps_.size() &&
                previous.state == OnlineSubmapState::kActive &&
                previous.membership_revision !=
                    std::numeric_limits<std::uint64_t>::max() &&
                group.membership_revision == previous.membership_revision + 1 &&
                group.primary.anchor_keyframe_id ==
                    previous.primary.anchor_keyframe_id &&
                group.support_keyframe_ids == previous.support_keyframe_ids,
            "Observed membership revision changed sealed history");
    const auto& old = previous.primary.keyframe_ids;
    const auto& next = group.primary.keyframe_ids;
    Require(group.state == OnlineSubmapState::kSealed
                ? old == next
                : next.size() == old.size() + 1 &&
                      std::equal(old.begin(), old.end(), next.begin()),
            "Observed membership revision does not append or seal");
    previous = group;
  }
  if (group.state == OnlineSubmapState::kSealed) QueueQuery(id);
}

void LoopBackend::StopDispatch() {
  if (!budget_stopped_) {
    budget_stopped_ = true;
    limit_event_pending_ = true;
  }
  pending_.reset();
}

void LoopBackend::Observe(const OnlineSubmapUpdate& update) noexcept {
  if (failure_) return;
  try {
    Require(!finished_ && bool(update.poses),
            "Missing observed pose snapshot or closed backend");
    const auto previous_count = stats_.frame_count;
    const auto count = previous_count + (update.keyframe ? 1 : 0);
    Require(update.poses->frame_count == count &&
                (update.poses->version == stats_.pose_version ||
                 (!update.keyframe && update.revisions.empty() &&
                  update.poses->version == stats_.pose_version + 1)),
            "Observed pose frontier or version is not consecutive");
    if (update.keyframe) {
      const auto& frame = *update.keyframe;
      Require(frame.id == previous_count && std::isfinite(frame.timestamp) &&
                  (!previous_count || frame.timestamp >= latest_timestamp_),
              "Observed raw keyframes are not consecutive and ordered");
      latest_timestamp_ = frame.timestamp;
    }
    stats_.frame_count = count;
    stats_.pose_version = update.poses->version;
    for (const auto& group : update.revisions)
      if (group.state == OnlineSubmapState::kSealed)
        latest_query_ = group.primary.id;
    if (count > options_.max_keyframes || stats_.accepted >= options_.max_loops)
      StopDispatch();
    if (budget_stopped_) return;
    ValidatePoseSnapshot(*update.poses);
    if (update.keyframe) {
      raw_poses_.push_back(update.keyframe->odom_from_body);
      timestamps_.push_back(update.keyframe->timestamp);
    }
    poses_ = update.poses;
    for (const auto& group : update.revisions) RecordMembership(group);
  } catch (...) {
    failure_ = std::current_exception();
  }
}

LoopAdmissionInput LoopBackend::CurrentInput(std::uint64_t query_id) const {
  Require(poses_ && poses_->frame_count == stats_.frame_count &&
              raw_poses_.size() == stats_.frame_count &&
              timestamps_.size() == stats_.frame_count,
          "Loop observer did not receive the full committed prefix");
  LoopAdmissionInput input;
  input.query_id = query_id;
  input.poses = *poses_;
  input.raw_poses = raw_poses_;
  input.accepted_loops = accepted_loops_;
  for (const auto& group : submaps_) {
    if (group.state != OnlineSubmapState::kSealed) break;
    input.submaps.push_back(
        {group, timestamps_.at(group.primary.keyframe_ids.front()),
         timestamps_.at(group.primary.keyframe_ids.back()),
         poses_->poses.at(group.primary.anchor_keyframe_id).map_from_body,
         poses_->version, poses_->frame_count, poses_->optimized_frame_count});
  }
  Require(query_id < input.submaps.size(), "Unknown pending loop query");
  return input;
}

void LoopBackend::FlushEvents() {
  if (coalesced_) {
    store_.WriteCoalesced(coalesced_->first, coalesced_->last,
                          coalesced_->count);
    coalesced_.reset();
  }
  if (limit_event_pending_) {
    store_.WriteResourceLimit(latest_query_, stats_.frame_count);
    ++stats_.resource_limited;
    limit_event_pending_ = false;
  }
}

void LoopBackend::Complete(LoopTaskResult result) {
  LogStageTiming(options_.pose_graph_enabled ? "loop_optimization_latency"
                                              : "loop_detection_latency",
                 dispatch_time_);
  ++stats_.completed;
  if (result.error) {
    try {
      store_.WriteQuery(result, LoopDisposition::kFailed, stats_.pose_version,
                        stats_.frame_count);
    } catch (...) {
    }
    std::rethrow_exception(result.error);
  }
  Require(bool(result.admission), "Worker returned no admission or exception");
  if (system_.options_.stage_timing) {
    for (const auto& candidate : result.admission->candidates) {
      LogStageSeconds("loop_registration",
                      candidate.verification.elapsed_seconds);
    }
    if (result.admission->trial) {
      LogStageSeconds("loop_graph_build",
                      result.admission->trial->graph_build_seconds);
      LogStageSeconds("loop_solve", result.admission->trial->solve_seconds);
    }
  }
  if (budget_stopped_ ||
      result.admission->status == LoopAdmissionStatus::kResourceLimit) {
    store_.WriteQuery(result, LoopDisposition::kResourceLimit,
                      stats_.pose_version, stats_.frame_count);
    ++stats_.resource_limited;
    return;
  }
  if (result.admission->status == LoopAdmissionStatus::kDetected) {
    Require(!options_.pose_graph_enabled &&
                result.admission->detected_target_id &&
                !result.admission->proposal,
            "Invalid detection-only result");
    store_.WriteQuery(result, LoopDisposition::kDetected, stats_.pose_version,
                      stats_.frame_count);
    ++stats_.detected;
    return;
  }
  if (!result.admission->proposal) {
    Require(result.admission->status != LoopAdmissionStatus::kProposed,
            "Proposed status has no proposal");
    store_.WriteQuery(result, LoopDisposition::kRejected, stats_.pose_version,
                      stats_.frame_count);
    ++stats_.rejected;
    return;
  }
  Require(options_.pose_graph_enabled,
          "Pose graph is disabled for this loop session");
  const auto& constraint = result.admission->proposal->loop.constraint;
  Require(
      constraint.rotation_sigma_rad == options_.loop_rotation_sigma_rad &&
          constraint.translation_sigma_m == options_.loop_translation_sigma_m,
      "Loop runner changed the fixed constraint noise");
  const auto current = CurrentInput(result.task.input.query_id);
  const auto checked =
      CheckLoopCommit(result.task, *result.admission, current, timestamps_);
  if (checked.status == LoopCommitStatus::kStale) {
    store_.WriteQuery(result, LoopDisposition::kStale, stats_.pose_version,
                      stats_.frame_count);
    ++stats_.stale;
    if (!pending_) QueueQuery(result.task.input.query_id);
    return;
  }
  auto next_loops = accepted_loops_;
  next_loops.push_back(result.admission->proposal->loop);
  store_.WritePrepared(result, *checked.poses);
  system_.ApplyLoopPoseUpdate(result.admission->proposal->update);
  // No allocation separates successful system publication from formal state.
  accepted_loops_.swap(next_loops);
  ++stats_.accepted;
  if (failure_) std::rethrow_exception(failure_);
  store_.WriteQuery(result, LoopDisposition::kAccepted, stats_.pose_version,
                    stats_.frame_count);
  if (stats_.accepted >= options_.max_loops) StopDispatch();
}

void LoopBackend::StopOnFailure(std::exception_ptr error) noexcept {
  if (!failure_) failure_ = std::move(error);
  system_.FailLoopBackend(failure_);
  pending_.reset();
  try {
    worker_.Stop();
  } catch (...) {
  }
  store_.WriteFailure(failure_);
}

[[noreturn]] void LoopBackend::Fail(std::exception_ptr error) {
  StopOnFailure(std::move(error));
  std::rethrow_exception(failure_);
}

void LoopBackend::Poll() {
  if (failure_) Fail(failure_);
  if (system_.session_error_) Fail(system_.session_error_);
  if (finished_) throw std::logic_error("Online loop backend has finished");
  try {
    Require(system_.GetKeyframeReadView().frame_count() == stats_.frame_count,
            "Loop observer did not receive committed frames");
    FlushEvents();
    if (auto result = worker_.Take()) Complete(std::move(*result));
    FlushEvents();
    if (!budget_stopped_ && pending_ && worker_.idle()) {
      auto view = system_.GetKeyframeReadView();
      Require(view.frame_count() == stats_.frame_count,
              "Unobserved committed frames");
      LoopTask task{stats_.submitted + 1, CurrentInput(*pending_), timestamps_,
                    std::move(view)};
      dispatch_time_ = StartTiming(system_.options_.stage_timing);
      worker_.Submit(std::move(task));
      ++stats_.submitted;
      stats_.peak_work_payloads = 1;
      pending_.reset();
    }
  } catch (...) {
    Fail(std::current_exception());
  }
}

void LoopBackend::Finish() {
  if (failure_) Fail(failure_);
  if (finished_) return;
  system_.FinishSession(this);
}

void LoopBackend::Drain() {
  system_.SealInput();
  for (;;) {
    Poll();
    if (worker_.idle() && !pending_) break;
    worker_.Wait();
  }
  worker_.Stop();
}

void LoopBackend::FinishJournal() {
  store_.Finish(stats_);
  finished_ = true;
}

}  // namespace livo_atlas
