#include "pose_graph/pose_graph.h"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

#include "common/thread_pool.h"
#include "common/timing.h"
#include "pose_graph/loop_commit.h"

namespace livo_atlas::pose_graph {
namespace {

constexpr int kMaxWorkItemsPerDrain = 16;

void Require(bool condition, const char* message) {
  if (!condition) throw std::invalid_argument(message);
}

}  // namespace

PoseGraph::PoseGraph(LoopAdmissionOptions options, LoopTaskRunner runner,
                     bool stage_timing)
    : options_(std::move(options)),
      runner_(std::move(runner)),
      stage_timing_(stage_timing),
      thread_pool_(std::make_unique<ThreadPool>(1)) {
  Require(static_cast<bool>(runner_), "Missing loop task runner");
}

PoseGraph::~PoseGraph() { Stop(); }

void PoseGraph::CheckInputCapacity() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (progress_.failure) std::rethrow_exception(progress_.failure);
  if (stopping_) throw std::logic_error("Pose graph has stopped");
  if (queued_keyframes_ >= options_.max_pending_keyframes)
    throw std::overflow_error("pose_graph.max_pending_keyframes exceeded");
}

void PoseGraph::Enqueue(WorkItem item, bool keyframe) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (progress_.failure) std::rethrow_exception(progress_.failure);
  if (stopping_) return;
  if (keyframe && queued_keyframes_ >= options_.max_pending_keyframes)
    throw std::overflow_error("pose_graph.max_pending_keyframes exceeded");
  queue_.push_back(std::move(item));
  if (keyframe) {
    ++queued_keyframes_;
    peak_queued_keyframes_ =
        std::max(peak_queued_keyframes_, queued_keyframes_);
  }
  ScheduleDrainLocked();
}

void PoseGraph::AddObservation(Observation observation) {
  const bool keyframe = observation.keyframe.has_value();
  Enqueue({StartTiming(stage_timing_),
           [this, observation = std::move(observation)] {
             if (observation.keyframe) {
               std::lock_guard<std::mutex> lock(mutex_);
               --queued_keyframes_;
             }
             RecordObservation(observation);
             return WorkItem::Result::kDoNotRunOptimization;
           }},
          keyframe);
}

void PoseGraph::SetReadView(KeyframeReadView view) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (progress_.failure || stopping_ ||
      (read_view_ && read_view_->frame_count() == view.frame_count()))
    return;
  read_view_ = std::move(view);
  view_changed_ = true;
  ScheduleDrainLocked();
}

void PoseGraph::ScheduleDrainLocked() {
  if (drain_scheduled_ || stopping_ || progress_.failure) return;
  auto task = std::make_unique<Task>();
  task->SetWorkItem([this] { Drain(); });
  thread_pool_->Schedule(std::move(task));
  drain_scheduled_ = true;
  idle_ = false;
}

void PoseGraph::Drain() noexcept {
  try {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      view_changed_ = false;
    }
    for (int count = 0; count < kMaxWorkItemsPerDrain; ++count) {
      WorkItem item;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty() || stopping_ || progress_.failure) break;
        item = std::move(queue_.front());
        queue_.pop_front();
      }
      LogStageTiming("pose_graph_queue_wait", item.time);
      item.task();
    }
    Dispatch();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      published_stats_ = stats_;
      drain_scheduled_ = false;
      if (!queue_.empty() || view_changed_) ScheduleDrainLocked();
      idle_ = !drain_scheduled_ && !active_query_ && !pending_;
    }
    condition_.notify_all();
  } catch (...) {
    RecordFailure(std::current_exception());
  }
}

void PoseGraph::RecordFailure(std::exception_ptr error) noexcept {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!progress_.failure) progress_.failure = std::move(error);
  }
  condition_.notify_all();
}

Progress PoseGraph::TakeProgress() {
  std::lock_guard<std::mutex> lock(mutex_);
  return {std::exchange(progress_.coalesced, std::nullopt),
          std::exchange(progress_.limit, std::nullopt),
          std::exchange(progress_.completion, std::nullopt), progress_.failure};
}

LoopSessionStats PoseGraph::GetStats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto stats = published_stats_;
  stats.peak_queued_keyframes = peak_queued_keyframes_;
  return stats;
}

bool PoseGraph::Drained() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return idle_ && !progress_.coalesced && !progress_.limit &&
         !progress_.completion && !progress_.failure;
}

void PoseGraph::Wait() {
  std::unique_lock<std::mutex> lock(mutex_);
  condition_.wait(lock, [this] {
    return idle_ || stopping_ || progress_.failure || progress_.coalesced ||
           progress_.limit || progress_.completion;
  });
}

void PoseGraph::Stop() {
  std::unique_ptr<ThreadPool> pool;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
    pool = std::move(thread_pool_);
  }
  condition_.notify_all();
  pool.reset();
}

void PoseGraph::RecordObservation(const Observation& observation) {
  const auto& frontier = observation.frontier;
  const auto& keyframe = observation.keyframe;
  const auto count = stats_.frame_count + (keyframe ? 1 : 0);
  Require(frontier.frame_count == count &&
              frontier.version == stats_.pose_version,
          "Pose graph observations are not consecutive");
  stats_.frame_count = count;
  for (const auto& group : observation.revisions) {
    if (group.state == OnlineSubmapState::kSealed)
      latest_query_ = group.primary.id;
  }
  if (count > options_.max_keyframes || stats_.accepted >= options_.max_loops)
    StopDispatch();
  if (budget_stopped_) return;

  if (observation.poses) {
    ValidatePoseSnapshot(*observation.poses);
    Require(observation.poses->frontier() == frontier,
            "Observed pose snapshot does not match its frontier");
    poses_ = *observation.poses;
  } else {
    auto expected = poses_.frontier();
    expected.frame_count = count;
    Require(frontier == expected, "Pose graph append changed the version");
    if (keyframe) {
      Require(keyframe->parent_from_body.has_value(),
              "Missing resolved keyframe pose");
      poses_.poses.push_back({keyframe->id, *keyframe->parent_from_body});
      poses_.frame_count = count;
    }
  }
  if (keyframe) {
    raw_poses_.push_back(keyframe->odom_from_body);
    timestamps_.push_back(keyframe->timestamp);
  }
  for (const auto& group : observation.revisions) RecordMembership(group);
}

void PoseGraph::RecordMembership(const OnlineSubmap& group) {
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

void PoseGraph::QueueQuery(std::uint64_t query_id) {
  if (budget_stopped_) return;
  if (pending_ && *pending_ != query_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& skipped = progress_.coalesced;
    if (!skipped) skipped = CoalescedQueries{*pending_, *pending_, 0};
    skipped->first = std::min(skipped->first, *pending_);
    skipped->last = std::max(skipped->last, *pending_);
    ++skipped->count;
    ++stats_.skipped;
  }
  pending_ = query_id;
  stats_.peak_pending = 1;
}

void PoseGraph::StopDispatch() {
  if (budget_stopped_) return;
  budget_stopped_ = true;
  pending_.reset();
  ++stats_.resource_limited;
  std::lock_guard<std::mutex> lock(mutex_);
  progress_.limit = ResourceLimit{latest_query_, stats_.frame_count};
}

LoopAdmissionInput PoseGraph::CurrentInput(std::uint64_t query_id) const {
  Require(poses_.frame_count == stats_.frame_count &&
              poses_.version == stats_.pose_version &&
              raw_poses_.size() == stats_.frame_count &&
              timestamps_.size() == stats_.frame_count,
          "Pose graph is missing committed frames");
  LoopAdmissionInput input;
  input.query_id = query_id;
  input.poses = poses_;
  input.raw_poses = raw_poses_;
  input.accepted_loops = accepted_loops_;
  for (const auto& group : submaps_) {
    if (group.state != OnlineSubmapState::kSealed) break;
    input.submaps.push_back(
        {group, timestamps_.at(group.primary.keyframe_ids.front()),
         timestamps_.at(group.primary.keyframe_ids.back()),
         poses_.poses.at(group.primary.anchor_keyframe_id).map_from_body,
         poses_.version, poses_.frame_count, poses_.optimized_frame_count});
  }
  Require(query_id < input.submaps.size(), "Unknown pending loop query");
  return input;
}

void PoseGraph::Dispatch() {
  if (!pending_ || active_query_) return;
  std::optional<KeyframeReadView> view;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || !queue_.empty() || !read_view_ ||
        read_view_->frame_count() != stats_.frame_count)
      return;
    view = read_view_;
  }
  LoopTask query{stats_.submitted + 1, CurrentInput(*pending_), timestamps_,
                 std::move(*view)};
  auto payload = std::make_shared<LoopTaskResult>(
      LoopTaskResult{std::move(query), {}, {}});
  auto task = std::make_unique<Task>();
  task->SetWorkItem([this, payload] { RunQuery(payload); });
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_) return;
    thread_pool_->Schedule(std::move(task));
  }
  dispatch_time_ = StartTiming(stage_timing_);
  active_query_ = *pending_;
  pending_.reset();
  ++stats_.submitted;
  stats_.peak_work_payloads = 1;
}

void PoseGraph::RunQuery(std::shared_ptr<LoopTaskResult> payload) noexcept {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || progress_.failure) return;
  }
  try {
    payload->admission = runner_(payload->task);
  } catch (...) {
    payload->error = std::current_exception();
  }
  try {
    // A completion follows already queued observations even on a full queue.
    Enqueue({{},
             [this, payload = std::move(payload)] {
               Complete(payload);
               return WorkItem::Result::kDoNotRunOptimization;
             }},
            /*keyframe=*/false);
  } catch (...) {
    RecordFailure(std::current_exception());
  }
}

void PoseGraph::Complete(std::shared_ptr<const LoopTaskResult> result) {
  const auto disposition = Classify(*result);
  std::lock_guard<std::mutex> lock(mutex_);
  Require(active_query_ && !progress_.completion,
          "Pose graph has no slot for a loop completion");
  progress_.completion = Completion{std::move(result), disposition};
}

LoopDisposition PoseGraph::Classify(const LoopTaskResult& result) {
  LogStageTiming(
      options_.pose_graph_enabled ? "loop_optimization_result_latency"
                                  : "loop_detection_result_latency",
      dispatch_time_);
  if (result.error) return LoopDisposition::kFailed;
  Require(result.admission.has_value(),
          "Worker returned no admission or exception");
  if (stage_timing_) {
    for (const auto& candidate : result.admission->candidates)
      LogStageSeconds("loop_registration",
                      candidate.verification.elapsed_seconds);
    if (result.admission->trial) {
      LogStageSeconds("loop_graph_build",
                      result.admission->trial->graph_build_seconds);
      LogStageSeconds("loop_solve", result.admission->trial->solve_seconds);
    }
  }
  if (budget_stopped_ ||
      result.admission->status == LoopAdmissionStatus::kResourceLimit)
    return LoopDisposition::kResourceLimit;
  if (result.admission->status == LoopAdmissionStatus::kDetected) {
    Require(!options_.pose_graph_enabled &&
                result.admission->detected_target_id &&
                !result.admission->proposal,
            "Invalid detection-only result");
    return LoopDisposition::kDetected;
  }
  if (!result.admission->proposal) {
    Require(result.admission->status != LoopAdmissionStatus::kProposed,
            "Proposed status has no proposal");
    return LoopDisposition::kRejected;
  }
  Require(options_.pose_graph_enabled,
          "Pose graph is disabled for this loop session");
  const auto& proposal = *result.admission->proposal;
  Require(proposal.loop.constraint.rotation_sigma_rad ==
                  options_.loop_rotation_sigma_rad &&
              proposal.loop.constraint.translation_sigma_m ==
                  options_.loop_translation_sigma_m,
          "Loop runner changed the fixed constraint noise");
  const auto current = CurrentInput(result.task.input.query_id);
  if (ValidateLoopCommit(result.task, *result.admission, current,
                         timestamps_) == LoopCommitStatus::kStale)
    return LoopDisposition::kStale;
  next_loops_ = accepted_loops_;
  next_loops_.push_back(proposal.loop);
  return LoopDisposition::kAccepted;
}

void PoseGraph::Acknowledge(LoopDisposition disposition,
                            std::shared_ptr<const PoseSnapshot> poses) {
  Enqueue({{},
           [this, disposition, poses = std::move(poses)] {
             ApplyAcknowledgement(disposition, poses);
             return WorkItem::Result::kDoNotRunOptimization;
           }},
          /*keyframe=*/false);
}

void PoseGraph::ApplyAcknowledgement(
    LoopDisposition disposition,
    const std::shared_ptr<const PoseSnapshot>& poses) {
  Require(active_query_.has_value(), "Unexpected pose graph acknowledgement");
  switch (disposition) {
    case LoopDisposition::kAccepted:
      Require(poses && poses->frame_count == stats_.frame_count &&
                  poses->version == stats_.pose_version + 1 &&
                  next_loops_.size() == accepted_loops_.size() + 1,
              "Published loop poses do not match the graph frontier");
      poses_ = *poses;
      stats_.pose_version = poses->version;
      accepted_loops_.swap(next_loops_);
      ++stats_.accepted;
      if (stats_.accepted >= options_.max_loops) StopDispatch();
      break;
    case LoopDisposition::kStale:
      ++stats_.stale;
      if (!pending_) QueueQuery(*active_query_);
      break;
    case LoopDisposition::kResourceLimit:
      ++stats_.resource_limited;
      break;
    case LoopDisposition::kRejected:
      ++stats_.rejected;
      break;
    case LoopDisposition::kDetected:
      ++stats_.detected;
      break;
    case LoopDisposition::kFailed:
      throw std::logic_error("Cannot acknowledge a failed loop task");
  }
  next_loops_.clear();
  ++stats_.completed;
  active_query_.reset();
}

}  // namespace livo_atlas::pose_graph
