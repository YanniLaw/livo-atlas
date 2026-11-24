#include "pose_graph/pose_graph.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "common/thread_pool.h"
#include "common/timing.h"
#include "livo_atlas/core/system_options.h"
#include "pose_graph/loop_commit.h"
#include "pose_graph/online_submap_archive.h"

namespace livo_atlas::pose_graph {
namespace {

constexpr int kMaxWorkItemsPerDrain = 16;

void Require(bool condition, const char* message) {
  if (!condition) throw std::invalid_argument(message);
}

}  // namespace

PoseGraph::PoseGraph(const SystemOptions& options)
    : keyframes_(options.exporter.directory / "keyframes",
                 options.frontend.calibration),
      groups_(options.submaps.max_keyframes),
      stage_timing_(options.stage_timing),
      thread_pool_(std::make_unique<ThreadPool>(1)) {
  if (options.max_pending_keyframes)
    options_.max_pending_keyframes = *options.max_pending_keyframes;
  if (!options.submaps.enabled) return;
  const auto directory = options.exporter.directory / "keyframes";
  if (options.submap_mode == SubmapMode::kOnline) {
    online_ = std::make_unique<OnlineSubmapArchive>(
        directory / "online_submaps", keyframes_,
        options.submaps.max_keyframes, options.online_submaps);
  } else {
    group_store_ = std::make_unique<SubmapStore>(
        directory / "submaps", keyframes_,
        options.submaps.max_keyframes);
  }
}

PoseGraph::~PoseGraph() { Stop(); }

KeyframeReadView PoseGraph::InitialReadView() const {
  return keyframes_.CaptureReadView();
}

void PoseGraph::EnableLoops(LoopAdmissionOptions options, LoopTaskRunner runner,
                            bool standalone) {
  // Registration precedes the first queued archive operation.
  Require(static_cast<bool>(runner), "Missing loop task runner");
  journal_ = std::make_unique<LoopClosureStore>(InitialReadView(), options);
  options_ = std::move(options);
  runner_ = std::move(runner);
  standalone_ = standalone;
}

void PoseGraph::Enqueue(WorkItem item, bool keyframe) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (failure_) std::rethrow_exception(failure_);
  if (stopping_) throw std::logic_error("Pose graph has stopped");
  if (keyframe && queued_keyframes_ >= options_.max_pending_keyframes)
    throw std::overflow_error("pose_graph.max_pending_keyframes exceeded");
  queue_.push_back(std::move(item));
  if (keyframe) {
    ++queued_keyframes_;
    peak_queued_keyframes_ =
        std::max(peak_queued_keyframes_, queued_keyframes_);
  }
  idle_ = false;
  ScheduleDrainLocked();
}

void PoseGraph::AddKeyframe(std::shared_ptr<const Keyframe> keyframe) {
  Enqueue({StartTiming(stage_timing_),
           [this, keyframe = std::move(keyframe)]() mutable {
             Append(std::move(keyframe));
             return WorkItem::Result::kDoNotRunOptimization;
           }},
          /*keyframe=*/true);
}

std::shared_ptr<const ControlReceipt> PoseGraph::EnqueueControl(
    std::function<std::exception_ptr()> task) {
  auto receipt = std::make_shared<ControlReceipt>();
  Enqueue({{}, [this, task = std::move(task), receipt] {
             auto rejection = task();
             {
               std::lock_guard<std::mutex> lock(mutex_);
               receipt->rejection_ = std::move(rejection);
               receipt->done_ = true;
             }
             condition_.notify_all();
             return WorkItem::Result::kDoNotRunOptimization;
           }},
          /*keyframe=*/false);
  return receipt;
}

std::shared_ptr<const ControlReceipt> PoseGraph::SealInput() {
  return EnqueueControl([this]() -> std::exception_ptr {
    if (online_) {
      auto view = keyframes_.CaptureReadView();
      if (auto update = online_->SealInput()) {
        RecordMembership(update->revisions);
        Publish({std::move(view), std::move(update), online_->correction()});
      }
    } else if (group_store_) {
      if (const auto tail = groups_.Finish()) group_store_->Append(*tail);
    }
    return {};
  });
}

std::shared_ptr<const ControlReceipt> PoseGraph::ApplyPoseUpdate(
    PoseUpdate update) {
  return EnqueueControl(
      [this, update = std::move(update)]() -> std::exception_ptr {
        const auto start = StartTiming(stage_timing_);
        std::shared_ptr<const PoseSnapshot> candidate;
        try {
          candidate = std::make_shared<const PoseSnapshot>(
              online_->PreparePoseUpdate(update));
        } catch (const std::invalid_argument&) {
          // A stale or malformed external proposal leaves the writer open.
          return std::current_exception();
        } catch (const std::overflow_error&) {
          return std::current_exception();
        }
        auto view = keyframes_.CaptureReadView();
        auto delta = online_->WritePoses(std::move(candidate));
        Publish({std::move(view), std::move(delta), online_->correction()});
        LogStageTiming("pose_commit", start);
        return {};
      });
}

std::shared_ptr<const ControlReceipt> PoseGraph::FinishArchive() {
  return EnqueueControl([this]() -> std::exception_ptr {
    if (journal_ && !standalone_abandoned_) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        stats_.peak_queued_keyframes = peak_queued_keyframes_;
      }
      journal_->Finish(stats_);
    }
    return {};
  });
}

void PoseGraph::ScheduleDrainLocked() {
  if (drain_scheduled_ || awaiting_delivery_ || stopping_ || failure_) return;
  auto task = std::make_unique<Task>();
  task->SetWorkItem([this] { Drain(); });
  thread_pool_->Schedule(std::move(task));
  drain_scheduled_ = true;
  idle_ = false;
}

void PoseGraph::Drain() noexcept {
  try {
    for (int count = 0; count < kMaxWorkItemsPerDrain; ++count) {
      WorkItem item;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (standalone_abandoned_) {
          pending_.reset();
          active_query_.reset();
        }
        if (awaiting_delivery_ || stopping_ || failure_) break;
        if (!query_report_) {
          if (queue_.empty()) break;
          item = std::move(queue_.front());
          queue_.pop_front();
        }
      }
      if (item.task) {
        LogStageTiming("pose_graph_queue_wait", item.time);
        item.task();
      } else {
        WriteQueryReport();
      }
      WriteLoopEvents();
    }
    Dispatch();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      published_stats_ = stats_;
      drain_scheduled_ = false;
      if (standalone_abandoned_) {
        pending_.reset();
        active_query_.reset();
      }
      // A delivery acknowledgement or first standalone poll can arrive
      // after Dispatch checked its conditions and before this drain exits.
      if (!queue_.empty() || query_report_ || CanDispatchLocked())
        ScheduleDrainLocked();
      idle_ = !drain_scheduled_ && !awaiting_delivery_ && !active_query_ &&
              !pending_;
    }
    condition_.notify_all();
  } catch (...) {
    Cancel(std::current_exception());
  }
}

void PoseGraph::Publish(ArchiveCommit commit, bool appended_keyframe) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (appended_keyframe) --queued_keyframes_;
    publication_.emplace(std::move(commit));
    awaiting_delivery_ = true;
    published_stats_ = stats_;
  }
  condition_.notify_all();
}

ArchiveProgress PoseGraph::TakeProgress() {
  std::lock_guard<std::mutex> lock(mutex_);
  return {std::exchange(publication_, std::nullopt), failure_};
}

void PoseGraph::AcknowledgeCommit() {
  std::lock_guard<std::mutex> lock(mutex_);
  awaiting_delivery_ = false;
  ScheduleDrainLocked();
}

std::optional<std::exception_ptr> PoseGraph::CheckControl(
    const ControlReceipt& receipt) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!receipt.done_) return std::nullopt;
  return receipt.rejection_;
}

LoopSessionStats PoseGraph::GetStats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto stats = published_stats_;
  stats.peak_queued_keyframes = peak_queued_keyframes_;
  return stats;
}

bool PoseGraph::Drained() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return idle_ && !failure_;
}

void PoseGraph::Wait(const ControlReceipt* receipt, bool poll_standalone) {
  std::unique_lock<std::mutex> lock(mutex_);
  condition_.wait(lock, [this, receipt, poll_standalone] {
    return idle_ || stopping_ || failure_ || publication_ ||
           (receipt && receipt->done_) ||
           (poll_standalone && standalone_result_);
  });
}

void PoseGraph::RethrowFailure() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (failure_) std::rethrow_exception(failure_);
}

std::exception_ptr PoseGraph::Cancel(std::exception_ptr error) noexcept {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!failure_) failure_ = std::move(error);
    error = failure_;
  }
  condition_.notify_all();
  return error;
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
  // Finish can still take a durable publication after a terminal error.
  queue_.clear();
  standalone_result_.reset();
  query_report_.reset();
  if (journal_ && failure_ && !failure_written_) {
    journal_->WriteFailure(failure_);
    failure_written_ = true;
  }
}

void PoseGraph::Append(std::shared_ptr<const Keyframe> keyframe) {
  if (journal_) {
    Require(!stats_.frame_count || keyframe->timestamp >= latest_timestamp_,
            "Observed raw keyframes are not ordered");
    latest_timestamp_ = keyframe->timestamp;
    if (!budget_stopped_) timestamps_.push_back(keyframe->timestamp);
  }
  const auto write_start = StartTiming(stage_timing_);
  keyframes_.Append(*keyframe);
  auto view = keyframes_.CaptureReadView();
  LogStageTiming("keyframe_write", write_start);
  std::optional<OnlineSubmapDelta> update;
  if (online_) {
    update = online_->Append(std::move(keyframe));
  } else if (group_store_) {
    if (const auto group = groups_.AddKeyframe(keyframe->id))
      group_store_->Append(*group);
  }
  stats_.frame_count = view.frame_count();
  const auto correction = online_ ? online_->correction() : std::nullopt;
  if (update) {
    // These bounded revisions also update the loop candidate index.
    RecordMembership(update->revisions);
  }
  Publish({std::move(view), std::move(update), correction},
          /*appended_keyframe=*/true);
}

void PoseGraph::RecordMembership(const std::vector<OnlineSubmap>& revisions) {
  if (!journal_) return;
  for (const auto& group : revisions) {
    if (group.state == OnlineSubmapState::kSealed)
      latest_query_ = group.primary.id;
  }
  if (stats_.frame_count > options_.max_keyframes) StopDispatch();
  if (budget_stopped_) return;
  for (const auto& group : revisions) {
    const auto id = group.primary.id;
    if (id == submaps_.size()) {
      submaps_.push_back(group);
    } else {
      submaps_.back() = group;
    }
    if (group.state == OnlineSubmapState::kSealed) QueueQuery(id);
  }
}

void PoseGraph::QueueQuery(std::uint64_t query_id) {
  if (budget_stopped_) return;
  if (pending_ && *pending_ != query_id) {
    if (!coalesced_) coalesced_ = CoalescedQueries{*pending_, *pending_, 0};
    coalesced_->first = std::min(coalesced_->first, *pending_);
    coalesced_->last = std::max(coalesced_->last, *pending_);
    ++coalesced_->count;
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
  resource_limit_query_ = latest_query_;
}

void PoseGraph::WriteLoopEvents() {
  if (coalesced_) {
    journal_->WriteCoalesced(coalesced_->first, coalesced_->last,
                             coalesced_->count);
    coalesced_.reset();
  }
  if (resource_limit_query_) {
    journal_->WriteResourceLimit(*resource_limit_query_, stats_.frame_count);
    resource_limit_query_.reset();
  }
}

LoopAdmissionInput PoseGraph::CurrentInput(std::uint64_t query_id) const {
  LoopAdmissionInput input;
  input.query_id = query_id;
  input.poses = online_->MaterializePoses();
  input.raw_poses = online_->raw_poses();
  input.accepted_loops = accepted_loops_;
  for (const auto& group : submaps_) {
    if (group.state != OnlineSubmapState::kSealed) break;
    input.submaps.push_back(
        {group, timestamps_.at(group.primary.keyframe_ids.front()),
         timestamps_.at(group.primary.keyframe_ids.back()),
         input.poses.poses.at(group.primary.anchor_keyframe_id)
             .parent_from_body(),
         input.poses.version, input.poses.frame_count,
         input.poses.optimized_frame_count});
  }
  Require(query_id < input.submaps.size(), "Unknown pending loop query");
  return input;
}

bool PoseGraph::CanDispatchLocked() const {
  return pending_ && !active_query_ && !query_report_ && !stopping_ &&
         !failure_ && !awaiting_delivery_ && queue_.empty() &&
         !standalone_abandoned_ && (!standalone_ || standalone_polling_);
}

void PoseGraph::Dispatch() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!CanDispatchLocked()) return;
  }
  LoopTask query{stats_.submitted + 1, CurrentInput(*pending_), timestamps_,
                 keyframes_.CaptureReadView()};
  auto payload = std::make_shared<LoopTaskResult>(
      LoopTaskResult{std::move(query), {}, {}});
  auto task = std::make_unique<Task>();
  task->SetWorkItem([this, payload] { RunQuery(payload); });
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!CanDispatchLocked()) return;
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
    if (stopping_ || failure_ || standalone_abandoned_) return;
  }
  try {
    payload->admission = runner_(payload->task);
  } catch (...) {
    payload->error = std::current_exception();
  }
  try {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_ || failure_ || standalone_abandoned_) return;
      if (standalone_) {
        standalone_result_ = std::move(payload);
      } else {
        // The completion follows earlier frames even on a full input queue.
        queue_.push_back(CompletionWork(std::move(payload)));
        ScheduleDrainLocked();
      }
    }
    condition_.notify_all();
  } catch (...) {
    Cancel(std::current_exception());
  }
}

WorkItem PoseGraph::CompletionWork(
    std::shared_ptr<const LoopTaskResult> result) {
  return {{}, [this, result = std::move(result)]() mutable {
            Complete(std::move(result));
            return WorkItem::Result::kDoNotRunOptimization;
          }};
}

void PoseGraph::PollStandalone() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (failure_) std::rethrow_exception(failure_);
  if (standalone_result_) {
    queue_.push_back(CompletionWork(std::move(standalone_result_)));
    ScheduleDrainLocked();
  } else if (!standalone_polling_) {
    standalone_polling_ = true;
    ScheduleDrainLocked();
  }
}

void PoseGraph::AbandonStandalone() noexcept {
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    standalone_abandoned_ = true;
    standalone_result_.reset();
    ScheduleDrainLocked();
  } catch (...) {
    Cancel(std::current_exception());
  }
}

void PoseGraph::Complete(std::shared_ptr<const LoopTaskResult> result) {
  if (!active_query_) return;
  const auto disposition = Classify(*result);
  if (disposition == LoopDisposition::kFailed) {
    try {
      journal_->WriteQuery(*result, disposition, stats_.pose_version,
                           stats_.frame_count);
    } catch (...) {
      // The original worker exception owns terminal failure.
    }
    std::rethrow_exception(result->error);
  }
  if (disposition == LoopDisposition::kAccepted) {
    const auto start = StartTiming(stage_timing_);
    auto view = keyframes_.CaptureReadView();
    auto candidate = std::make_shared<const PoseSnapshot>(
        online_->PreparePoseUpdate(result->admission->proposal->update));
    journal_->WritePrepared(*result, *candidate);
    auto delta = online_->WritePoses(std::move(candidate));
    stats_.pose_version = delta.frontier.version;
    RecordDisposition(disposition);
    // Keep the accepted version available if delivery or reporting fails.
    query_report_ = QueryReport{std::move(result), disposition};
    Publish({std::move(view), std::move(delta), online_->correction()});
    LogStageTiming("pose_commit", start);
  } else {
    journal_->WriteQuery(*result, disposition, stats_.pose_version,
                         stats_.frame_count);
    RecordDisposition(disposition);
  }
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

void PoseGraph::RecordDisposition(LoopDisposition disposition) {
  switch (disposition) {
    case LoopDisposition::kAccepted:
      accepted_loops_.swap(next_loops_);
      ++stats_.accepted;
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
      throw std::logic_error("Cannot record a failed loop task");
  }
  next_loops_.clear();
  ++stats_.completed;
  active_query_.reset();
}

void PoseGraph::WriteQueryReport() {
  journal_->WriteQuery(*query_report_->result, query_report_->disposition,
                       stats_.pose_version, stats_.frame_count);
  query_report_.reset();
  if (stats_.accepted >= options_.max_loops) StopDispatch();
}

}  // namespace livo_atlas::pose_graph
