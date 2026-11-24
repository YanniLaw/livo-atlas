#include "livo_atlas/core/loop_backend.h"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "core/online_submap_manager.h"
#include "livo_atlas/core/livo_system.h"
#include "pose_graph/pose_graph.h"

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
      system.GetPoseFrontier().version)
    throw std::logic_error("Loop backend requires a new unclaimed session");
  return options;
}

LoopBackend::LoopBackend(LivoSystem& system, LoopAdmissionOptions options)
    : LoopBackend(system, options, [closure = LoopClosure(options)](
                                       const auto& task) {
        return closure.Run(task);
      }) {}

LoopBackend::LoopBackend(LivoSystem& system, LoopAdmissionOptions options,
                         LoopTaskRunner runner)
    : system_(system),
      options_(ValidateSession(system, std::move(options))),
      store_(system.GetKeyframeReadView(), options_),
      graph_(std::make_unique<pose_graph::PoseGraph>(
          options_, std::move(runner), system_.options_.stage_timing)) {
  system_.ClaimLoopPoseUpdates(*this);
}

LoopBackend::~LoopBackend() { system_.ClearLoopBackend(*this); }

void LoopBackend::CheckInputCapacity() const {
  if (failure_) std::rethrow_exception(failure_);
  graph_->CheckInputCapacity();
}

void LoopBackend::Observe(const OnlineSubmapUpdate& update) noexcept {
  if (failure_) return;
  try {
    Require(static_cast<bool>(update.poses), "Missing observed pose snapshot");
    ObserveFrontier(update.keyframe.get(), update.revisions,
                    update.poses->frontier(), update.poses, {});
  } catch (...) {
    failure_ = std::current_exception();
  }
}

void LoopBackend::ObserveDelta(const OnlineSubmapDelta& update) noexcept {
  if (failure_) return;
  try {
    ObserveFrontier(update.keyframe.get(), update.revisions, update.frontier,
                    update.poses, update.appended_pose);
  } catch (...) {
    failure_ = std::current_exception();
  }
}

void LoopBackend::ObserveFrontier(
    const Keyframe* keyframe, const std::vector<OnlineSubmap>& revisions,
    const PoseFrontier& frontier,
    const std::shared_ptr<const PoseSnapshot>& poses,
    const std::optional<RigidTransform>& appended_pose) {
  const auto count = observed_.frame_count + (keyframe ? 1 : 0);
  Require(!finished_ && frontier.frame_count == count,
          "Observed keyframe frontier is not consecutive");
  if (frontier.version != observed_.version) {
    Require(publishing_ && !published_poses_ && poses && !keyframe &&
                revisions.empty() && frontier.version == observed_.version + 1,
            "Unexpected published loop poses");
    published_poses_ = poses;
  } else {
    pose_graph::Observation observation{{}, revisions, frontier, poses};
    if (keyframe) {
      Require(keyframe->id == observed_.frame_count &&
                  std::isfinite(keyframe->timestamp) &&
                  (!observed_.frame_count ||
                   keyframe->timestamp >= latest_timestamp_),
              "Observed raw keyframes are not consecutive and ordered");
      observation.keyframe = pose_graph::KeyframeObservation{
          keyframe->id, keyframe->timestamp, keyframe->odom_from_body,
          appended_pose};
    }
    graph_->AddObservation(std::move(observation));
    if (keyframe) latest_timestamp_ = keyframe->timestamp;
  }
  observed_ = frontier;
}

void LoopBackend::Complete(std::shared_ptr<const LoopTaskResult> result,
                           LoopDisposition disposition) {
  if (result->error) {
    try {
      store_.WriteQuery(*result, LoopDisposition::kFailed, observed_.version,
                        observed_.frame_count);
    } catch (...) {
    }
    std::rethrow_exception(result->error);
  }
  if (observed_.frame_count > options_.max_keyframes)
    disposition = LoopDisposition::kResourceLimit;
  std::shared_ptr<const PoseSnapshot> committed_poses;
  if (disposition == LoopDisposition::kAccepted) {
    const auto& update = result->admission->proposal->update;
    // The system may have committed more raw frames since graph acceptance.
    // Prepare the same prefix that the owner is about to publish.
    auto candidate = std::make_shared<const PoseSnapshot>(
        system_.Online().PreparePoseUpdate(update));
    store_.WritePrepared(*result, *candidate);
    publishing_ = true;
    system_.ApplyLoopPoseUpdate(update);
    publishing_ = false;
    ++published_loops_;
    if (failure_) std::rethrow_exception(failure_);
    Require(published_poses_ &&
                published_poses_->frontier() == candidate->frontier(),
            "Loop observer did not receive committed poses");
    published_poses_.reset();
    committed_poses = std::move(candidate);
  }
  store_.WriteQuery(*result, disposition, observed_.version,
                    observed_.frame_count);
  // The acknowledgement permits the next query to allocate its payload.
  result.reset();
  graph_->Acknowledge(disposition, std::move(committed_poses));
}

void LoopBackend::StopOnFailure(std::exception_ptr error) noexcept {
  if (!failure_) failure_ = std::move(error);
  system_.RecordFailure(failure_);
  try {
    graph_->Stop();
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
    auto view = system_.GetKeyframeReadView();
    Require(view.frame_count() == observed_.frame_count,
            "Loop observer did not receive committed frames");
    auto progress = graph_->TakeProgress();
    if (progress.failure) std::rethrow_exception(progress.failure);
    if (progress.coalesced) {
      const auto& skipped = *progress.coalesced;
      store_.WriteCoalesced(skipped.first, skipped.last, skipped.count);
    }
    if (progress.limit) {
      const auto& limit = *progress.limit;
      store_.WriteResourceLimit(limit.query_id, limit.frame_count);
    }
    if (progress.completion)
      Complete(std::move(progress.completion->result),
               progress.completion->disposition);
    graph_->SetReadView(std::move(view));
  } catch (...) {
    Fail(std::current_exception());
  }
}

LoopSessionStats LoopBackend::GetStats() const {
  auto stats = graph_->GetStats();
  // A published pose remains committed if its query report fails to persist.
  stats.completed += published_loops_ - stats.accepted;
  stats.accepted = published_loops_;
  stats.frame_count = observed_.frame_count;
  stats.pose_version = observed_.version;
  return stats;
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
    if (graph_->Drained()) break;
    graph_->Wait();
  }
  graph_->Stop();
}

void LoopBackend::FinishJournal() {
  store_.Finish(GetStats());
  finished_ = true;
}

}  // namespace livo_atlas
