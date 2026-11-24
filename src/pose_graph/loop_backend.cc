#include "livo_atlas/core/loop_backend.h"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "livo_atlas/core/livo_system.h"
#include "pose_graph/pose_graph.h"

namespace livo_atlas {
namespace {

void Require(bool condition, const char* message) {
  if (!condition) throw std::invalid_argument(message);
}

}  // namespace

LoopBackend::LoopBackend(LivoSystem& system, LoopAdmissionOptions options)
    : LoopBackend(system, options,
                  [closure = LoopClosure(options)](const LoopTask& task) {
                    return closure.Run(task);
                  }) {}

LoopBackend::LoopBackend(LivoSystem& system, LoopAdmissionOptions options,
                         LoopTaskRunner runner)
    : system_(system) {
  ValidateLoopAdmissionOptions(options);
  const auto& config = system_.options_;
  Require(config.keyframes.enabled && config.submaps.enabled &&
              config.submap_mode == SubmapMode::kOnline &&
              config.submaps.max_keyframes <=
                  static_cast<int>(kMaxGeometryKeyframes),
          "Online loops require keyframes and online submaps with at most 64 "
          "members");
  system_.EnableLoops(std::move(options), std::move(runner), this);
}

LoopBackend::~LoopBackend() { system_.ClearLoopBackend(*this); }

void LoopBackend::Observe(const OnlineSubmapUpdate& update) noexcept {
  if (failure_) return;
  try {
    Require(static_cast<bool>(update.poses), "Missing observed pose snapshot");
    const auto frontier = update.poses->frontier();
    const auto count = observed_.frame_count + (update.keyframe ? 1 : 0);
    Require(!finished_ && frontier == system_.GetPoseFrontier() &&
                frontier.frame_count == count &&
                update.poses->poses.size() == count,
            "Observed keyframe frontier is not consecutive");
    if (frontier.version != observed_.version) {
      Require(!update.keyframe && update.revisions.empty() &&
                  frontier.version == observed_.version + 1,
              "Unexpected published loop poses");
    }
    if (update.keyframe) {
      Require(update.keyframe->id == observed_.frame_count &&
                  std::isfinite(update.keyframe->timestamp) &&
                  (!observed_.frame_count ||
                   update.keyframe->timestamp >= latest_timestamp_),
              "Observed raw keyframes are not consecutive and ordered");
      latest_timestamp_ = update.keyframe->timestamp;
    }
    observed_ = frontier;
  } catch (...) {
    failure_ = std::current_exception();
    system_.RecordFailure(failure_);
  }
}

void LoopBackend::Poll() {
  if (finished_) throw std::logic_error("Online loop backend has finished");
  try {
    system_.PumpArchive();
    Require(observed_ == system_.GetPoseFrontier(),
            "Loop observer did not receive committed frames");
    system_.pose_graph_->PollStandalone();
  } catch (...) {
    system_.RecordFailure(std::current_exception());
    std::rethrow_exception(system_.session_error_);
  }
}

LoopSessionStats LoopBackend::GetStats() const { return system_.LoopStats(); }

void LoopBackend::Finish() {
  if (finished_ && system_.IsFinished()) return;
  system_.FinishSession(this);
  finished_ = true;
}

}  // namespace livo_atlas
