#include "livo_atlas/core/trajectory_output.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <utility>

#include "livo_atlas/common/logging.h"
#include "livo_atlas/core/livo_system.h"

namespace livo_atlas {
namespace {

std::string ProducerId() {
  std::random_device random;
  std::string id;
  id.reserve(32);
  constexpr char kHex[] = "0123456789abcdef";
  for (int i = 0; i < 4; ++i) {
    const std::uint32_t value = random();
    for (int shift = 28; shift >= 0; shift -= 4)
      id.push_back(kHex[(value >> shift) & 15]);
  }
  return id;
}

std::string ErrorText(std::exception_ptr error) {
  try {
    std::rethrow_exception(error);
  } catch (const std::exception& exception) {
    return std::string(exception.what()).substr(0, 1024);
  } catch (...) {
    return "Unknown trajectory output failure";
  }
}

void CheckHeader(const PoseFrontier& poses) {
  const bool projected =
      poses.tail_policy == PoseTailPolicy::kProjectedLastOptimizedAnchor;
  if ((poses.tail_policy != PoseTailPolicy::kLegacyLastOptimizedAnchor &&
       !projected) ||
      (poses.version == 0
           ? poses.base_version != 0 || poses.optimized_frame_count != 0 ||
                 poses.parent_frame != PoseFrame::kOdom || projected
           : poses.base_version != poses.version - 1 ||
                 !poses.optimized_frame_count ||
                 poses.optimized_frame_count > poses.frame_count ||
                 poses.parent_frame != PoseFrame::kMap || !projected))
    throw std::invalid_argument("Invalid live trajectory source header");
}

double Seconds(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
      .count();
}

}  // namespace

void ValidateTrajectoryOutputOptions(const TrajectoryOutputOptions& options) {
  if (!options.enabled) return;
  if (!std::isfinite(options.max_frequency_hz) ||
      options.max_frequency_hz <= 0 || options.max_frequency_hz > 10 ||
      options.max_keyframes < 1 || options.max_keyframes > 100000)
    throw std::invalid_argument(
        "Trajectory output requires frequency in (0,10] Hz and "
        "max_keyframes in [1,100000]");
}

TrajectoryOutput::TrajectoryOutput(LivoSystem& system,
                                   TrajectoryOutputOptions options,
                                   Consumer consumer)
    : system_(system), options_(options), consumer_(std::move(consumer)) {
  ValidateTrajectoryOutputOptions(options_);
  if (!options_.enabled) return;
  if (!consumer_) throw std::invalid_argument("Missing trajectory consumer");
  producer_id_ = ProducerId();
  // Finish all potentially throwing setup before starting an owned thread.
  ObserveFrontier(system_.GetPoseFrontier(), {});
  thread_ = std::thread(&TrajectoryOutput::Run, this);
}

TrajectoryOutput::~TrajectoryOutput() { Stop(); }

void TrajectoryOutput::Observe(const OnlineSubmapUpdate& update) noexcept {
  const auto frontier =
      update.poses ? std::optional<PoseFrontier>(update.poses->frontier())
                   : std::nullopt;
  ObserveFrontier(frontier, update.poses);
}

void TrajectoryOutput::ObserveDelta(const OnlineSubmapDelta& update) noexcept {
  ObserveFrontier(update.frontier, update.poses);
}

void TrajectoryOutput::ObserveFrontier(
    std::optional<PoseFrontier> frontier,
    const std::shared_ptr<const PoseSnapshot>& snapshot) noexcept {
  if (!options_.enabled || finished_ || terminal_pending_) return;
  const auto start = Clock::now();
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    if (failed_) return;
    if (!frontier) throw std::invalid_argument("Missing trajectory poses");
    const auto& poses = *frontier;
    CheckHeader(poses);
    if (snapshot && (snapshot->poses.size() != poses.frame_count ||
                     !(snapshot->frontier() == poses))) {
      throw std::invalid_argument("Invalid live trajectory source header");
    }
    if (have_source_) {
      if (poses.version < latest_header_.pose_version ||
          poses.frame_count < latest_header_.frame_count ||
          poses.optimized_frame_count < latest_header_.optimized_frame_count ||
          (poses.version == latest_header_.pose_version &&
           poses.optimized_frame_count != latest_header_.optimized_frame_count))
        throw std::invalid_argument("Trajectory source moved backwards");
      if (poses.version == latest_header_.pose_version &&
          poses.frame_count == latest_header_.frame_count) {
        return;
      }
    }
    if (revision_ == std::numeric_limits<std::uint64_t>::max())
      throw std::overflow_error("Trajectory revision exhausted");
    TrajectorySnapshot header;
    header.producer_id = producer_id_;
    header.revision = ++revision_;
    header.pose_version = poses.version;
    header.frame_count = poses.frame_count;
    header.optimized_frame_count = poses.optimized_frame_count;
    header.parent_frame = poses.parent_frame;
    const bool limited =
        poses.frame_count > static_cast<std::uint64_t>(options_.max_keyframes);
    if (limited) {
      header.status = TrajectoryStatus::kResourceLimited;
      header.error = "Trajectory exceeds configured max_keyframes";
    }
    if (pending_) ++stats_.coalesced;
    pending_ = Observed{header, limited ? nullptr : snapshot, Clock::now()};
    latest_header_ = std::move(header);
    have_source_ = true;
    ++stats_.observed;
    stats_.peak_pending = 1;
    stats_.max_observe_seconds =
        std::max(stats_.max_observe_seconds, Seconds(start));
  } catch (...) {
    // Preserve the exception without allocating inside a noexcept observer.
    observe_error_ = std::current_exception();
    terminal_pending_ = true;
    if (pending_) pending_->poses.reset();
  }
}

void TrajectoryOutput::Poll() { Poll(false); }

void TrajectoryOutput::Poll(bool force) {
  if (!options_.enabled || finished_) return;
  const auto start = Clock::now();
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    if (failed_) {
      pending_.reset();
      return;
    }
    if (busy_) return;
    if (!force && have_delivery_ &&
        std::chrono::duration<double>(Clock::now() - last_delivery_).count() <
            1 / options_.max_frequency_hz)
      return;
    if (observe_error_) {
      if (revision_ == std::numeric_limits<std::uint64_t>::max())
        throw std::overflow_error("Trajectory revision exhausted");
      auto header = latest_header_;
      header.producer_id = producer_id_;
      header.revision = ++revision_;
      header.status = TrajectoryStatus::kError;
      header.error = ErrorText(observe_error_);
      pending_ = Observed{std::move(header), {}, Clock::now()};
      observe_error_ = {};
    }
    if (!pending_) return;
    std::optional<KeyframeReadView> view;
    if (pending_->header.status == TrajectoryStatus::kReady) {
      try {
        view = system_.GetKeyframeReadView();
        if (view->frame_count() < pending_->header.frame_count)
          throw std::invalid_argument(
              "Trajectory read view precedes its poses");
        if (!pending_->poses) {
          auto poses = system_.GetPoseSnapshot();
          CheckHeader(poses->frontier());
          const auto& header = pending_->header;
          if (poses->version != header.pose_version ||
              poses->frame_count != header.frame_count ||
              poses->optimized_frame_count != header.optimized_frame_count ||
              poses->parent_frame != header.parent_frame) {
            throw std::invalid_argument(
                "Trajectory pose snapshot does not match the observed "
                "frontier");
          }
          pending_->poses = std::move(poses);
        }
      } catch (...) {
        pending_->header.status = TrajectoryStatus::kError;
        pending_->header.error = ErrorText(std::current_exception());
        pending_->poses.reset();
        view.reset();
        terminal_pending_ = true;
      }
    }
    task_.emplace(Task{std::move(*pending_), std::move(view)});
    pending_.reset();
    busy_ = true;
    ++stats_.submitted;
    stats_.peak_work_payloads = 1;
    stats_.max_poll_seconds = std::max(stats_.max_poll_seconds, Seconds(start));
    condition_.notify_all();
  } catch (...) {
    RecordFailure(ErrorText(std::current_exception()));
    pending_.reset();
  }
}

void TrajectoryOutput::RecordFailure(const std::string& error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (failed_) return;
  failed_ = true;
  ++stats_.failed;
  stats_.error = error.substr(0, 1024);
  LOG(ERROR) << "livo_atlas trajectory output: " << stats_.error;
}

void TrajectoryOutput::Run() {
  for (;;) {
    std::optional<Task> task;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [&] { return stopping_.load() || task_; });
      if (stopping_) return;
      task = std::move(task_);
      task_.reset();
    }
    const auto start = Clock::now();
    auto result = std::move(task->source.header);
    std::uint64_t reads = 0;
    try {
      if (result.status == TrajectoryStatus::kReady) {
        while (timestamps_.size() < result.frame_count && !stopping_) {
          timestamps_.push_back(task->view->ReadTimestamp(timestamps_.size()));
          ++reads;
        }
        if (!stopping_) {
          auto built =
              BuildTrajectorySnapshot(*task->source.poses, timestamps_);
          built.producer_id = std::move(result.producer_id);
          built.revision = result.revision;
          result = std::move(built);
        }
      }
    } catch (...) {
      result.status = TrajectoryStatus::kError;
      result.error = ErrorText(std::current_exception());
      result.keyframes.clear();
    }
    bool delivered = false;
    try {
      if (!stopping_) {
        consumer_(result);
        delivered = true;
      }
    } catch (...) {
      RecordFailure(ErrorText(std::current_exception()));
    }
    if (result.status == TrajectoryStatus::kError) RecordFailure(result.error);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stats_.timestamp_reads += reads;
      stats_.max_worker_seconds =
          std::max(stats_.max_worker_seconds, Seconds(start));
      if (delivered) {
        ++stats_.delivered;
        stats_.resource_limited +=
            result.status == TrajectoryStatus::kResourceLimited;
        stats_.last_revision = result.revision;
        stats_.last_pose_version = result.pose_version;
        stats_.last_frame_count = result.frame_count;
        stats_.last_latency_seconds = Seconds(task->source.observed_at);
        last_delivery_ = Clock::now();
        have_delivery_ = true;
      }
      busy_ = false;
    }
    condition_.notify_all();
  }
}

void TrajectoryOutput::Finish() {
  if (finished_) return;
  if (options_.enabled) {
    if (!GetStats().failed && !terminal_pending_) {
      ObserveFrontier(system_.GetPoseFrontier(), {});
    }
    for (;;) {
      Poll(true);
      std::unique_lock<std::mutex> lock(mutex_);
      if (failed_ || (!busy_ && !pending_)) break;
      condition_.wait(lock, [&] { return !busy_; });
    }
  }
  Stop();
  finished_ = true;
  std::lock_guard<std::mutex> lock(mutex_);
  stats_.finished = true;
}

void TrajectoryOutput::Stop() {
  {
    // Use the wait's mutex so stop cannot notify between its check and sleep.
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  condition_.notify_all();
  if (thread_.joinable()) thread_.join();
  pending_.reset();
}

TrajectoryOutputStats TrajectoryOutput::GetStats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

}  // namespace livo_atlas
