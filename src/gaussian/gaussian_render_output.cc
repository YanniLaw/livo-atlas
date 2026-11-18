#include "livo_atlas/gaussian/gaussian_render_output.h"

#include <algorithm>
#include <random>
#include <stdexcept>
#include <utility>

namespace livo_atlas {
namespace {
using Clock = std::chrono::steady_clock;

std::string ProducerId() {
  std::random_device random;
  constexpr char kHex[] = "0123456789abcdef";
  std::string result;
  for (int i = 0; i < 4; ++i) {
    const std::uint32_t value = random();
    for (int shift = 28; shift >= 0; shift -= 4)
      result.push_back(kHex[(value >> shift) & 15]);
  }
  return result;
}

bool SameHeader(const GaussianRenderSnapshot& a,
                const GaussianRenderSnapshot& b) {
  return a.session_id == b.session_id && a.generation == b.generation &&
         a.map_sequence == b.map_sequence && a.pose_version == b.pose_version &&
         a.frame_count == b.frame_count &&
         a.optimized_frame_count == b.optimized_frame_count &&
         a.parent_frame == b.parent_frame && a.policy == b.policy &&
         a.status == b.status && a.reason == b.reason;
}

}  // namespace

GaussianRenderOutput::GaussianRenderOutput(GaussianRenderOutputOptions options,
                                           Consumer consumer)
    : options_(std::move(options)),
      consumer_(std::move(consumer)),
      producer_id_(options_.enabled ? ProducerId() : "") {
  if (!options_.enabled) return;
  // The CPU scheduler uses only the interval. Frame names are validated by the
  // config loader and native publisher, which know the actual parent frame.
  if (options_.minimum_interval.count() < 100 ||
      options_.minimum_interval.count() > 60000)
    throw std::invalid_argument("Invalid Gaussian output interval");
  if (!consumer_) throw std::invalid_argument("Missing Gaussian consumer");
  thread_ = std::thread(&GaussianRenderOutput::Run, this);
}

GaussianRenderOutput::~GaussianRenderOutput() { Finish(); }

void GaussianRenderOutput::Queue(GaussianRenderSnapshot source, bool urgent) {
  // Called under mutex. Replacing the slot releases its previous RGB reference.
  if (pending_) ++stats_.coalesced;
  source.producer_id = producer_id_;
  source.revision = ++stats_.last_revision;
  pending_ = std::move(source);
  urgent_ = urgent_ || urgent;
  ++stats_.submitted;
  stats_.pending = stats_.peak_pending = 1;
  condition_.notify_one();
}

void GaussianRenderOutput::Fail(const char* reason) {
  // Called under mutex. The error event itself is never retried on failure.
  if (stats_.failed) return;
  ++stats_.failed;
  stats_.error = reason;
  pending_.reset();
  stats_.pending = 0;
  latest_render_.reset();
  if (latest_) {
    auto error = *latest_;
    error.status = GaussianRenderStatus::kError;
    error.reason = reason;
    Queue(std::move(error), true);
  }
}

void GaussianRenderOutput::Observe(
    const GaussianRenderSnapshot& source) noexcept {
  if (!options_.enabled) return;
  std::lock_guard<std::mutex> lock(mutex_);
  if (closing_ || stats_.failed) return;
  ++stats_.observed;
  try {
    ValidateGaussianRenderSnapshotHeader(source, false);
    if (latest_ && SameHeader(*latest_, source) &&
        latest_render_.lock() == source.render)
      return;
    const bool exceptional = source.status == GaussianRenderStatus::kError ||
                             source.status == GaussianRenderStatus::kDisabled ||
                             source.status == GaussianRenderStatus::kStopping ||
                             source.status == GaussianRenderStatus::kStopped;
    const bool urgent =
        !latest_ || latest_->status != source.status || exceptional;
    // Release the old pending payload before taking a new RGB reference. The
    // in-flight callback may still own a different image.
    if (pending_) {
      ++stats_.coalesced;
      pending_.reset();
      stats_.pending = 0;
    }
    latest_ = source;
    latest_->render.reset();
    latest_render_ = source.render;
    Queue(source, urgent);
    if (source.status == GaussianRenderStatus::kStopped) closing_ = true;
  } catch (...) {
    // A malformed image may still have a usable source header. Never invent a
    // source for an invalid identity, and never retain the malformed pixels.
    if (!latest_ && source.session_id.size() == 32 &&
        source.producer_id.size() <= 32 && source.reason.size() <= 64) {
      try {
        auto context = source;
        context.render.reset();
        context.status = GaussianRenderStatus::kError;
        context.reason = "invalid_snapshot";
        ValidateGaussianRenderSnapshotHeader(context, false);
        latest_ = std::move(context);
      } catch (...) {
      }
    }
    Fail("invalid_snapshot");
  }
}

void GaussianRenderOutput::Run() noexcept {
  auto next = Clock::now();
  std::unique_lock<std::mutex> lock(mutex_);
  while (true) {
    condition_.wait(lock, [&] { return closing_ || pending_.has_value(); });
    if (!pending_ && closing_) break;
    if (!urgent_ && !closing_ && Clock::now() < next) {
      condition_.wait_until(lock, next, [&] { return closing_ || urgent_; });
      continue;
    }
    auto source = std::move(*pending_);
    pending_.reset();
    urgent_ = false;
    stats_.pending = 0;
    stats_.in_flight = stats_.peak_in_flight = 1;
    lock.unlock();
    const auto start = Clock::now();
    bool published = false;
    const char* failure = nullptr;
    try {
      ValidateGaussianRenderSnapshot(source);
    } catch (...) {
      failure = "invalid_snapshot";
    }
    // Validation is deliberately outside the mutex. Admit the callback only
    // if the owner has not replaced this revision while validation ran.
    lock.lock();
    if (source.revision != stats_.last_revision) {
      source.render.reset();
      stats_.in_flight = 0;
      ++stats_.coalesced;
      stats_.max_worker_seconds =
          std::max(stats_.max_worker_seconds,
                   std::chrono::duration<double>(Clock::now() - start).count());
      continue;
    }
    lock.unlock();
    if (!failure) {
      try {
        published = consumer_(source);
      } catch (...) {
        failure = "publication_failed";
      }
    }
    // Release pixels before opening the next execution slot.
    source.render.reset();
    const auto elapsed = std::chrono::duration<double>(Clock::now() - start);
    lock.lock();
    stats_.in_flight = 0;
    stats_.max_worker_seconds =
        std::max(stats_.max_worker_seconds, elapsed.count());
    if (failure) {
      Fail(failure);
    } else if (published) {
      ++stats_.published;
    } else {
      ++stats_.unavailable;
    }
    next = Clock::now() + options_.minimum_interval;
  }
}

void GaussianRenderOutput::Finish() noexcept {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stats_.finished) return;
    const bool already_terminal = closing_;
    closing_ = true;
    if (!stats_.failed && !already_terminal) {
      pending_.reset();
      stats_.pending = 0;
      if (latest_ && options_.enabled) {
        auto stopped = *latest_;
        stopped.status = GaussianRenderStatus::kStopped;
        stopped.reason = latest_->reason == "mapper_failed" ||
                                 latest_->reason == "render_disabled"
                             ? latest_->reason
                             : "stopped";
        Queue(std::move(stopped), true);
      }
    }
    latest_render_.reset();
    condition_.notify_one();
  }
  if (thread_.joinable()) thread_.join();
  std::lock_guard<std::mutex> lock(mutex_);
  latest_.reset();
  pending_.reset();
  stats_.pending = stats_.in_flight = 0;
  stats_.finished = true;
}

GaussianRenderOutputStats GaussianRenderOutput::GetStats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

}  // namespace livo_atlas
