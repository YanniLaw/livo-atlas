#include "livo_atlas/backend/loop_worker.h"

#include <stdexcept>
#include <utility>

#include "cartographer/common/thread_pool.h"

namespace livo_atlas {

LoopWorker::LoopWorker(LoopTaskRunner runner) : runner_(std::move(runner)) {
  if (!runner_) throw std::invalid_argument("Missing loop task runner");
  thread_pool_ = std::make_unique<cartographer::common::ThreadPool>(1);
}

LoopWorker::~LoopWorker() { Stop(); }

bool LoopWorker::idle() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return !stopping_ && !busy_ && !result_;
}

void LoopWorker::Submit(LoopTask task) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (stopping_ || busy_ || result_) {
    throw std::logic_error("Loop worker is not idle");
  }
  auto work = std::make_unique<cartographer::common::Task>();
  // Cartographer stores a copyable callback. Sharing the payload avoids
  // copying the task data when setting the callback.
  auto payload = std::make_shared<LoopTask>(std::move(task));
  work->SetWorkItem([this, payload] { Run(std::move(*payload)); });
  busy_ = true;
  try {
    thread_pool_->Schedule(std::move(work));
  } catch (...) {
    busy_ = false;
    throw;
  }
}

std::optional<LoopTaskResult> LoopWorker::Take() {
  std::lock_guard<std::mutex> lock(mutex_);
  auto result = std::move(result_);
  result_.reset();
  return result;
}

void LoopWorker::Wait() {
  std::unique_lock<std::mutex> lock(mutex_);
  condition_.wait(lock, [&] { return !busy_; });
}

void LoopWorker::Stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  // A queued task checks stopping_ before starting. Join outside mutex_ so
  // a running task can publish its result.
  thread_pool_.reset();
}

void LoopWorker::Run(LoopTask task) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_) {
      busy_ = false;
      condition_.notify_all();
      return;
    }
  }
  LoopTaskResult result{std::move(task), {}, {}};
  try {
    result.admission = runner_(result.task);
  } catch (...) {
    result.error = std::current_exception();
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (result.error) stopping_ = true;
    result_.emplace(std::move(result));
    busy_ = false;
  }
  condition_.notify_all();
}

}  // namespace livo_atlas
