#include "livo_atlas/backend/loop_worker.h"

#include <stdexcept>
#include <utility>

namespace livo_atlas {

LoopWorker::LoopWorker(LoopTaskRunner runner) : runner_(std::move(runner)) {
  if (!runner_) throw std::invalid_argument("Missing loop task runner");
  thread_ = std::thread(&LoopWorker::Run, this);
}

LoopWorker::~LoopWorker() { Stop(); }

bool LoopWorker::idle() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return !stopping_ && !busy_ && !result_;
}

void LoopWorker::Submit(LoopTask task) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || busy_ || result_)
      throw std::logic_error("Loop worker is not idle");
    task_.emplace(std::move(task));
    busy_ = true;
  }
  condition_.notify_all();
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
  condition_.notify_all();
  if (thread_.joinable()) thread_.join();
}

std::optional<LoopTask> LoopWorker::NextTask() {
  std::unique_lock<std::mutex> lock(mutex_);
  condition_.wait(lock, [&] { return stopping_ || task_; });
  auto task = std::move(task_);
  task_.reset();
  if (stopping_) {
    busy_ = false;
    lock.unlock();
    condition_.notify_all();
    return {};
  }
  return task;
}

void LoopWorker::Run() {
  while (auto task = NextTask()) {
    LoopTaskResult result{std::move(*task), {}, {}};
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
}

}  // namespace livo_atlas
