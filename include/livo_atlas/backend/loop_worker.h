#ifndef LIVO_ATLAS_BACKEND_LOOP_WORKER_H_
#define LIVO_ATLAS_BACKEND_LOOP_WORKER_H_

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>

#include "livo_atlas/backend/loop_task.h"

namespace livo_atlas::cartographer::common {
class ThreadPool;
}  // namespace livo_atlas::cartographer::common

namespace livo_atlas {

using LoopTaskRunner = std::function<LoopAdmissionResult(const LoopTask&)>;

// One serial caller owns Submit/Take/Wait/Stop. A task and its result share
// one slot; taking a result is required before submitting another task.
class LoopWorker {
 public:
  explicit LoopWorker(LoopTaskRunner runner);
  ~LoopWorker();
  LoopWorker(const LoopWorker&) = delete;
  LoopWorker& operator=(const LoopWorker&) = delete;

  bool idle() const;
  void Submit(LoopTask task);
  std::optional<LoopTaskResult> Take();
  void Wait();
  // Joins without cancelling an active runner or committing any result.
  void Stop();

 private:
  void Run(LoopTask task);

  LoopTaskRunner runner_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::optional<LoopTaskResult> result_;
  bool busy_ = false;
  bool stopping_ = false;
  std::unique_ptr<cartographer::common::ThreadPool> thread_pool_;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_BACKEND_LOOP_WORKER_H_
