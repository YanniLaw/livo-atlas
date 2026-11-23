/*
 * Copyright 2016 The Cartographer Authors
 * Modified by livo-atlas to use C++ standard-library synchronization and
 * isolate symbols from upstream Cartographer.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef LIVO_ATLAS_SRC_COMMON_THREAD_POOL_H_
#define LIVO_ATLAS_SRC_COMMON_THREAD_POOL_H_

#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include "common/task.h"

namespace livo_atlas {

class ThreadPoolInterface {
 public:
  ThreadPoolInterface() = default;
  virtual ~ThreadPoolInterface() = default;
  virtual std::weak_ptr<Task> Schedule(std::unique_ptr<Task> task) = 0;

 protected:
  void Execute(Task* task);
  void SetThreadPool(Task* task);

 private:
  friend class Task;

  virtual void NotifyDependenciesCompleted(Task* task) = 0;
};

// A fixed number of threads working on tasks. Adding a task does not block.
// Tasks may be added whether or not their dependencies are completed.
// When all dependencies of a task are completed, it is queued up for execution
// in a background thread. Stop scheduling new tasks before destruction.
// Outstanding dependencies must be able to complete during shutdown, when
// workers drain the ready queue and finish running work.
class ThreadPool : public ThreadPoolInterface {
 public:
  explicit ThreadPool(int num_threads);
  ~ThreadPool() override;

  ThreadPool(const ThreadPool&) = delete;
  ThreadPool& operator=(const ThreadPool&) = delete;

  // When the returned weak pointer is expired, 'task' has certainly completed,
  // so dependants no longer need to add it as a dependency.
  std::weak_ptr<Task> Schedule(std::unique_ptr<Task> task) override;

 private:
  void DoWork();

  void NotifyDependenciesCompleted(Task* task) override;

  std::mutex mutex_;
  std::condition_variable task_ready_;
  bool running_ = true;
  std::vector<std::thread> pool_;
  std::deque<std::shared_ptr<Task>> task_queue_;
  std::unordered_map<Task*, std::shared_ptr<Task>> tasks_not_ready_;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_SRC_COMMON_THREAD_POOL_H_
