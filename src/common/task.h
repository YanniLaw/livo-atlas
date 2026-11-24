/*
 * Copyright 2018 The Cartographer Authors
 * Modified by livo-atlas to use C++ standard-library synchronization and
 * isolate symbols from upstream Cartographer and cancel unsubmitted
 * dependencies.
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

#ifndef LIVO_ATLAS_SRC_COMMON_TASK_H_
#define LIVO_ATLAS_SRC_COMMON_TASK_H_

#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <vector>

namespace livo_atlas {

class ThreadPoolInterface;

class Task {
 public:
  friend class ThreadPoolInterface;

  using WorkItem = std::function<void()>;
  enum State { NEW, DISPATCHED, DEPENDENCIES_COMPLETED, RUNNING, COMPLETED };

  Task() = default;
  ~Task();

  State GetState();

  // State must be 'NEW'.
  void SetWorkItem(const WorkItem& work_item);

  // State must be 'NEW'. 'dependency' may be nullptr, in which case it is
  // assumed completed. Destroying a NEW task cancels its registrations.
  void AddDependency(std::weak_ptr<Task> dependency);

 private:
  // Allowed in all states.
  void AddDependentTask(Task* dependent_task);
  void RemoveDependentTask(Task* dependent_task);

  // State must be 'DEPENDENCIES_COMPLETED' and becomes 'COMPLETED'.
  void Execute();

  // State must be 'NEW' and becomes 'DISPATCHED' or 'DEPENDENCIES_COMPLETED'.
  void SetThreadPool(ThreadPoolInterface* thread_pool);

  // State must be 'NEW' or 'DISPATCHED'. If 'DISPATCHED', may become
  // 'DEPENDENCIES_COMPLETED'.
  void OnDependenyCompleted();

  WorkItem work_item_;
  ThreadPoolInterface* thread_pool_to_notify_ = nullptr;
  State state_ = NEW;
  unsigned int uncompleted_dependencies_ = 0;
  std::set<Task*> dependent_tasks_;
  // Only NEW tasks retain predecessors, so failed submission can unsubscribe.
  std::vector<std::shared_ptr<Task>> dependencies_;

  std::mutex mutex_;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_SRC_COMMON_TASK_H_
