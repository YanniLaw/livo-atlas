#include "livo_atlas/backend/loop_task.h"

#include <utility>

#include "loop_closure/loop_query.h"

namespace livo_atlas {

LoopClosure::LoopClosure(LoopAdmissionOptions options)
    : options_(std::move(options)) {}

LoopAdmissionResult LoopClosure::Run(const LoopTask& task) const {
  return RunLoopQuery(task, options_);
}

LoopAdmissionResult RunLoopTask(const LoopTask& task,
                                const LoopAdmissionOptions& options) {
  return LoopClosure(options).Run(task);
}

}  // namespace livo_atlas
