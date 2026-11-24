#ifndef LIVO_ATLAS_SRC_LOOP_CLOSURE_LOOP_QUERY_H_
#define LIVO_ATLAS_SRC_LOOP_CLOSURE_LOOP_QUERY_H_

#include <functional>
#include <memory>

#include "livo_atlas/backend/loop_task.h"

namespace livo_atlas {

class ThreadPoolInterface;

LoopAdmissionResult RunLoopQuery(const LoopTask& task,
                                const LoopAdmissionOptions& options);
// Validates owned metadata without reading clouds or using the pool lock.
LoopAdmissionResult PrepareLoopTask(const LoopTask& task,
                                   const LoopAdmissionOptions& options);

// Registers the entire dependency graph before returning. The caller keeps
// the pool alive during registration; workers never schedule further tasks.
// On success, completion must not throw and is called once by the final task.
// If registration throws, submitted tasks drain without publishing a result.
void ScheduleLoopQuery(
    std::shared_ptr<LoopTaskResult> payload, LoopAdmissionOptions options,
    LoopAdmissionResult prepared, ThreadPoolInterface& pool, int num_threads,
    std::function<void(std::shared_ptr<LoopTaskResult>)> completion,
    bool stage_timing = false);

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_SRC_LOOP_CLOSURE_LOOP_QUERY_H_
