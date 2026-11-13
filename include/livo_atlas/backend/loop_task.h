#ifndef LIVO_ATLAS_BACKEND_LOOP_TASK_H_
#define LIVO_ATLAS_BACKEND_LOOP_TASK_H_

#include <cstdint>
#include <exception>
#include <optional>
#include <vector>

#include "livo_atlas/backend/loop_admission.h"
#include "livo_atlas/io/keyframe_read_view.h"

namespace livo_atlas {

struct LoopTask {
  std::uint64_t task_id = 0;
  LoopAdmissionInput input;
  std::vector<double> timestamps;
  KeyframeReadView frames;
};

struct LoopTaskResult {
  LoopTask task;
  std::optional<LoopAdmissionResult> admission;
  std::exception_ptr error;
};

// Uses only owned task data, committed assets and a private geometry cache.
LoopAdmissionResult RunLoopTask(const LoopTask& task,
                                const LoopAdmissionOptions& options);

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_BACKEND_LOOP_TASK_H_
