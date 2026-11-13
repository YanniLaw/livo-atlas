#ifndef LIVO_ATLAS_SRC_CORE_LOOP_COMMIT_H_
#define LIVO_ATLAS_SRC_CORE_LOOP_COMMIT_H_

#include <optional>
#include <vector>

#include "livo_atlas/backend/loop_task.h"

namespace livo_atlas {
enum class LoopCommitStatus { kEligible, kStale };

struct LoopCommitCheck {
  LoopCommitStatus status = LoopCommitStatus::kStale;
  std::optional<PoseSnapshot> poses;
};

LoopCommitCheck CheckLoopCommit(const LoopTask& task,
                                const LoopAdmissionResult& result,
                                const LoopAdmissionInput& current,
                                const std::vector<double>& current_timestamps);
}  // namespace livo_atlas

#endif  // LIVO_ATLAS_SRC_CORE_LOOP_COMMIT_H_
