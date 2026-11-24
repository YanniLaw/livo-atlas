#ifndef LIVO_ATLAS_SRC_LOOP_CLOSURE_LOOP_SELECTION_H_
#define LIVO_ATLAS_SRC_LOOP_CLOSURE_LOOP_SELECTION_H_

#include <cstddef>
#include <optional>
#include <vector>

#include "livo_atlas/backend/loop_admission.h"

namespace livo_atlas::loop_internal {

// Only geometrically verified candidates enter selection.
struct LoopHypothesis {
  AcceptedLoop loop;
  LoopCandidateReport report;
};

struct LoopSelection {
  LoopAdmissionStatus status = LoopAdmissionStatus::kGeometryRejected;
  std::optional<std::size_t> index;
};

LoopSelection SelectLoopCandidate(const std::vector<LoopHypothesis>& hypotheses,
                                  const std::vector<AcceptedLoop>& accepted,
                                  const PoseSnapshot& poses,
                                  const LoopAdmissionOptions& options);

}  // namespace livo_atlas::loop_internal

#endif  // LIVO_ATLAS_SRC_LOOP_CLOSURE_LOOP_SELECTION_H_
