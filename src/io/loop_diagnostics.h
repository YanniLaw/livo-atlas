#ifndef LIVO_ATLAS_SRC_IO_LOOP_DIAGNOSTICS_H_
#define LIVO_ATLAS_SRC_IO_LOOP_DIAGNOSTICS_H_

#include <opencv2/core/persistence.hpp>

#include "livo_atlas/backend/loop_closure.h"

namespace livo_atlas::loop_diagnostics {

void WriteMetrics(cv::FileStorage& yaml, const LoopMetrics& metrics);
void WriteMatch(cv::FileStorage& yaml, const LoopCandidate& candidate,
                const LoopVerification& result);

}  // namespace livo_atlas::loop_diagnostics

#endif  // LIVO_ATLAS_SRC_IO_LOOP_DIAGNOSTICS_H_
