#ifndef LIVO_ATLAS_SRC_LOOP_CLOSURE_LOOP_ADMISSION_H_
#define LIVO_ATLAS_SRC_LOOP_CLOSURE_LOOP_ADMISSION_H_

#include "livo_atlas/backend/loop_admission.h"

namespace livo_atlas::loop_internal {

// These stages are shared by synchronous SDK calls and dependent pool tasks.
LoopAdmissionResult PrepareLoopAdmission(const LoopAdmissionInput& input,
                                         const LoopAdmissionOptions& options);
GeometrySubmap ReadLoopGeometry(const OnlineSubmap& membership,
                                const PoseSnapshot& poses,
                                const LoopGeometryReader& reader);
LoopAdmissionResult FinishLoopAdmission(const LoopAdmissionInput& input,
                                        const LoopGeometryReader& reader,
                                        const LoopAdmissionOptions& options,
                                        LoopAdmissionResult result);

}  // namespace livo_atlas::loop_internal

#endif  // LIVO_ATLAS_SRC_LOOP_CLOSURE_LOOP_ADMISSION_H_
