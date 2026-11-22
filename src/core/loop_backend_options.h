#ifndef LIVO_ATLAS_SRC_CORE_LOOP_BACKEND_OPTIONS_H_
#define LIVO_ATLAS_SRC_CORE_LOOP_BACKEND_OPTIONS_H_

#include "livo_atlas/backend/loop_admission.h"
#include "livo_atlas/core/parameter_reader.h"

namespace livo_atlas {

LoopAdmissionOptions ReadOnlineLoopOptions(ParameterReader& reader);

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_SRC_CORE_LOOP_BACKEND_OPTIONS_H_
