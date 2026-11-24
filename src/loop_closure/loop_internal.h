#ifndef LIVO_ATLAS_SRC_LOOP_CLOSURE_LOOP_INTERNAL_H_
#define LIVO_ATLAS_SRC_LOOP_CLOSURE_LOOP_INTERNAL_H_

#include <array>
#include <optional>

#include "livo_atlas/backend/loop_closure.h"

namespace livo_atlas::loop_internal {

void ValidateLoopSubmapInfo(const LoopSubmapInfo& info);

// Internal timed entries preserve the public synchronous SDK signatures.
LoopVerification VerifyLoop(const GeometrySubmap& target,
                            const GeometrySubmap& source,
                            const LoopRegistrationOptions& options,
                            bool stage_timing);
LoopGeometryCheck EvaluateLoopGeometry(
    const GeometrySubmap& target, const GeometrySubmap& source,
    const RigidTransform& target_from_source,
    const LoopRegistrationOptions& options, bool stage_timing);

LoopMetrics EvaluateMetrics(const GeometryData& target,
                            const GeometryData& source,
                            const RigidTransform& target_from_source,
                            const LoopRegistrationOptions& options,
                            bool stage_timing = false);
// Target cycle, source cycle, forward correction, reverse correction.
std::array<LoopTransformError, 4> TransformErrors(
    const RigidTransform& initial, const RigidTransform& forward,
    const RigidTransform& reverse);
LoopStatus ClassifyQuality(
    const LoopMetrics& forward, const LoopMetrics& reverse,
    const std::optional<std::array<LoopTransformError, 4>>& errors,
    const LoopRegistrationOptions& options);

}  // namespace livo_atlas::loop_internal

#endif  // LIVO_ATLAS_SRC_LOOP_CLOSURE_LOOP_INTERNAL_H_
