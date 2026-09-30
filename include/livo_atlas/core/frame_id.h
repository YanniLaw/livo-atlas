#ifndef LIVO_ATLAS_INCLUDE_LIVO_ATLAS_CORE_FRAME_ID_H_
#define LIVO_ATLAS_INCLUDE_LIVO_ATLAS_CORE_FRAME_ID_H_

#include <cstdint>

namespace livo_atlas {

// Frame IDs are numeric labels, not strongly typed identifiers. Zero is
// invalid; IDs assigned to frames start at one.
using FrameId = std::uint64_t;
constexpr FrameId kInvalidFrameId = 0;

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_INCLUDE_LIVO_ATLAS_CORE_FRAME_ID_H_
