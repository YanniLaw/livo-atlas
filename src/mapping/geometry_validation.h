#ifndef LIVO_ATLAS_SRC_MAPPING_GEOMETRY_VALIDATION_H_
#define LIVO_ATLAS_SRC_MAPPING_GEOMETRY_VALIDATION_H_

#include <cstdint>
#include <vector>

#include "livo_atlas/mapping/geometry_submap.h"

namespace livo_atlas::geometry_detail {

bool SameMembership(const OnlineSubmap& a, const OnlineSubmap& b);
void ValidateRevision(const OnlineSubmap& group, std::uint64_t frame_count);
void ValidateGroups(const std::vector<OnlineSubmap>& groups,
                    const PoseSnapshot& poses);

// Stage changes to the last group and any new groups before changing a cache.
std::vector<OnlineSubmap> ReviseSubmapTail(
    const OnlineSubmap* last, const std::vector<OnlineSubmap>& revisions,
    std::uint64_t frame_count);

}  // namespace livo_atlas::geometry_detail

#endif  // LIVO_ATLAS_SRC_MAPPING_GEOMETRY_VALIDATION_H_
