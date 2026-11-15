#ifndef LIVO_ATLAS_SRC_MAPPING_GEOMETRY_VALIDATION_H_
#define LIVO_ATLAS_SRC_MAPPING_GEOMETRY_VALIDATION_H_

#include <vector>

#include "livo_atlas/mapping/geometry_submap.h"

namespace livo_atlas::geometry_detail {

void ValidateRevision(const OnlineSubmap& group, const PoseSnapshot& poses);
void ValidateGroups(const std::vector<OnlineSubmap>& groups,
                    const PoseSnapshot& poses);

}  // namespace livo_atlas::geometry_detail

#endif  // LIVO_ATLAS_SRC_MAPPING_GEOMETRY_VALIDATION_H_
