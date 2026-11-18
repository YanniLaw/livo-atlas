#ifndef LIVO_ATLAS_GAUSSIAN_GAUSSIAN_PROJECTION_H_
#define LIVO_ATLAS_GAUSSIAN_GAUSSIAN_PROJECTION_H_

#include "livo_atlas/gaussian/gaussian_map.h"

namespace livo_atlas {

struct GaussianProjection {
  std::uint64_t candidate_pixels = 0;
  double distance = 0;
  bool uncertain = false;
};

// Conservative candidate area, not visible/occlusion-tested image coverage.
// coordinate_abs_upper bounds the L1 sum of query/block parent and C_B
// translations; an overflowed positive infinity keeps the full canvas.
GaussianProjection ProjectGaussianExtents(
    const std::vector<GaussianExtent>& extents, const GaussianCamera& camera,
    const RigidTransform& camera_from_submap, double coordinate_abs_upper);

}  // namespace livo_atlas
#endif  // LIVO_ATLAS_GAUSSIAN_GAUSSIAN_PROJECTION_H_
