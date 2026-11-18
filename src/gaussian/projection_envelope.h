#ifndef LIVO_ATLAS_SRC_GAUSSIAN_PROJECTION_ENVELOPE_H_
#define LIVO_ATLAS_SRC_GAUSSIAN_PROJECTION_ENVELOPE_H_

#include "livo_atlas/gaussian/gaussian_map.h"

namespace livo_atlas {

// Half-open integer canvas rectangle for one validated origin extent.
struct GaussianEnvelope {
  int left = 0, top = 0, right = 0, bottom = 0;
  bool uncertain = false;
};

// Caller validates the extent, camera and rigid transform before aggregation.
GaussianEnvelope ProjectGaussianEnvelope(
    const GaussianExtent& extent, const GaussianCamera& camera,
    const RigidTransform& camera_from_submap, double coordinate_abs_upper);

}  // namespace livo_atlas
#endif  // LIVO_ATLAS_SRC_GAUSSIAN_PROJECTION_ENVELOPE_H_
