#ifndef LIVO_ATLAS_GAUSSIAN_GAUSSIAN_PROJECTION_H_
#define LIVO_ATLAS_GAUSSIAN_GAUSSIAN_PROJECTION_H_

#include "livo_atlas/gaussian/gaussian_map.h"

namespace livo_atlas {

struct GaussianProjection {
  std::uint64_t candidate_pixels = 0;
  double distance = 0;
  bool uncertain = false;
};

struct GaussianSpatialCandidate {
  std::uint64_t submap_id = 0;
  std::vector<GaussianExtent> extents;
  RigidTransform parent_from_submap;
};

struct GaussianSpatialSelection {
  std::optional<std::uint64_t> submap_id;
  GaussianProjection projection;
  std::uint64_t candidate_count = 0;
};

// Callers validate source identity/currentness. This shared geometry-only
// ranking retains earlier groups and orders by area, distance, then ID.
GaussianSpatialSelection SelectGaussianSpatialNeighbor(
    const std::vector<GaussianSpatialCandidate>& candidates,
    const GaussianCamera& camera, const RigidTransform& parent_from_body,
    std::uint64_t owner);

// Conservative candidate area, not visible/occlusion-tested image coverage.
// coordinate_abs_upper bounds the L1 sum of query/block parent and C_B
// translations; an overflowed positive infinity keeps the full canvas.
GaussianProjection ProjectGaussianExtents(
    const std::vector<GaussianExtent>& extents, const GaussianCamera& camera,
    const RigidTransform& camera_from_submap, double coordinate_abs_upper);

}  // namespace livo_atlas
#endif  // LIVO_ATLAS_GAUSSIAN_GAUSSIAN_PROJECTION_H_
