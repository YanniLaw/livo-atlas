#ifndef LIVO_ATLAS_MAPPING_MAP_CORRECTION_H_
#define LIVO_ATLAS_MAPPING_MAP_CORRECTION_H_

#include <cstdint>

#include "livo_atlas/sensor/extrinsics.h"

namespace livo_atlas {

// A derived view of committed poses. New raw frames advance frame_count
// without changing the optimization version or its map-to-odom correction.
struct MapCorrection {
  std::uint64_t pose_version = 0;
  std::uint64_t frame_count = 0;
  std::uint64_t optimized_frame_count = 0;
  RigidTransform map_from_odom;
};

// The caller supplies the rigid pose represented by its published odometry,
// rather than a nearly orthogonal filter matrix. Both inputs remain unchanged.
RigidTransform ApplyMapCorrection(
    const MapCorrection& correction,
    const RigidTransform& published_odom_from_body);

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_MAPPING_MAP_CORRECTION_H_
