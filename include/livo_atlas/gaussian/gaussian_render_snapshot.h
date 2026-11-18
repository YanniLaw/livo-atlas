#ifndef LIVO_ATLAS_GAUSSIAN_GAUSSIAN_RENDER_SNAPSHOT_H_
#define LIVO_ATLAS_GAUSSIAN_GAUSSIAN_RENDER_SNAPSHOT_H_

#include <cstdint>
#include <memory>
#include <string>

#include "livo_atlas/gaussian/gaussian_render.h"

namespace livo_atlas {

enum class GaussianRenderStatus : std::uint8_t {
  kReady = 0,
  kWaiting = 1,
  kInvalidated = 2,
  kDisabled = 3,
  kError = 4,
  kStopping = 5,
  kStopped = 6
};

// An as-of owner capture. Only READY owns pixels; no asset lease is held.
// The output controller assigns a separate producer/revision ordering domain.
struct GaussianRenderSnapshot {
  std::string producer_id;
  std::uint64_t revision = 0;
  std::string session_id;
  std::uint64_t generation = 0, map_sequence = 0, pose_version = 0;
  std::uint64_t frame_count = 0, optimized_frame_count = 0;
  PoseFrame parent_frame = PoseFrame::kOdom;
  GaussianRenderPolicy policy = GaussianRenderPolicy::kTemporal;
  GaussianRenderStatus status = GaussianRenderStatus::kWaiting;
  std::string reason = "awaiting_render";
  std::shared_ptr<const GaussianRender> render;
};

// Bounded scalar/dimension checks are safe on the owner. Complete camera/source
// validation belongs to the worker, or to a native transport boundary.
void ValidateGaussianRenderSnapshotHeader(const GaussianRenderSnapshot& source,
                                         bool require_output_identity);
void ValidateGaussianRenderSnapshot(const GaussianRenderSnapshot& source);

}  // namespace livo_atlas
#endif  // LIVO_ATLAS_GAUSSIAN_GAUSSIAN_RENDER_SNAPSHOT_H_
