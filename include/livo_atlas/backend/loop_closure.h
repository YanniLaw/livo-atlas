#ifndef LIVO_ATLAS_BACKEND_LOOP_CLOSURE_H_
#define LIVO_ATLAS_BACKEND_LOOP_CLOSURE_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "livo_atlas/mapping/geometry_submap.h"

namespace livo_atlas {

struct LoopSearchOptions {
  std::uint64_t min_submap_gap = 3;
  std::uint64_t min_keyframe_gap = 50;
  double min_time_gap_s = 30;
  double search_radius_m = 15;
  std::size_t max_candidates = 5;
};

// Metadata only: candidate retrieval never retains a cloud or image.
struct LoopSubmapInfo {
  OnlineSubmap membership;
  double first_timestamp = 0;
  double last_timestamp = 0;
  RigidTransform parent_from_submap;
  std::uint64_t pose_version = 0;
  std::uint64_t frame_count = 0;
  std::uint64_t optimized_frame_count = 0;
};

struct LoopCandidate {
  std::uint64_t target_id = 0;
  std::uint64_t source_id = 0;
  double anchor_distance_m = 0;
};

void ValidateLoopSearchOptions(const LoopSearchOptions& options);
std::vector<LoopCandidate> FindLoopCandidates(
    const LoopSubmapInfo& query, const std::vector<LoopSubmapInfo>& history,
    const LoopSearchOptions& options = {});

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_BACKEND_LOOP_CLOSURE_H_
