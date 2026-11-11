#include "livo_atlas/backend/loop_closure.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace livo_atlas {
namespace {

void ValidateInfo(const LoopSubmapInfo& info) {
  const auto& group = info.membership;
  ValidateSubmap(group.primary);
  const auto& ids = group.primary.keyframe_ids;
  const auto error = ValidateExtrinsic(info.parent_from_submap);
  if (!error.empty() || group.state != OnlineSubmapState::kSealed ||
      ids.size() > kMaxGeometryKeyframes ||
      group.membership_revision != ids.size() + 1 ||
      ids.back() >= info.frame_count ||
      info.optimized_frame_count > info.frame_count ||
      ((info.pose_version == 0) != (info.optimized_frame_count == 0)) ||
      !std::isfinite(info.first_timestamp) ||
      !std::isfinite(info.last_timestamp) ||
      info.first_timestamp > info.last_timestamp) {
    throw std::invalid_argument("Invalid loop submap metadata");
  }
  const auto& support = group.support_keyframe_ids;
  for (std::size_t i = 0; i < support.size(); ++i) {
    if (support[i] >= ids.front() || (i && support[i] - support[i - 1] != 1)) {
      throw std::invalid_argument("Invalid loop support members");
    }
  }
}

}  // namespace

void ValidateLoopSearchOptions(const LoopSearchOptions& options) {
  if (!options.min_submap_gap || options.min_keyframe_gap < 2 ||
      !std::isfinite(options.min_time_gap_s) || options.min_time_gap_s <= 0 ||
      !std::isfinite(options.search_radius_m) || options.search_radius_m <= 0 ||
      !options.max_candidates) {
    throw std::invalid_argument("Invalid loop search options");
  }
}

std::vector<LoopCandidate> FindLoopCandidates(
    const LoopSubmapInfo& query, const std::vector<LoopSubmapInfo>& history,
    const LoopSearchOptions& options) {
  ValidateLoopSearchOptions(options);
  ValidateInfo(query);
  std::set<std::uint64_t> seen;
  std::vector<LoopCandidate> result;
  for (const auto& target : history) {
    ValidateInfo(target);
    const auto id = target.membership.primary.id;
    if (!seen.insert(id).second || target.pose_version != query.pose_version ||
        target.frame_count != query.frame_count ||
        target.optimized_frame_count != query.optimized_frame_count) {
      throw std::invalid_argument("Duplicate loop ID or mixed pose views");
    }
    const auto& before = target.membership.primary.keyframe_ids;
    const auto& after = query.membership.primary.keyframe_ids;
    // Check ordering before unsigned subtraction, including overlapping
    // members.
    if (id >= query.membership.primary.id || before.back() >= after.front() ||
        query.membership.primary.id - id < options.min_submap_gap ||
        after.front() - before.back() < options.min_keyframe_gap ||
        query.first_timestamp - target.last_timestamp <
            options.min_time_gap_s) {
      continue;
    }
    const double distance = (query.parent_from_submap.translation -
                             target.parent_from_submap.translation)
                                .stableNorm();
    if (distance <= options.search_radius_m) {
      result.push_back({id, query.membership.primary.id, distance});
    }
  }
  std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) {
    return a.anchor_distance_m < b.anchor_distance_m ||
           (a.anchor_distance_m == b.anchor_distance_m &&
            a.target_id < b.target_id);
  });
  if (result.size() > options.max_candidates)
    result.resize(options.max_candidates);
  return result;
}

}  // namespace livo_atlas
