#include "loop_closure/loop_selection.h"

#include <algorithm>
#include <cmath>
#include <tuple>

#include <Eigen/Geometry>

#include "common/rotation_projection.h"
#include "mapping/pose_math.h"

namespace livo_atlas::loop_internal {
namespace {

bool Intersects(const Submap& a, const Submap& b) {
  auto i = a.keyframe_ids.begin(), j = b.keyframe_ids.begin();
  while (i != a.keyframe_ids.end() && j != b.keyframe_ids.end()) {
    if (*i == *j) return true;
    if (*i < *j)
      ++i;
    else
      ++j;
  }
  return false;
}

std::uint64_t Distance(std::uint64_t a, std::uint64_t b) {
  return a > b ? a - b : b - a;
}

bool Correlated(const AcceptedLoop& a, const AcceptedLoop& b,
                std::uint64_t gap) {
  for (const auto* endpoint_a : {&a.target.primary, &a.source.primary}) {
    for (const auto* endpoint_b : {&b.target.primary, &b.source.primary}) {
      if (Intersects(*endpoint_a, *endpoint_b)) return true;
    }
  }
  return Distance(a.target.primary.id, b.target.primary.id) <= gap &&
         Distance(a.source.primary.id, b.source.primary.id) <= gap;
}

auto Rank(const LoopHypothesis& hypothesis) {
  const auto& f = *hypothesis.report.verification.forward.metrics;
  const auto& r = *hypothesis.report.verification.reverse.metrics;
  return std::make_tuple(-std::min({f.source_coverage, f.target_coverage,
                                    r.source_coverage, r.target_coverage}),
                         std::max(*f.rmse_m, *r.rmse_m),
                         hypothesis.report.candidate.anchor_distance_m,
                         hypothesis.report.candidate.target_id);
}

RigidTransform HypothesisPose(const LoopHypothesis& hypothesis,
                              const PoseSnapshot& poses) {
  auto anchor =
      poses.poses[hypothesis.loop.constraint.from_id].parent_from_body();
  anchor.rotation = ProjectRotation(anchor.rotation);
  return pose_math::Compose(anchor,
                            hypothesis.loop.constraint.from_body_from_to_body);
}

}  // namespace

LoopSelection SelectLoopCandidate(const std::vector<LoopHypothesis>& hypotheses,
                                  const std::vector<AcceptedLoop>& accepted,
                                  const PoseSnapshot& poses,
                                  const LoopAdmissionOptions& options) {
  if (hypotheses.empty()) return {};
  for (std::size_t i = 0; i < hypotheses.size(); ++i) {
    const auto inverse =
        pose_math::Inverse(HypothesisPose(hypotheses[i], poses));
    for (std::size_t j = i + 1; j < hypotheses.size(); ++j) {
      const auto delta =
          pose_math::Compose(inverse, HypothesisPose(hypotheses[j], poses));
      const double angle = Eigen::AngleAxisd(delta.rotation).angle();
      if (!delta.translation.allFinite() || !std::isfinite(angle) ||
          delta.translation.stableNorm() >
              options.max_hypothesis_translation_m ||
          angle > options.max_hypothesis_rotation_deg *
                      (3.14159265358979323846 / 180)) {
        return {LoopAdmissionStatus::kAmbiguous, std::nullopt};
      }
    }
  }
  const auto best = std::min_element(
      hypotheses.begin(), hypotheses.end(),
      [](const auto& a, const auto& b) { return Rank(a) < Rank(b); });
  for (const auto& previous : accepted) {
    if (Correlated(best->loop, previous, options.correlated_submap_gap))
      return {LoopAdmissionStatus::kCorrelated, std::nullopt};
  }
  return {LoopAdmissionStatus::kProposed,
          static_cast<std::size_t>(best - hypotheses.begin())};
}

}  // namespace livo_atlas::loop_internal
