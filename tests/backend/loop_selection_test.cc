#include "loop_closure/loop_selection.h"

#include <limits>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

namespace livo_atlas {
namespace {

using loop_internal::LoopHypothesis;
using loop_internal::SelectLoopCandidate;

OnlineSubmap Group(std::uint64_t id, std::uint64_t anchor) {
  return {{id, anchor, {anchor}}, {}, 2, OnlineSubmapState::kSealed};
}

LoopHypothesis Hypothesis(std::uint64_t target, std::uint64_t source,
                          std::uint64_t from, std::uint64_t to) {
  LoopHypothesis result;
  result.loop = {
      {from, to, {}, 0.1, 0.2}, Group(target, from), Group(source, to)};
  result.report.candidate = {target, source, 1};
  auto& verification = result.report.verification;
  verification.status = LoopStatus::kVerified;
  verification.target_from_source = RigidTransform{};
  LoopMetrics metrics;
  metrics.source_coverage = metrics.target_coverage = 1;
  metrics.rmse_m = 0.01;
  verification.forward.metrics = metrics;
  verification.reverse.metrics = metrics;
  return result;
}

PoseSnapshot Poses() {
  return MaterializePoseSnapshot({}, std::vector<RigidTransform>(64), 64);
}

TEST(LoopSelectionTest, TestsEveryPairBeforeSelectingTheBest) {
  auto a = Hypothesis(0, 10, 0, 30);
  auto b = Hypothesis(1, 10, 1, 30);
  auto c = Hypothesis(2, 10, 2, 30);
  b.loop.constraint.from_body_from_to_body.translation.x() = 0.4;
  c.loop.constraint.from_body_from_to_body.translation.x() = -0.4;
  const auto result = SelectLoopCandidate({a, b, c}, {}, Poses(), {});
  EXPECT_EQ(result.status, LoopAdmissionStatus::kAmbiguous);
  EXPECT_FALSE(result.index);
}

TEST(LoopSelectionTest, IncludesTranslationAndRotationBoundaries) {
  auto a = Hypothesis(0, 10, 0, 30);
  auto b = Hypothesis(1, 10, 1, 30);
  auto& transform = b.loop.constraint.from_body_from_to_body;
  transform.translation.x() = 0.5;
  transform.rotation = Eigen::AngleAxisd(5 * 3.14159265358979323846 / 180,
                                         Eigen::Vector3d::UnitZ())
                           .toRotationMatrix();
  EXPECT_EQ(SelectLoopCandidate({a, b}, {}, Poses(), {}).status,
            LoopAdmissionStatus::kProposed);
  transform.translation.x() = 0.50001;
  EXPECT_EQ(SelectLoopCandidate({a, b}, {}, Poses(), {}).status,
            LoopAdmissionStatus::kAmbiguous);
  transform.translation.x() = 0;
  transform.rotation =
      Eigen::AngleAxisd(0.09, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_EQ(SelectLoopCandidate({a, b}, {}, Poses(), {}).status,
            LoopAdmissionStatus::kAmbiguous);
}

TEST(LoopSelectionTest, OrdersByWorstCoverageThenWorstRmseDistanceAndId) {
  auto a = Hypothesis(0, 10, 0, 30);
  auto b = Hypothesis(1, 10, 1, 30);
  const auto select = [&] {
    return SelectLoopCandidate({b, a}, {}, Poses(), {}).index;
  };
  EXPECT_EQ(select(), 1u);
  b.report.candidate.anchor_distance_m = 0.5;
  EXPECT_EQ(select(), 0u);
  b.report.verification.reverse.metrics->rmse_m = 0.02;
  EXPECT_EQ(select(), 1u);
  a.report.verification.forward.metrics->source_coverage = 0.9;
  EXPECT_EQ(select(), 0u);
  b.report.verification.reverse.metrics->target_coverage = 0.8;
  EXPECT_EQ(select(), 1u);
}

TEST(LoopSelectionTest, ChecksAllPrimaryMemberIntersections) {
  const auto a = Hypothesis(0, 10, 1, 30);
  LoopAdmissionOptions options;
  options.correlated_submap_gap = 0;
  for (const bool current_source : {false, true}) {
    for (const bool previous_source : {false, true}) {
      auto previous = Hypothesis(20, 30, 40, 50).loop;
      auto& group = previous_source ? previous.source : previous.target;
      group.primary.keyframe_ids = {current_source ? 30u : 1u};
      const auto result =
          SelectLoopCandidate({a}, {previous}, Poses(), options);
      EXPECT_EQ(result.status, LoopAdmissionStatus::kCorrelated);
      EXPECT_FALSE(result.index);
    }
  }
}

TEST(LoopSelectionTest, RejectsCorrelatedBestWithoutTryingWeakerAlternative) {
  auto best = Hypothesis(2, 10, 2, 30);
  auto weaker = Hypothesis(8, 10, 8, 30);
  weaker.report.verification.forward.metrics->source_coverage = 0.9;
  auto previous = Hypothesis(0, 8, 0, 20).loop;
  const auto result =
      SelectLoopCandidate({best, weaker}, {previous}, Poses(), {});
  EXPECT_EQ(result.status, LoopAdmissionStatus::kCorrelated);
  EXPECT_FALSE(result.index);
  previous.target.primary.id = 20;
  EXPECT_EQ(SelectLoopCandidate({best}, {previous}, Poses(), {}).status,
            LoopAdmissionStatus::kProposed);
}

TEST(LoopSelectionTest, ComputesUnsignedEndpointDistancesWithoutWrapping) {
  const auto a = Hypothesis(0, 5, 0, 10);
  const auto max = std::numeric_limits<std::uint64_t>::max();
  const auto previous = Hypothesis(max - 1, max, 30, 40).loop;
  EXPECT_EQ(SelectLoopCandidate({a}, {previous}, Poses(), {}).status,
            LoopAdmissionStatus::kProposed);
  EXPECT_EQ(SelectLoopCandidate({}, {}, Poses(), {}).status,
            LoopAdmissionStatus::kGeometryRejected);
}

}  // namespace
}  // namespace livo_atlas
