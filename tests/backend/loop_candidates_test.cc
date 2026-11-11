#include "livo_atlas/backend/loop_closure.h"

#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

namespace livo_atlas {
namespace {

LoopSubmapInfo Info(std::uint64_t id, std::uint64_t first, double time,
                   double x) {
  LoopSubmapInfo result;
  result.membership.primary = {id, first, {first, first + 1}};
  result.membership.membership_revision = 3;
  result.membership.state = OnlineSubmapState::kSealed;
  result.first_timestamp = time;
  result.last_timestamp = time + 1;
  result.frame_count = 200;
  result.parent_from_submap.translation.x() = x;
  return result;
}

TEST(LoopCandidatesTest, SortsDistanceThenIdAndBoundsCandidates) {
  const auto query = Info(10, 100, 100, 0);
  const std::vector<LoopSubmapInfo> history = {
      Info(2, 4, 4, 2), Info(0, 0, 0, -2), Info(1, 2, 2, 1)};
  LoopSearchOptions options;
  options.max_candidates = 2;
  const auto candidates = FindLoopCandidates(query, history, options);
  ASSERT_EQ(candidates.size(), 2u);
  EXPECT_EQ(candidates[0].target_id, 1u);
  EXPECT_EQ(candidates[1].target_id, 0u);
  EXPECT_EQ(candidates[0].source_id, 10u);
  EXPECT_DOUBLE_EQ(candidates[0].anchor_distance_m, 1);
  EXPECT_TRUE(FindLoopCandidates(query, {}).empty());
}

TEST(LoopCandidatesTest, IncludesExactGapAndRadiusBoundaries) {
  const auto query = Info(3, 51, 31, 15);
  const auto target = Info(0, 0, 0, 0);
  ASSERT_EQ(FindLoopCandidates(query, {target}).size(), 1u);
  auto changed = query;
  changed.parent_from_submap.translation.x() = 15.001;
  EXPECT_TRUE(FindLoopCandidates(changed, {target}).empty());
  changed = query;
  changed.first_timestamp = 30.999;
  EXPECT_TRUE(FindLoopCandidates(changed, {target}).empty());
  changed = Info(3, 50, 31, 15);
  EXPECT_TRUE(FindLoopCandidates(changed, {target}).empty());
  changed = Info(2, 51, 31, 15);
  EXPECT_TRUE(FindLoopCandidates(changed, {target}).empty());
}

TEST(LoopCandidatesTest, ExcludesFutureSelfAndSharedMembers) {
  const auto query = Info(10, 100, 100, 0);
  EXPECT_TRUE(FindLoopCandidates(query, {query, Info(11, 110, 110, 0),
                                         Info(0, 100, 0, 0)})
                  .empty());
}

TEST(LoopCandidatesTest, RejectsMalformedMetadataBeforeReturningCandidates) {
  const auto query = Info(10, 100, 100, 0);
  auto target = Info(0, 0, 0, 0);
  EXPECT_THROW(FindLoopCandidates(query, {target, target}),
               std::invalid_argument);
  target.first_timestamp = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(FindLoopCandidates(query, {target}), std::invalid_argument);
  target = Info(0, 0, 0, 0);
  target.pose_version = 1;
  target.optimized_frame_count = 100;
  EXPECT_THROW(FindLoopCandidates(query, {target}), std::invalid_argument);
  target = Info(0, 0, 0, 0);
  target.membership.state = OnlineSubmapState::kActive;
  EXPECT_THROW(FindLoopCandidates(query, {target}), std::invalid_argument);
  target = Info(0, 0, 0, 0);
  target.parent_from_submap.rotation.setZero();
  EXPECT_THROW(FindLoopCandidates(query, {target}), std::invalid_argument);
  LoopSearchOptions bad;
  bad.search_radius_m = std::numeric_limits<double>::infinity();
  EXPECT_THROW(FindLoopCandidates(query, {}, bad), std::invalid_argument);
}

}  // namespace
}  // namespace livo_atlas
