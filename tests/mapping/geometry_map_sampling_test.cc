#include "mapping/geometry_map_sampling.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

namespace livo_atlas::geometry_map_detail {
namespace {

GeometryMapCandidate Candidate(std::uint64_t submap, GeometryVoxelKey key) {
  return {submap, key, {{static_cast<double>(submap), 2, 3}, 4}};
}

TEST(GeometryMapSamplingTest, HasStableSignedKeyRankAndCollisionOrder) {
  const auto rank = MakeGeometryMapRank(3, {-1, 2, 0});
  EXPECT_EQ(rank.hash, 733970706543307389ULL);
  const GeometryMapRank first{42, 1, {-2, 5, 9}};
  const GeometryMapRank second{42, 1, {-1, 5, 9}};
  const GeometryMapRank third{42, 2, {-9, 0, 0}};
  EXPECT_LT(first, second);
  EXPECT_LT(second, third);
  EXPECT_FALSE(third < first);
}

TEST(GeometryMapSamplingTest, SelectsAcrossBlocksIndependentOfTraversalOrder) {
  std::vector<GeometryMapCandidate> candidates{
      Candidate(0, {0, 0, 0}), Candidate(1, {0, 0, 0}), Candidate(2, {0, 0, 0}),
      Candidate(3, {-1, 2, 0}), Candidate(1, {-2, 5, 9})};
  GeometryMapSampler forward(2);
  for (const auto& candidate : candidates) forward.Add(candidate);
  const auto expected = forward.Take();
  ASSERT_EQ(expected.size(), 2u);
  EXPECT_EQ(expected[0].submap_id, 1u);
  EXPECT_EQ(expected[0].key.x, -2);
  EXPECT_EQ(expected[1].submap_id, 3u);
  EXPECT_EQ(expected[1].key.x, -1);
  std::reverse(candidates.begin(), candidates.end());
  GeometryMapSampler backward(2);
  for (const auto& candidate : candidates) backward.Add(candidate);
  const auto actual = backward.Take();
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t i = 0; i < actual.size(); ++i) {
    EXPECT_EQ(actual[i].submap_id, expected[i].submap_id);
    EXPECT_TRUE(actual[i].key == expected[i].key);
    EXPECT_TRUE(actual[i].point.position.isApprox(expected[i].point.position));
  }
}

TEST(GeometryMapSamplingTest, HandlesEmptySingleAndCompleteSelection) {
  GeometryMapSampler empty(0);
  empty.Add(Candidate(0, {0, 0, 0}));
  EXPECT_TRUE(empty.Take().empty());
  GeometryMapSampler one(1);
  one.Add(Candidate(0, {0, 0, 0}));
  one.Add(Candidate(3, {-1, 2, 0}));
  const auto selected = one.Take();
  ASSERT_EQ(selected.size(), 1u);
  EXPECT_EQ(selected[0].submap_id, 3u);
  GeometryMapSampler all(3);
  all.Add(Candidate(2, {0, 0, 0}));
  all.Add(Candidate(0, {0, 0, 0}));
  const auto complete = all.Take();
  ASSERT_EQ(complete.size(), 2u);
  EXPECT_EQ(complete[0].submap_id, 0u);
  EXPECT_EQ(complete[1].submap_id, 2u);
}

TEST(GeometryMapSamplingTest, RejectsNonfiniteCandidateValues) {
  GeometryMapSampler sampler(1);
  auto point = Candidate(0, {0, 0, 0});
  point.point.position.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(sampler.Add(point), std::invalid_argument);
  point = Candidate(0, {0, 0, 0});
  point.point.intensity = std::numeric_limits<double>::infinity();
  EXPECT_THROW(sampler.Add(point), std::invalid_argument);
  EXPECT_TRUE(sampler.Take().empty());
}

TEST(GeometryMapSamplingTest, LimitsPointsWithoutUnsignedOverflow) {
  GeometryMapOptions options;
  EXPECT_EQ(GeometryMapPointLimit(options, 13), 100000u);
  options.max_message_bytes = 4096 + 128 * 3 + 16 * 2;
  EXPECT_EQ(GeometryMapPointLimit(options, 3), 2u);
  --options.max_message_bytes;
  EXPECT_EQ(GeometryMapPointLimit(options, 3), 1u);
  options.max_message_bytes = 4096;
  EXPECT_EQ(GeometryMapPointLimit(options, 1), 0u);
  EXPECT_EQ(
      GeometryMapPointLimit(options, std::numeric_limits<std::uint64_t>::max()),
      0u);
}

TEST(GeometryMapSamplingTest, ValidatesConfiguredLimits) {
  GeometryMapOptions options;
  EXPECT_NO_THROW(ValidateGeometryMapOptions(options));
  options.max_cached_points = 0;
  EXPECT_NO_THROW(ValidateGeometryMapOptions(options));
  options.max_points = 0;
  EXPECT_THROW(ValidateGeometryMapOptions(options), std::invalid_argument);
  options = {};
  options.max_cached_points = 3000001;
  EXPECT_THROW(ValidateGeometryMapOptions(options), std::invalid_argument);
  options = {};
  options.max_message_bytes = 4095;
  EXPECT_THROW(ValidateGeometryMapOptions(options), std::invalid_argument);
  options.max_message_bytes = 67108865;
  EXPECT_THROW(ValidateGeometryMapOptions(options), std::invalid_argument);
}

}  // namespace
}  // namespace livo_atlas::geometry_map_detail
