#include "livo_atlas/mapping/submap.h"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <vector>

namespace livo_atlas {
namespace {
TEST(SubmapBuilderTest, EmitsIndependentFullGroupsAndTheFinalPartialGroup) {
  SubmapBuilder builder(2);
  EXPECT_FALSE(builder.AddKeyframe(0));
  const auto first = builder.AddKeyframe(1);
  ASSERT_TRUE(first);
  EXPECT_EQ(first->id, 0u);
  EXPECT_EQ(first->anchor_keyframe_id, 0u);
  EXPECT_EQ(first->keyframe_ids, (std::vector<std::uint64_t>{0, 1}));
  EXPECT_FALSE(builder.AddKeyframe(2));
  const auto second = builder.AddKeyframe(3);
  ASSERT_TRUE(second);
  EXPECT_EQ(second->id, 1u);
  EXPECT_EQ(second->anchor_keyframe_id, 2u);
  EXPECT_FALSE(builder.AddKeyframe(4));
  const auto tail = builder.Finish();
  ASSERT_TRUE(tail);
  EXPECT_EQ(tail->id, 2u);
  EXPECT_EQ(tail->keyframe_ids, (std::vector<std::uint64_t>{4}));
  EXPECT_EQ(first->keyframe_ids, (std::vector<std::uint64_t>{0, 1}));
}

TEST(SubmapBuilderTest, InvalidIdentifiersDoNotAdvanceTheActiveGroup) {
  SubmapBuilder builder(2);
  EXPECT_THROW(builder.AddKeyframe(1), std::invalid_argument);
  EXPECT_FALSE(builder.AddKeyframe(0));
  EXPECT_THROW(builder.AddKeyframe(0), std::invalid_argument);
  EXPECT_THROW(builder.AddKeyframe(2), std::invalid_argument);
  ASSERT_TRUE(builder.AddKeyframe(1));
  EXPECT_FALSE(builder.Finish());
}

TEST(SubmapBuilderTest, FinishIsIdempotentAndPreventsFurtherInput) {
  SubmapBuilder empty(2);
  EXPECT_FALSE(empty.Finish());
  EXPECT_FALSE(empty.Finish());
  EXPECT_THROW(empty.AddKeyframe(0), std::logic_error);
  SubmapBuilder builder(1);
  ASSERT_TRUE(builder.AddKeyframe(0));
  EXPECT_FALSE(builder.Finish());
  EXPECT_FALSE(builder.Finish());
  EXPECT_THROW(builder.AddKeyframe(1), std::logic_error);
}

TEST(SubmapBuilderTest, RequiresPositiveGroupLimits) {
  EXPECT_THROW(SubmapBuilder(0), std::invalid_argument);
  EXPECT_THROW(SubmapBuilder(-1), std::invalid_argument);
  SubmapOptions options;
  options.max_keyframes = 0;
  EXPECT_THROW(ValidateSubmapOptions(options), std::invalid_argument);
}

TEST(SubmapTest, RequiresAnOrderedContinuousMembershipAndItsFirstAnchor) {
  const auto maximum = std::numeric_limits<std::uint64_t>::max();
  const Submap large{4294967297ULL, maximum - 1, {maximum - 1, maximum}};
  EXPECT_NO_THROW(ValidateSubmap(large));
  EXPECT_THROW(ValidateSubmap(Submap{0, 0, {}}), std::invalid_argument);
  EXPECT_THROW(ValidateSubmap(Submap{0, 1, {0, 1}}), std::invalid_argument);
  EXPECT_THROW(ValidateSubmap(Submap{0, 0, {0, 0}}), std::invalid_argument);
  EXPECT_THROW(ValidateSubmap(Submap{0, 0, {0, 2}}), std::invalid_argument);
  EXPECT_THROW(ValidateSubmap(Submap{0, maximum, {maximum, 0}}),
               std::invalid_argument);
}
}  // namespace
}  // namespace livo_atlas
