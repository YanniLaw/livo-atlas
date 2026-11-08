#include "livo_atlas/mapping/online_submap.h"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace livo_atlas {
namespace {
TEST(OnlineSubmapTest, CountLimitProducesInsertionThenSealingRevision) {
  OnlineSubmapBuilder builder(2, {});
  EXPECT_EQ(builder.AddKeyframe(0, 0, {}).front().membership_revision, 1u);
  const auto revisions = builder.AddKeyframe(1, 1, {});
  ASSERT_EQ(revisions.size(), 2u);
  EXPECT_EQ(revisions[0].state, OnlineSubmapState::kActive);
  EXPECT_EQ(revisions[1].state, OnlineSubmapState::kSealed);
  EXPECT_EQ(revisions[1].membership_revision, 3u);
  EXPECT_EQ(revisions[1].primary.keyframe_ids,
            (std::vector<std::uint64_t>{0, 1}));
  const auto next = builder.AddKeyframe(2, 2, {});
  EXPECT_EQ(next.front().support_keyframe_ids,
            (std::vector<std::uint64_t>{0, 1}));
  EXPECT_EQ(next.front().primary.anchor_keyframe_id, 2u);
  EXPECT_EQ(builder.Finish().front().membership_revision, 2u);
  EXPECT_TRUE(builder.Finish().empty());
  EXPECT_THROW(builder.AddKeyframe(3, 3, {}), std::logic_error);
}
TEST(OnlineSubmapTest, ThresholdEqualityStartsNewGroup) {
  for (int dimension = 0; dimension < 3; ++dimension) {
    for (double fraction : {.999, 1.0, 1.001}) {
      OnlineSubmapBuilder builder(20, {});
      builder.AddKeyframe(0, 0, {});
      RigidTransform pose;
      double timestamp = 0;
      if (dimension == 0) pose.translation.x() = 5 * fraction;
      if (dimension == 1)
        pose.rotation = Eigen::AngleAxisd(std::acos(-1.0) / 4 * fraction,
                                          Eigen::Vector3d::UnitZ())
                            .toRotationMatrix();
      if (dimension == 2) timestamp = 20 * fraction;
      const auto result = builder.AddKeyframe(1, timestamp, pose);
      EXPECT_EQ(result.size(), fraction < 1 ? 1u : 2u);
      EXPECT_EQ(result.back().primary.id, fraction < 1 ? 0u : 1u);
    }
  }
}
TEST(OnlineSubmapTest, UsesShortestRotation) {
  OnlineSubmapBuilder builder(20, {});
  RigidTransform pose;
  const double radians = std::acos(-1.0) / 180;
  pose.rotation = Eigen::AngleAxisd(179 * radians, Eigen::Vector3d::UnitZ())
                      .toRotationMatrix();
  builder.AddKeyframe(0, 0, pose);
  pose.rotation = Eigen::AngleAxisd(-179 * radians, Eigen::Vector3d::UnitZ())
                      .toRotationMatrix();
  EXPECT_EQ(builder.AddKeyframe(1, 1, pose).back().primary.id, 0u);
}
TEST(OnlineSubmapTest, RejectedInputDoesNotAdvance) {
  OnlineSubmapBuilder builder(20, {});
  EXPECT_THROW(builder.AddKeyframe(1, 0, {}), std::invalid_argument);
  builder.AddKeyframe(0, 1, {});
  EXPECT_THROW(builder.AddKeyframe(0, 1, {}), std::invalid_argument);
  EXPECT_THROW(builder.AddKeyframe(2, 1, {}), std::invalid_argument);
  EXPECT_THROW(builder.AddKeyframe(1, 0, {}), std::invalid_argument);
  for (double value : {std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_THROW(builder.AddKeyframe(1, value, {}), std::invalid_argument);
    RigidTransform pose;
    pose.translation.x() = value;
    EXPECT_THROW(builder.AddKeyframe(1, 1, pose), std::invalid_argument);
  }
  RigidTransform bad;
  bad.rotation.setZero();
  EXPECT_THROW(builder.AddKeyframe(1, 1, bad), std::invalid_argument);
  EXPECT_EQ(builder.AddKeyframe(1, 1, {}).back().primary.keyframe_ids.size(),
            2u);
}
TEST(OnlineSubmapTest, ValidatesOptionsAndEmptyFinish) {
  OnlineSubmapBuilder empty(20, {});
  EXPECT_TRUE(empty.Finish().empty());
  for (int limit : {0, -1})
    EXPECT_THROW((OnlineSubmapBuilder{limit, {}}), std::invalid_argument);
  for (int field = 0; field < 3; ++field) {
    for (double value : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()}) {
      OnlineSubmapOptions options;
      if (field == 0) options.translation_threshold = value;
      if (field == 1) options.rotation_threshold_deg = value;
      if (field == 2) options.time_threshold = value;
      EXPECT_THROW((OnlineSubmapBuilder{20, options}), std::invalid_argument);
    }
  }
  for (int count : {-1, 21}) {
    OnlineSubmapOptions options;
    options.support_keyframes = count;
    EXPECT_THROW((OnlineSubmapBuilder{20, options}), std::invalid_argument);
  }
  OnlineSubmapOptions options;
  options.support_keyframes = 0;
  OnlineSubmapBuilder single(1, options);
  EXPECT_EQ(single.AddKeyframe(0, 0, {}).size(), 2u);
  EXPECT_TRUE(single.AddKeyframe(1, 1, {}).back().support_keyframe_ids.empty());
  EXPECT_TRUE(single.Finish().empty());
}
}  // namespace
}  // namespace livo_atlas
