#include <cmath>
#include <cstdint>
#include <limits>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "livo_atlas/core/frame_id.h"
#include "livo_atlas/core/pose3d.h"
#include "livo_atlas/core/timestamp.h"

namespace livo_atlas {
namespace {

TEST(CoreTypesTest, FrameIdAndTimestampDefaults) {
  FrameId frame_id{};
  EXPECT_EQ(kInvalidFrameId, frame_id);
  EXPECT_EQ(FrameId{0}, kInvalidFrameId);
  EXPECT_EQ(FrameId{1}, kInvalidFrameId + 1);

  Timestamp timestamp{};
  EXPECT_EQ(0, timestamp);
  ASSERT_TRUE(TimestampFromSeconds(0.0, &timestamp));
  EXPECT_EQ(0, timestamp);
}

TEST(CoreTypesTest, SecondsConversionRoundsToNearestNanosecond) {
  Timestamp timestamp = -1;

  // This exactly representable binary value maps to 976562.5 nanoseconds.
  const double half_nanosecond_tie_seconds = 1.0 / 1024.0;
  const double below_tie_seconds =
      std::nextafter(half_nanosecond_tie_seconds, 0.0);
  ASSERT_TRUE(TimestampFromSeconds(below_tie_seconds, &timestamp));
  EXPECT_EQ(976'562, timestamp);

  ASSERT_TRUE(TimestampFromSeconds(half_nanosecond_tie_seconds, &timestamp));
  EXPECT_EQ(976'563, timestamp);

  const double infinity = std::numeric_limits<double>::infinity();
  const double above_tie_seconds =
      std::nextafter(half_nanosecond_tie_seconds, infinity);
  ASSERT_TRUE(TimestampFromSeconds(above_tie_seconds, &timestamp));
  EXPECT_EQ(976'563, timestamp);
}

TEST(CoreTypesTest, SecondsConversionRejectsInvalidInputsWithoutWriting) {
  Timestamp timestamp = 123;
  const double nan_value = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();

  EXPECT_FALSE(TimestampFromSeconds(-1.0, &timestamp));
  EXPECT_EQ(123, timestamp);
  EXPECT_FALSE(TimestampFromSeconds(nan_value, &timestamp));
  EXPECT_EQ(123, timestamp);
  EXPECT_FALSE(TimestampFromSeconds(infinity, &timestamp));
  EXPECT_EQ(123, timestamp);
  EXPECT_FALSE(TimestampFromSeconds(-infinity, &timestamp));
  EXPECT_EQ(123, timestamp);

  const double two_to_63_nanoseconds_in_seconds =
      std::ldexp(1.0, 63) / static_cast<double>(kNanosecondsPerSecond);
  const double above_limit =
      std::nextafter(two_to_63_nanoseconds_in_seconds, infinity);
  const double below_limit =
      std::nextafter(two_to_63_nanoseconds_in_seconds, 0.0);
  ASSERT_TRUE(TimestampFromSeconds(below_limit, &timestamp));
  EXPECT_GT(timestamp, 0);

  timestamp = 123;
  EXPECT_FALSE(TimestampFromSeconds(
      two_to_63_nanoseconds_in_seconds, &timestamp));
  EXPECT_EQ(123, timestamp);
  EXPECT_FALSE(TimestampFromSeconds(above_limit, &timestamp));
  EXPECT_EQ(123, timestamp);
  EXPECT_FALSE(TimestampFromSeconds(1.0e300, &timestamp));
  EXPECT_EQ(123, timestamp);
  EXPECT_FALSE(TimestampFromSeconds(1.0, nullptr));
}

TEST(CoreTypesTest, IntegerSecondsAndNanosecondsConvertExactlyAtLimit) {
  Timestamp timestamp = -1;

  ASSERT_TRUE(TimestampFromSecondsAndNanoseconds(
      2, 345'678'901u, &timestamp));
  EXPECT_EQ(2'345'678'901, timestamp);

  ASSERT_TRUE(TimestampFromSecondsAndNanoseconds(
      9'223'372'036LL, 854'775'807u, &timestamp));
  EXPECT_EQ(std::numeric_limits<Timestamp>::max(), timestamp);

  timestamp = 123;
  EXPECT_FALSE(TimestampFromSecondsAndNanoseconds(
      9'223'372'036LL, 854'775'808u, &timestamp));
  EXPECT_EQ(123, timestamp);
  EXPECT_FALSE(TimestampFromSecondsAndNanoseconds(
      9'223'372'037LL, 0u, &timestamp));
  EXPECT_EQ(123, timestamp);
  EXPECT_FALSE(TimestampFromSecondsAndNanoseconds(-1, 0u, &timestamp));
  EXPECT_EQ(123, timestamp);
  EXPECT_FALSE(TimestampFromSecondsAndNanoseconds(
      0, 1'000'000'000u, &timestamp));
  EXPECT_EQ(123, timestamp);
  EXPECT_FALSE(TimestampFromSecondsAndNanoseconds(0, 0u, nullptr));
}

TEST(CoreTypesTest, PoseCompositionInverseAndPointTransform) {
  Pose3d pose_a_b_quarter_turn = Pose3d::Identity();
  pose_a_b_quarter_turn.linear() =
      Eigen::AngleAxisd(std::acos(-1.0) / 2.0, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  pose_a_b_quarter_turn.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  const Eigen::Vector3d point_b(1.0, 0.0, 0.0);
  const Eigen::Vector3d expected_point_a(1.0, 3.0, 3.0);
  EXPECT_LT((pose_a_b_quarter_turn * point_b - expected_point_a).norm(),
            1e-12);

  Pose3d pose_a_b = Pose3d::Identity();
  pose_a_b.linear() =
      Eigen::AngleAxisd(0.37, Eigen::Vector3d(1.0, 2.0, 3.0).normalized())
          .toRotationMatrix();
  pose_a_b.translation() = Eigen::Vector3d(1.2, -0.4, 2.1);

  Pose3d pose_b_c = Pose3d::Identity();
  pose_b_c.linear() = Eigen::AngleAxisd(-0.61, Eigen::Vector3d::UnitY())
                          .toRotationMatrix();
  pose_b_c.translation() = Eigen::Vector3d(-0.3, 0.8, 0.5);

  const Eigen::Vector3d point_c(0.4, -1.7, 2.3);
  const Eigen::Vector3d point_a_sequential = pose_a_b * (pose_b_c * point_c);

  const Pose3d pose_a_c = pose_a_b * pose_b_c;
  const Eigen::Vector3d point_a_composed = pose_a_c * point_c;
  EXPECT_LT((point_a_composed - point_a_sequential).norm(), 1e-12);

  const Pose3d pose_c_a = pose_a_c.inverse();
  const Eigen::Vector3d point_c_round_trip = pose_c_a * point_a_composed;
  EXPECT_LT((point_c_round_trip - point_c).norm(), 1e-12);
}

}  // namespace
}  // namespace livo_atlas

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
