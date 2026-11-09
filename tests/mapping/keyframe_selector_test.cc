#include "livo_atlas/mapping/keyframe_selector.h"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "mapping/keyframe_fixtures.h"

namespace livo_atlas {
namespace {
KeyframeOptions EnabledOptions() {
  KeyframeOptions options;
  options.enabled = true;
  return options;
}

TEST(KeyframeSelectorTest, DefaultOffAndEachModeUsesItsOwnUpdateStage) {
  KeyframeSelector disabled(FrontendMode::kLidarInertialVisual, {});
  EXPECT_FALSE(disabled.Select(test_data::KeyframeResult()));
  for (auto mode : {FrontendMode::kLidarOnly, FrontendMode::kLidarInertial,
                    FrontendMode::kLidarInertialVisual}) {
    KeyframeSelector selector(mode, EnabledOptions());
    const auto expected =
        mode == FrontendMode::kLidarOnly       ? UpdateKind::kLidarOnly
        : mode == FrontendMode::kLidarInertial ? UpdateKind::kLidarInertial
                                               : UpdateKind::kVisual;
    for (auto kind : {UpdateKind::kLidarOnly, UpdateKind::kLidarInertial,
                      UpdateKind::kVisual}) {
      EXPECT_EQ(selector.Select(test_data::KeyframeResult(kind)),
                kind == expected);
    }
  }
}

TEST(KeyframeSelectorTest, MeasuresTranslationFromTheLastSelectedFrame) {
  KeyframeSelector selector(FrontendMode::kLidarOnly, EnabledOptions());
  auto result = test_data::KeyframeResult(UpdateKind::kLidarOnly);
  result.estimate.pos_end.setZero();
  EXPECT_TRUE(selector.Select(result));
  for (double x : {.2, .4}) {
    result.timestamp += .1;
    result.estimate.pos_end.x() = x;
    EXPECT_FALSE(selector.Select(result));
  }
  result.timestamp += .1;
  result.estimate.pos_end.x() = .5;
  EXPECT_TRUE(selector.Select(result));
  result.timestamp += .1;
  result.estimate.pos_end.x() = .8;
  EXPECT_FALSE(selector.Select(result));
}

TEST(KeyframeSelectorTest, RotationOrElapsedTimeCanSelectWithoutTranslation) {
  auto options = EnabledOptions();
  options.translation_threshold = 100;
  options.time_threshold = 100;
  KeyframeSelector rotate(FrontendMode::kLidarOnly, options);
  auto result = test_data::KeyframeResult(UpdateKind::kLidarOnly);
  result.estimate.rot_end.setIdentity();
  ASSERT_TRUE(rotate.Select(result));
  result.timestamp += .1;
  result.estimate.rot_end =
      Eigen::AngleAxisd(5 * std::acos(-1) / 180, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  EXPECT_FALSE(rotate.Select(result));
  result.timestamp += .1;
  result.estimate.rot_end =
      Eigen::AngleAxisd(11 * std::acos(-1) / 180, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  EXPECT_TRUE(rotate.Select(result));
  options.time_threshold = 2;
  KeyframeSelector timer(FrontendMode::kLidarOnly, options);
  result.timestamp = 100;
  ASSERT_TRUE(timer.Select(result));
  result.timestamp = 101.99;
  EXPECT_FALSE(timer.Select(result));
  result.timestamp = 102;
  EXPECT_TRUE(timer.Select(result));
}

TEST(KeyframeSelectorTest, DuplicateTimeIsSkippedAndRollbackStartsNewAnchor) {
  KeyframeSelector selector(FrontendMode::kLidarOnly, EnabledOptions());
  auto result = test_data::KeyframeResult(UpdateKind::kLidarOnly);
  result.timestamp = 10;
  ASSERT_TRUE(selector.Select(result));
  EXPECT_FALSE(selector.Select(result));
  result.timestamp = 9;
  EXPECT_TRUE(selector.Select(result));
  result.timestamp = 10.9;
  EXPECT_FALSE(selector.Select(result));
  result.timestamp = 11;
  EXPECT_TRUE(selector.Select(result));
}

TEST(KeyframeSelectorTest, BadCandidatesDoNotAdvanceTheSelectionAnchor) {
  KeyframeSelector selector(FrontendMode::kLidarInertialVisual,
                            EnabledOptions());
  auto result = test_data::KeyframeResult();
  result.estimate.pos_end.setZero();
  ASSERT_TRUE(selector.Select(result));
  result.timestamp += .1;
  result.estimate.pos_end.x() = .6;
  result.color_image = cv::Mat(4, 6, CV_8UC1);
  EXPECT_THROW(selector.Select(result), std::invalid_argument);
  result.color_image = cv::Mat(4, 6, CV_8UC3);
  result.timestamp += .1;
  result.estimate.pos_end.x() = .1;
  EXPECT_FALSE(selector.Select(result));
  result.estimate.pos_end.x() = .5;
  EXPECT_TRUE(selector.Select(result));
}

TEST(KeyframeSelectorTest, ValidatesThresholdsAndZeroSelectsEveryNewTime) {
  for (int field = 0; field < 3; ++field) {
    for (double bad : {-1., std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity()}) {
      auto options = EnabledOptions();
      if (field == 0) options.translation_threshold = bad;
      if (field == 1) options.rotation_threshold_deg = bad;
      if (field == 2) options.time_threshold = bad;
      EXPECT_THROW(KeyframeSelector(FrontendMode::kLidarOnly, options),
                   std::invalid_argument);
    }
  }
  auto options = EnabledOptions();
  options.time_threshold = 0;
  KeyframeSelector selector(FrontendMode::kLidarOnly, options);
  auto result = test_data::KeyframeResult(UpdateKind::kLidarOnly);
  ASSERT_TRUE(selector.Select(result));
  result.timestamp += .001;
  EXPECT_TRUE(selector.Select(result));
}
}  // namespace
}  // namespace livo_atlas
