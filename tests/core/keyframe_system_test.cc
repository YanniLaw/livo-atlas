#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <limits>
#include <vector>

#include "core/fixtures.h"
#include "core/system_observations.h"
#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/io/keyframe_store.h"
#include "test_images.h"

namespace livo_atlas {
namespace {
using test::RunSystemWithKeyframes;

TEST(KeyframeSystemTest, CapturingAllModesPreservesEstimatesAndOwnsSnapshots) {
  for (auto mode : {FrontendMode::kLidarOnly, FrontendMode::kLidarInertial,
                    FrontendMode::kLidarInertialVisual}) {
    auto options = test::Options(mode);
    options.exposure_time = .025;
    const auto disabled = RunSystemWithKeyframes(options);
    EXPECT_FALSE(
        std::filesystem::exists(options.exporter.directory / "keyframes"));
    const auto disabled_directory = options.exporter.directory;
    options.exporter.directory /= "enabled";
    options.keyframes.enabled = true;
    options.keyframes.time_threshold = 0;
    const auto enabled = RunSystemWithKeyframes(options);
    ASSERT_GT(enabled.results.size(), 2u);
    EXPECT_EQ(disabled.states, enabled.states);
    EXPECT_EQ(disabled.clouds, enabled.clouds);
    EXPECT_EQ(disabled.image_hashes, enabled.image_hashes);
    ASSERT_EQ(disabled.results.size(), enabled.results.size());
    KeyframeStore dataset(options.exporter.directory / "keyframes");
    std::size_t index = 0;
    double cloud_time = 0;
    for (std::size_t i = 0; i < enabled.results.size(); ++i) {
      const auto& result = enabled.results[i];
      EXPECT_EQ(result.timestamp, disabled.results[i].timestamp);
      EXPECT_EQ(result.kind, disabled.results[i].kind);
      if (result.kind != UpdateKind::kVisual) cloud_time = result.timestamp;
      const bool selected = mode == FrontendMode::kLidarInertialVisual
                                ? result.kind == UpdateKind::kVisual
                                : result.kind != UpdateKind::kVisual;
      if (!selected) continue;
      const auto frame = dataset.Read(index++);
      EXPECT_EQ(frame.timestamp, result.timestamp);
      EXPECT_EQ(frame.cloud_timestamp, cloud_time);
      EXPECT_EQ(frame.image_timestamp, result.image_timestamp);
      EXPECT_EQ(frame.odom_from_body.translation, result.estimate.pos_end);
      const auto world = TransformKeyframeCloud(frame, frame.odom_from_body);
      ASSERT_EQ(world.size() * 4, enabled.clouds[i].size());
      for (std::size_t point = 0; point < world.size(); ++point) {
        EXPECT_NEAR(world[point].x, enabled.clouds[i][point * 4], 1e-5);
        EXPECT_NEAR(world[point].y, enabled.clouds[i][point * 4 + 1], 1e-5);
        EXPECT_NEAR(world[point].z, enabled.clouds[i][point * 4 + 2], 1e-5);
      }
      if (frame.image_timestamp)
        EXPECT_EQ(test_data::ImageHash(frame.image), enabled.image_hashes[i]);
    }
    EXPECT_EQ(dataset.ids().size(), index);
    EXPECT_GT(index, 2u);
    std::filesystem::remove_all(disabled_directory);
  }
}

TEST(KeyframeSystemTest, ExistingDatasetDoesNotTruncateDiagnosticFiles) {
  auto options = test::Options(FrontendMode::kLidarOnly);
  options.keyframes.enabled = true;
  const auto directory = options.exporter.directory;
  std::filesystem::create_directories(directory / "keyframes");
  for (const auto* name : {"mat_pre.txt", "mat_out.txt"})
    std::ofstream(directory / name) << "original";
  EXPECT_THROW(LivoSystem{options}, std::runtime_error);
  for (const auto* name : {"mat_pre.txt", "mat_out.txt"}) {
    std::ifstream stream(directory / name);
    std::string contents;
    stream >> contents;
    EXPECT_EQ(contents, "original");
  }
  std::filesystem::remove_all(directory);
}

TEST(KeyframeSystemTest, InvalidThresholdsFailBeforeAnyOutputIsCreated) {
  for (double value : {-1., std::numeric_limits<double>::infinity()}) {
    auto options = test::Options(FrontendMode::kLidarOnly);
    options.keyframes.enabled = true;
    options.keyframes.time_threshold = value;
    EXPECT_THROW(LivoSystem{options}, std::invalid_argument);
    EXPECT_FALSE(std::filesystem::exists(options.exporter.directory));
  }
}
}  // namespace
}  // namespace livo_atlas
