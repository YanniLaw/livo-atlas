#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

#include "core/system_observations.h"
#include "livo_atlas/io/submap_store.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;
std::string Contents(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(input), {}};
}

TEST(SubmapSystemTest, AllModesKeepObservationsAndRawAssetsWhileGrouping) {
  for (auto mode : {FrontendMode::kLidarOnly, FrontendMode::kLidarInertial,
                    FrontendMode::kLidarInertialVisual}) {
    auto options = test::Options(mode);
    options.keyframes.enabled = true;
    options.keyframes.time_threshold = 0;
    const auto original = test::RunSystemWithKeyframes(options);
    const auto directory = options.exporter.directory;
    EXPECT_FALSE(fs::exists(directory / "keyframes/submaps"));
    options.exporter.directory /= "with-submaps";
    options.submaps.enabled = true;
    options.submaps.max_keyframes = 2;
    const auto grouped = test::RunSystemWithKeyframes(options, true);
    EXPECT_EQ(original.states, grouped.states);
    EXPECT_EQ(original.clouds, grouped.clouds);
    EXPECT_EQ(original.image_hashes, grouped.image_hashes);
    ASSERT_EQ(original.results.size(), grouped.results.size());
    for (std::size_t i = 0; i < original.results.size(); ++i) {
      EXPECT_EQ(original.results[i].timestamp, grouped.results[i].timestamp);
      EXPECT_EQ(original.results[i].kind, grouped.results[i].kind);
    }
    const auto old_frames = directory / "keyframes";
    const auto new_frames = options.exporter.directory / "keyframes";
    for (const auto& entry : fs::recursive_directory_iterator(old_frames)) {
      if (entry.is_regular_file())
        EXPECT_EQ(
            Contents(entry.path()),
            Contents(new_frames / fs::relative(entry.path(), old_frames)));
    }
    KeyframeStore dataset(new_frames);
    SubmapStore submaps(new_frames / "submaps", dataset);
    ASSERT_GT(dataset.ids().size(), 2u);
    ASSERT_EQ(submaps.size(), (dataset.ids().size() + 1) / 2);
    std::vector<std::uint64_t> ids;
    for (std::size_t id = 0; id < submaps.size(); ++id) {
      const auto submap = submaps.Read(id);
      ids.insert(ids.end(), submap.keyframe_ids.begin(),
                 submap.keyframe_ids.end());
      const auto cloud = submaps.ReadCloud(id);
      std::size_t points = 0;
      for (auto member : submap.keyframe_ids)
        points += dataset.Read(member).cloud->size();
      EXPECT_EQ(cloud.cloud.size(), points);
    }
    EXPECT_EQ(ids, dataset.ids());
    EXPECT_EQ(submaps.Read(submaps.size() - 1).keyframe_ids.size(),
              (dataset.ids().size() - 1) % 2 + 1);
    fs::remove_all(directory);
  }
}

TEST(SubmapSystemTest, InvalidConfigurationPrecedesAnyOutput) {
  for (bool enabled : {false, true}) {
    for (int limit : {0, -1}) {
      auto options = test::Options(FrontendMode::kLidarOnly);
      options.keyframes.enabled = true;
      options.submaps = {enabled, limit};
      EXPECT_THROW(LivoSystem{options}, std::invalid_argument);
      EXPECT_FALSE(fs::exists(options.exporter.directory));
    }
  }
  auto options = test::Options(FrontendMode::kLidarOnly);
  options.submaps.enabled = true;
  EXPECT_THROW(LivoSystem{options}, std::invalid_argument);
  EXPECT_FALSE(fs::exists(options.exporter.directory));
}

TEST(SubmapSystemTest, ExistingDataKeepsDiagnosticSentinels) {
  auto options = test::Options(FrontendMode::kLidarOnly);
  options.keyframes.enabled = options.submaps.enabled = true;
  const auto directory = options.exporter.directory;
  fs::create_directories(directory / "keyframes/submaps");
  std::ofstream(directory / "mat_pre.txt") << "keep";
  EXPECT_THROW(LivoSystem{options}, std::runtime_error);
  EXPECT_EQ(Contents(directory / "mat_pre.txt"), "keep");
  fs::remove_all(directory);
}

TEST(SubmapSystemTest, EmptyFinishDoesNotCreateAnInventedGroup) {
  auto options = test::Options(FrontendMode::kLidarOnly);
  options.keyframes.enabled = options.submaps.enabled = true;
  {
    LivoSystem system(options);
    system.Finish();
    system.Finish();
  }
  KeyframeStore frames(options.exporter.directory / "keyframes");
  EXPECT_EQ(
      SubmapStore(options.exporter.directory / "keyframes/submaps", frames)
          .size(),
      0u);
  fs::remove_all(options.exporter.directory);
}

template <typename Operation>
std::string ExpectStorageError(Operation operation) {
  try {
    operation();
    ADD_FAILURE() << "A failed submap save must remain visible";
  } catch (const std::runtime_error& error) {
    return error.what();
  }
  return {};
}

void CheckPersistentSaveFailure(bool tail) {
  auto options = test::Options(FrontendMode::kLidarOnly);
  options.keyframes.enabled = true;
  options.keyframes.time_threshold = 0;
  options.submaps = {true, 2};
  const auto directory = options.exporter.directory;
  const auto submaps = directory / "keyframes/submaps";
  {
    LivoSystem system(options);
    int frame = 0;
    const auto feed = [&] {
      ASSERT_TRUE(system.AddLidar(test::Scan(100 + frame++ * .25)));
      system.ProcessAvailable({});
    };
    while (!fs::exists(submaps / "0.yml") && frame < 8) feed();
    ASSERT_TRUE(fs::exists(submaps / "0.yml"));
    if (tail) {
      while (KeyframeStore(directory / "keyframes").ids().size() < 3 &&
             frame < 10)
        feed();
      ASSERT_EQ(KeyframeStore(directory / "keyframes").ids().size(), 3u);
    }
    const auto committed = Contents(submaps / "0.yml");
    fs::rename(submaps, directory / "saved-submaps");
    std::ofstream(submaps) << "blocked directory";
    std::string error;
    if (tail) {
      error = ExpectStorageError([&] { system.Finish(); });
    } else {
      while (error.empty() && frame < 12) {
        try {
          feed();
        } catch (const std::runtime_error& failure) {
          error = failure.what();
        }
      }
    }
    ASSERT_FALSE(error.empty());
    fs::remove(submaps);
    fs::rename(directory / "saved-submaps", submaps);
    EXPECT_EQ(ExpectStorageError([&] { system.Finish(); }), error);
    EXPECT_EQ(ExpectStorageError([&] { system.Finish(); }), error);
    EXPECT_EQ(ExpectStorageError([&] { system.ProcessAvailable({}); }), error);
    EXPECT_EQ(Contents(submaps / "0.yml"), committed);
    KeyframeStore frames(directory / "keyframes");
    SubmapStore reader(submaps, frames);
    EXPECT_EQ(reader.size(), 1u);
    EXPECT_EQ(reader.Read(0).keyframe_ids, (std::vector<std::uint64_t>{0, 1}));
  }
  fs::remove_all(directory);
}

TEST(SubmapSystemTest, FailedTailSaveCannotBeHiddenByRepeatedFinish) {
  CheckPersistentSaveFailure(true);
}
TEST(SubmapSystemTest, FailedFullGroupSaveCannotBeHiddenByFinish) {
  CheckPersistentSaveFailure(false);
}
}  // namespace
}  // namespace livo_atlas
