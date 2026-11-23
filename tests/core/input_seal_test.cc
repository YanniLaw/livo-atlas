#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <stdexcept>
#include <string>

#include "core/fixtures.h"
#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/io/loop_closure_store.h"
#include "livo_atlas/io/online_submap_store.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;

std::map<std::string, std::string> Files(const fs::path& root) {
  std::map<std::string, std::string> result;
  for (const auto& entry : fs::recursive_directory_iterator(root)) {
    if (!entry.is_regular_file()) continue;
    std::ifstream input(entry.path(), std::ios::binary);
    result.emplace(fs::relative(entry.path(), root).string(),
                   std::string(std::istreambuf_iterator<char>(input), {}));
  }
  return result;
}

PoseUpdate Update(const PoseSnapshot& current) {
  PoseUpdate update{current.version, current.frame_count, current.poses,
                    PoseTailPolicy::kProjectedLastOptimizedAnchor};
  for (auto& pose : update.poses) pose.map_from_body.translation.x() += 1;
  return update;
}

class InputSealTest : public testing::Test {
 protected:
  void SetUp() override {
    options_ = test::Options(FrontendMode::kLidarOnly);
    options_.keyframes.enabled = options_.submaps.enabled = true;
    options_.keyframes.time_threshold = 0;
    options_.submap_mode = SubmapMode::kOnline;
    options_.submaps.max_keyframes = 32;
  }

  void TearDown() override { fs::remove_all(options_.exporter.directory); }

  void Populate(LivoSystem& system) {
    for (int frame = 0; frame < 4; ++frame) {
      ASSERT_TRUE(system.AddLidar(test::Scan(100 + frame * .25)));
      system.ProcessAvailable({});
    }
    ASSERT_GT(system.GetKeyframeReadView().frame_count(), 0u);
  }

  void ExpectClosedInput(LivoSystem& system) {
    EXPECT_THROW(system.AddLidar(test::Scan(102)), std::logic_error);
    EXPECT_THROW(system.AddImu({}), std::logic_error);
    EXPECT_THROW(system.AddImage({}), std::logic_error);
    EXPECT_THROW(system.ProcessNext({}), std::logic_error);
    EXPECT_THROW(system.ProcessAvailable({}), std::logic_error);
    EXPECT_THROW(system.PropagateLatest(), std::logic_error);
  }

  SystemOptions options_;
};

TEST_F(InputSealTest, SealAllowsCommitBeforeFinish) {
  LivoSystem system(options_);
  EXPECT_FALSE(system.IsFinished());
  int seals = 0, pose_events = 0;
  system.SetOnlineSubmapCallback([&](const auto& update) {
    for (const auto& group : update.revisions)
      if (group.state == OnlineSubmapState::kSealed) ++seals;
    if (!update.pose_changes.empty()) ++pose_events;
  });
  Populate(system);
  const auto before = system.GetPoseSnapshot();
  system.SealInput();
  system.SealInput();
  EXPECT_FALSE(system.IsFinished());
  EXPECT_EQ(seals, 1);
  ExpectClosedInput(system);
  system.ApplyPoseUpdate(Update(*before));
  EXPECT_EQ(system.GetPoseSnapshot()->version, 1u);
  EXPECT_EQ(system.GetPoseSnapshot()->frame_count, before->frame_count);
  EXPECT_EQ(pose_events, 1);
  system.Finish();
  system.Finish();
  EXPECT_TRUE(system.IsFinished());
  EXPECT_EQ(seals, 1);
  ExpectClosedInput(system);
  EXPECT_THROW(system.ApplyPoseUpdate(Update(*system.GetPoseSnapshot())),
               std::logic_error);
}

TEST_F(InputSealTest, EmptyAndLegacyFinishRemainCompatible) {
  for (int variant = 0; variant < 3; ++variant) {
    auto options = options_;
    options.exporter.directory /= std::to_string(variant);
    options.submaps.enabled = variant != 1;
    options.submap_mode =
        variant == 2 ? SubmapMode::kKeyframeGroups : SubmapMode::kOnline;
    LivoSystem system(options);
    if (variant) Populate(system);
    const auto count = system.GetKeyframeReadView().frame_count();
    system.Finish();
    const auto saved = Files(options.exporter.directory);
    system.Finish();
    system.SealInput();
    EXPECT_EQ(Files(options.exporter.directory), saved);
    EXPECT_EQ(system.GetKeyframeReadView().frame_count(), count);
    ExpectClosedInput(system);
  }
}

TEST_F(InputSealTest, SealFailureIsTerminal) {
  LivoSystem system(options_);
  Populate(system);
  const auto before = system.GetPoseSnapshot();
  const auto dataset = options_.exporter.directory / "keyframes";
  KeyframeStore frames(dataset);
  OnlineSubmapStore store(dataset / "online_submaps", frames);
  const auto revision = store.Read(0).membership_revision + 1;
  const auto pending = dataset / "online_submaps/records/0" /
                       (".pending-" + std::to_string(revision) + ".yml");
  fs::create_directory(pending);
  const auto saved = Files(dataset);
  std::string failure;
  try {
    system.SealInput();
    FAIL() << "Blocked seal must fail";
  } catch (const std::runtime_error& error) {
    failure = error.what();
  }
  for (int operation = 0; operation < 4; ++operation) {
    try {
      if (operation == 0) system.SealInput();
      if (operation == 1) system.Finish();
      if (operation == 2) system.AddImu({});
      if (operation == 3) system.ApplyPoseUpdate(Update(*before));
      ADD_FAILURE() << "Terminal seal failure must be rethrown";
    } catch (const std::runtime_error& error) {
      EXPECT_EQ(error.what(), failure);
    }
  }
  EXPECT_EQ(system.GetPoseSnapshot()->version, 0u);
  EXPECT_FALSE(system.IsFinished());
  EXPECT_EQ(Files(dataset), saved);
}

TEST_F(InputSealTest, InputOverflowKeepsCommittedViewsAndAnIncompleteJournal) {
  options_.sensor_buffer.max_lidar_scans = 1;
  options_.loop_closure.emplace();
  const auto dataset = options_.exporter.directory / "keyframes";
  std::string failure;
  {
    LivoSystem system(options_);
    Populate(system);
    const auto before = system.GetPoseSnapshot();
    ASSERT_TRUE(system.AddLidar(test::Scan(101)));
    try {
      system.AddLidar(test::Scan(101.25));
      FAIL() << "A full LiDAR queue should fail";
    } catch (const std::overflow_error& error) {
      failure = error.what();
    }
    EXPECT_NE(failure.find("LiDAR queue queued=1 max_count=1"),
              std::string::npos);
    EXPECT_NE(failure.find("oldest_timestamp=101"), std::string::npos);
    const auto expect_failure = [&failure](const auto& operation) {
      try {
        operation();
        ADD_FAILURE() << "The first overflow should be rethrown";
      } catch (const std::overflow_error& error) {
        EXPECT_EQ(error.what(), failure);
      }
    };
    expect_failure([&] { system.AddLidar(test::Scan(102)); });
    expect_failure([&] { system.AddImu({}); });
    expect_failure([&] { system.AddImage({}); });
    expect_failure([&] { system.ProcessNext({}); });
    expect_failure([&] { system.PropagateLatest(); });
    expect_failure([&] { system.SealInput(); });
    expect_failure([&] { system.Finish(); });
    expect_failure([&] { system.Finish(); });
    EXPECT_FALSE(system.IsFinished());
    EXPECT_EQ(system.GetPoseSnapshot()->version, before->version);
    EXPECT_EQ(system.GetPoseSnapshot()->frame_count, before->frame_count);
    EXPECT_EQ(system.GetKeyframeReadView().frame_count(), before->frame_count);
  }
  const auto report = InspectLoopDataset(dataset);
  EXPECT_EQ(report.status, LoopRecoveryStatus::kIncomplete);
  EXPECT_EQ(report.committed_version, 0u);
  std::ifstream record(dataset / "loop_closure/failure.yml");
  ASSERT_TRUE(record.good());
  const std::string recorded_error(std::istreambuf_iterator<char>(record), {});
  EXPECT_NE(recorded_error.find(failure), std::string::npos);
}

TEST_F(InputSealTest, PublishedPoseSurvivesNotificationFailure) {
  LivoSystem system(options_);
  system.SetOnlineSubmapCallback([](const auto& update) {
    if (!update.keyframe && update.poses->version)
      throw std::runtime_error("pose consumer failed");
  });
  Populate(system);
  system.SealInput();
  EXPECT_THROW(system.ApplyPoseUpdate(Update(*system.GetPoseSnapshot())),
               std::runtime_error);
  EXPECT_EQ(system.GetPoseSnapshot()->version, 1u);
  const auto dataset = options_.exporter.directory / "keyframes";
  const auto saved = Files(dataset);
  EXPECT_THROW(system.SealInput(), std::runtime_error);
  EXPECT_THROW(system.Finish(), std::runtime_error);
  EXPECT_FALSE(system.IsFinished());
  EXPECT_THROW(system.ProcessNext({}), std::runtime_error);
  EXPECT_THROW(system.AddLidar(test::Scan(102)), std::runtime_error);
  EXPECT_EQ(Files(dataset), saved);
  KeyframeStore frames(dataset);
  OnlineSubmapStore store(dataset / "online_submaps", frames);
  EXPECT_EQ(store.ReadPoseSnapshot(1).version, 1u);
}

TEST_F(InputSealTest, LegacyFinishReportsWriterFailureBeforeSealing) {
  options_.exporter.pcd = true;
  LivoSystem system(options_);
  int seals = 0;
  system.SetOnlineSubmapCallback([&](const auto& update) {
    for (const auto& group : update.revisions)
      if (group.state == OnlineSubmapState::kSealed) ++seals;
  });
  Populate(system);
  fs::create_directory(options_.exporter.directory / "pcd/all_raw_points.pcd");
  try {
    system.Finish();
    FAIL() << "Blocked accumulated cloud must fail";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string(error.what()).find("all_raw_points.pcd"),
              std::string::npos);
  }
  EXPECT_EQ(seals, 0);
  const auto dataset = options_.exporter.directory / "keyframes";
  KeyframeStore frames(dataset);
  OnlineSubmapStore store(dataset / "online_submaps", frames);
  EXPECT_EQ(store.Read(0).state, OnlineSubmapState::kActive);
  EXPECT_THROW(system.SealInput(), std::runtime_error);
}

}  // namespace
}  // namespace livo_atlas
