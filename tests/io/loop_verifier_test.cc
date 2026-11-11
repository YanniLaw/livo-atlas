#include "io/loop_verifier.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>

#include <gtest/gtest.h>
#include <opencv2/core/persistence.hpp>

#include "livo_atlas/io/keyframe_store.h"
#include "livo_atlas/io/online_submap_store.h"
#include "mapping/keyframe_fixtures.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;

std::string Bytes(const fs::path& path) {
  std::ifstream file(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(file), {}};
}

class LoopVerifierTest : public testing::Test {
 protected:
  void SetUp() override {
    root_ = fs::temp_directory_path() /
            ("loop-verifier-" +
             std::to_string(
                 std::chrono::steady_clock::now().time_since_epoch().count()));
    dataset_ = root_ / "keyframes";
    fs::create_directories(root_);
    Calibration calibration;
    calibration.camera = {"Pinhole", 12, 8, .5, 4, 5, 6, 4, {0, 0, 0, 0}};
    KeyframeStore frames(dataset_, calibration);
    OnlineSubmapOptions options;
    options.support_keyframes = 0;
    OnlineSubmapStore store(dataset_ / "online_submaps", frames, 1, options);
    OnlineSubmapBuilder builder(1, options);
    for (std::uint64_t id = 0; id < 3; ++id) {
      auto frame = CaptureKeyframe(
          id, test_data::KeyframeResult(UpdateKind::kLidarOnly));
      frame.timestamp += id;
      frame.cloud_timestamp += id;
      frames.Append(frame);
      for (const auto& revision :
           builder.AddKeyframe(id, frame.timestamp, frame.odom_from_body))
        store.AppendRevision(revision);
    }
    for (const auto& revision : builder.Finish())
      store.AppendRevision(revision);
  }

  void TearDown() override { fs::remove_all(root_); }

  LoopVerifierOptions Nearby() const {
    LoopVerifierOptions options;
    options.search.min_submap_gap = 1;
    options.search.min_keyframe_gap = 2;
    options.search.min_time_gap_s = 0.5;
    return options;
  }

  fs::path root_, dataset_;
};

TEST_F(LoopVerifierTest,
       ZeroCandidatesAndRejectedCandidatesAreCompleteResults) {
  const auto metadata = Bytes(dataset_ / "frames/0/frame.yml");
  const auto cloud = Bytes(dataset_ / "frames/0/cloud.pcd");
  VerifyLoopDataset(dataset_, root_ / "zero", {});
  cv::FileStorage zero((root_ / "zero/summary.yml").string(),
                       cv::FileStorage::READ);
  EXPECT_EQ(static_cast<int>(zero["complete"]), 1);
  EXPECT_EQ(zero["candidates"].string(), "0");
  VerifyLoopDataset(dataset_, root_ / "rejected", Nearby());
  cv::FileStorage report((root_ / "rejected/summary.yml").string(),
                         cv::FileStorage::READ);
  EXPECT_EQ(report["candidates"].string(), "1");
  EXPECT_EQ(report["verified"].string(), "0");
  cv::FileStorage match((root_ / "rejected/matches/0-2.yml").string(),
                        cv::FileStorage::READ);
  EXPECT_EQ(match["status"].string(), "insufficient_points");
  EXPECT_EQ(Bytes(dataset_ / "frames/0/frame.yml"), metadata);
  EXPECT_EQ(Bytes(dataset_ / "frames/0/cloud.pcd"), cloud);
}

TEST_F(LoopVerifierTest, DoesNotOverwriteOutputOrWriteInsideDataset) {
  fs::create_directory(root_ / "existing");
  std::ofstream(root_ / "existing/keep") << "keep";
  EXPECT_THROW(VerifyLoopDataset(dataset_, root_ / "existing", {}),
               std::runtime_error);
  EXPECT_EQ(Bytes(root_ / "existing/keep"), "keep");
  EXPECT_FALSE(fs::exists(root_ / "existing/failure.yml"));
  EXPECT_THROW(VerifyLoopDataset(dataset_, dataset_ / "results", {}),
               std::invalid_argument);
  EXPECT_FALSE(fs::exists(dataset_ / "results"));
}

TEST_F(LoopVerifierTest, AssetFailureLeavesExplicitIncompleteReport) {
  fs::remove(dataset_ / "frames/2/cloud.pcd");
  EXPECT_THROW(VerifyLoopDataset(dataset_, root_ / "failed", Nearby()),
               std::runtime_error);
  EXPECT_TRUE(fs::exists(root_ / "failed/failure.yml"));
  cv::FileStorage summary((root_ / "failed/summary.yml").string(),
                          cv::FileStorage::READ);
  EXPECT_EQ(static_cast<int>(summary["complete"]), 0);
}

TEST_F(LoopVerifierTest, ParsesTypedOptionsAndRejectsUnknownOrInvalidValues) {
  const auto config = root_ / "options.yml";
  std::ofstream(config) << "%YAML:1.0\n---\nsearch:\n  max_candidates: "
                           "2\nregistration:\n  max_iterations: 7\n";
  const auto options = ReadLoopVerifierOptions(config);
  EXPECT_EQ(options.search.max_candidates, 2u);
  EXPECT_EQ(options.registration.max_iterations, 7);
  std::ofstream(config) << "%YAML:1.0\n---\nsearch:\n  typo: 2\n";
  EXPECT_THROW(ReadLoopVerifierOptions(config), std::invalid_argument);
  std::ofstream(config) << "%YAML:1.0\n---\nregistration:\n  neighbors: 1.5\n";
  EXPECT_THROW(ReadLoopVerifierOptions(config), std::invalid_argument);
}

}  // namespace
}  // namespace livo_atlas
