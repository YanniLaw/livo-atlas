#include "livo_atlas/gaussian/holdout_split.h"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

#include "livo_atlas/core/online_gaussian_options.h"

namespace livo_atlas {
namespace {

class HoldoutSplitTest : public testing::Test {
 protected:
  void SetUp() override {
    path_ = std::filesystem::temp_directory_path() /
            ("holdout-" +
             std::to_string(
                 std::chrono::steady_clock::now().time_since_epoch().count()) +
             ".json");
    std::ofstream out(path_);
    out << R"({"schema":1,"kind":"livo_atlas.gaussian_split","split_id":")"
        << std::string(64, 'a') << R"(","source":{},"frames":[)";
    for (int id = 0; id < 3; ++id) {
      if (id) out << ',';
      out << "{\"keyframe_id\":\"" << id
          << "\",\"calibration_id\":\"0\",\"submap_id\":\"0\",\"role\":\""
          << (id == 1 ? "heldout" : "train")
          << "\",\"state_timestamp\":" << 100 + id
          << ",\"cloud_timestamp\":" << 100 + id
          << ",\"image_timestamp\":" << 100 + id << '}';
    }
    out << "]}";
  }

  void TearDown() override { std::filesystem::remove(path_); }

  Keyframe Frame(std::uint64_t id) {
    Keyframe frame;
    frame.id = id;
    frame.timestamp = frame.cloud_timestamp = 100 + id;
    frame.image_timestamp = 100 + id;
    return frame;
  }

  std::filesystem::path path_;
};

TEST_F(HoldoutSplitTest, ValidatesEveryFrameBeforeChoosingTrainingRole) {
  const auto split = GaussianHoldoutSplit::Read(path_);
  EXPECT_TRUE(split.ValidateFrame(Frame(0), 0));
  EXPECT_FALSE(split.ValidateFrame(Frame(1), 0));
  EXPECT_TRUE(split.ValidateFrame(Frame(2), 0));
  EXPECT_THROW(split.ValidateFrame(Frame(3), 0), std::invalid_argument);
  auto changed = Frame(0);
  changed.image_timestamp = 101;
  EXPECT_THROW(split.ValidateFrame(changed, 0), std::invalid_argument);
  changed = Frame(0);
  changed.calibration_id = 2;
  EXPECT_THROW(split.ValidateFrame(changed, 0), std::invalid_argument);
  EXPECT_THROW(split.ValidateFrame(Frame(0), 1), std::invalid_argument);
  EXPECT_EQ(split.split_id(), std::string(64, 'a'));
}

TEST_F(HoldoutSplitTest, AdmissionRejectsHeldoutEvenIfItReachesTheMailbox) {
  const auto split = GaussianHoldoutSplit::Read(path_);
  EXPECT_NO_THROW(split.ValidateTraining(Frame(0), 0));
  EXPECT_THROW(split.ValidateTraining(Frame(1), 0), std::invalid_argument);
}

TEST_F(HoldoutSplitTest, EvaluationRequiresExplicitPersistenceAndSplit) {
  OnlineGaussianOptions options;
  options.evaluation_enabled = true;
  EXPECT_THROW(ValidateOnlineGaussianOptions(options), std::invalid_argument);
  options.enabled = options.persist_submaps = true;
  options.socket_path = "/tmp/test.sock";
  options.output_directory = "/tmp/test-output";
  EXPECT_THROW(ValidateOnlineGaussianOptions(options), std::invalid_argument);
  options.evaluation_split_file = path_;
  EXPECT_NO_THROW(ValidateOnlineGaussianOptions(options));
}

}  // namespace
}  // namespace livo_atlas
