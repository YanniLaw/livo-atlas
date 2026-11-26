#include "livo_atlas/io/keyframe_read_view.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

#include "core/fixtures.h"
#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/io/keyframe_store.h"
#include "mapping/keyframe_fixtures.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;

class KeyframeReadViewTest : public testing::Test {
 protected:
  void SetUp() override {
    root_ = fs::temp_directory_path() /
            ("c2b-read-view-" +
             std::to_string(
                 std::chrono::steady_clock::now().time_since_epoch().count()));
    directory_ = root_ / "dataset";
    calibration_.camera = {"Pinhole", 12, 8, .5, 4, 5, 6, 4, {}};
  }

  void TearDown() override { fs::remove_all(root_); }

  Keyframe Frame(std::uint64_t id) {
    auto frame = CaptureKeyframe(id, test_data::KeyframeResult());
    frame.timestamp = 100.25 + id;
    frame.cloud_timestamp = 100.2 + id;
    frame.image_timestamp = 100.24 + id;
    frame.odom_from_body.rotation = Eigen::Matrix3d::Identity();
    frame.odom_from_body.rotation(0, 0) = 1.000045;
    return frame;
  }

  fs::path root_, directory_;
  Calibration calibration_;
};

TEST_F(KeyframeReadViewTest, FrozenPrefixSurvivesAppendAndWriterDestruction) {
  std::optional<KeyframeReadView> old;
  {
    KeyframeStore writer(fs::relative(directory_), calibration_);
    const auto empty = writer.CaptureReadView();
    EXPECT_THROW(empty.ReadRawBodyPose(0), std::out_of_range);
    EXPECT_THROW(empty.ReadTimestamp(0), std::out_of_range);
    EXPECT_THROW(empty.ReadBodyCloud(0, 1), std::out_of_range);
    writer.Append(Frame(0));
    old = writer.CaptureReadView();
    writer.Append(Frame(1));
    EXPECT_THROW(old->ReadTimestamp(1), std::out_of_range);
    EXPECT_THROW(old->ReadRawBodyPose(1), std::out_of_range);
    EXPECT_THROW(old->ReadBodyCloud(1, 1), std::out_of_range);
    EXPECT_DOUBLE_EQ(writer.CaptureReadView().ReadTimestamp(1), 101.25);
    EXPECT_EQ(writer.CaptureReadView().frame_count(), 2u);
  }
  ASSERT_TRUE(old);
  EXPECT_TRUE(old->directory().is_absolute());
  EXPECT_EQ(fs::weakly_canonical(old->directory()),
            fs::weakly_canonical(directory_));
  EXPECT_EQ(old->frame_count(), 1u);
  EXPECT_DOUBLE_EQ(old->ReadTimestamp(0), 100.25);
  EXPECT_DOUBLE_EQ(old->ReadRawBodyPose(0).rotation(0, 0), 1.000045);
  const auto cloud = old->ReadBodyCloud(0, 1);
  ASSERT_EQ(cloud->size(), 1u);
  EXPECT_FLOAT_EQ(cloud->front().x, 1);
  EXPECT_FLOAT_EQ(cloud->front().intensity, 42);
}

TEST_F(KeyframeReadViewTest, ReadsWithoutIndexScanOrImage) {
  KeyframeStore writer(directory_, calibration_);
  writer.Append(Frame(0));
  const auto view = writer.CaptureReadView();
  fs::remove(directory_ / "dataset.yml");
  fs::remove(directory_ / "frames/0/image.png");
  fs::create_directories(directory_ / "online_submaps");
  std::ofstream(directory_ / "online_submaps/dataset.yml") << "broken";
  fs::create_directories(directory_ / "frames/.pending-1");
  fs::create_directories(directory_ / "frames/not-an-id");
  EXPECT_DOUBLE_EQ(view.ReadTimestamp(0), 100.25);
  EXPECT_DOUBLE_EQ(view.ReadRawBodyPose(0).rotation(0, 0), 1.000045);
  EXPECT_EQ(view.ReadBodyCloud(0, 1)->size(), 1u);
  EXPECT_THROW(view.ReadBodyCloud(1, 1), std::out_of_range);
}

TEST_F(KeyframeReadViewTest,
       FullFrameReadRejectsOversizedMetadataBeforeParsing) {
  KeyframeStore writer(directory_, calibration_);
  writer.Append(Frame(0));
  const auto view = writer.CaptureReadView();
  std::ofstream out(directory_ / "frames/0/frame.yml", std::ios::app);
  for (int i = 0; i < 10000; ++i) out << "# harmless padding\n";
  out.close();
  EXPECT_THROW(view.Read(0, calibration_, 100000, 1000000),
               std::runtime_error);
}

TEST_F(KeyframeReadViewTest, FullFrameReadUsesOnlyTheCommittedPrefix) {
  KeyframeStore writer(directory_, calibration_);
  writer.Append(Frame(0));
  const auto view = writer.CaptureReadView();
  fs::create_directories(directory_ / "frames/.pending-1");
  std::ofstream(directory_ / "frames/.pending-1/frame.yml") << "incomplete";
  auto frame = view.Read(0, calibration_, 100000, 1000000);
  EXPECT_EQ(frame.id, 0u);
  EXPECT_DOUBLE_EQ(frame.timestamp, 100.25);
  EXPECT_FALSE(frame.image.empty());
  EXPECT_EQ(frame.cloud->size(), Frame(0).cloud->size());
  EXPECT_THROW(view.Read(1, calibration_, 100000, 1000000), std::out_of_range);
  EXPECT_THROW(view.Read(0, calibration_, 100000, 1), std::invalid_argument);
}

TEST_F(KeyframeReadViewTest, SharesPcdBoundsAndRawNumbers) {
  KeyframeStore writer(directory_, calibration_);
  auto frame = Frame(0);
  PointCloud::Ptr cloud(new PointCloud(*frame.cloud));
  cloud->push_back(cloud->front());
  frame.cloud = cloud;
  writer.Append(frame);
  const auto view = writer.CaptureReadView();
  EXPECT_DOUBLE_EQ(view.ReadRawBodyPose(0).rotation(0, 0), 1.000045);
  EXPECT_THROW(view.ReadBodyCloud(0, 0), std::invalid_argument);
  EXPECT_THROW(view.ReadBodyCloud(0, 1), std::runtime_error);
  EXPECT_EQ(view.ReadBodyCloud(0, 2)->size(), 2u);

  const auto path = directory_ / "frames/0/cloud.pcd";
  {
    std::fstream file(path, std::ios::in | std::ios::out | std::ios::binary);
    std::string line;
    while (std::getline(file, line) && line != "DATA binary") {
    }
    ASSERT_TRUE(file);
    const auto offset = file.tellg();
    file.seekp(offset);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    file.write(reinterpret_cast<const char*>(&nan), sizeof(nan));
  }
  EXPECT_THROW(view.ReadBodyCloud(0, 2), std::runtime_error);
  {
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    file << "VERSION .7\nFIELDS x y z intensity extra\n"
            "SIZE 4 4 4 4 1\nTYPE F F F F U\nCOUNT 1 1 1 1 49\n"
            "WIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA binary\n";
    const std::string payload(130, '\0');
    file.write(payload.data(), payload.size());
  }
  EXPECT_THROW(view.ReadBodyCloud(0, 2), std::runtime_error);
}

TEST(KeyframeReadViewSystemTest, ExposesOnlyEnabledCommittedFrames) {
  auto options = test::Options(FrontendMode::kLidarOnly);
  {
    LivoSystem disabled(options);
    EXPECT_THROW(disabled.GetKeyframeReadView(), std::logic_error);
  }
  fs::remove_all(options.exporter.directory);
  options.keyframes.enabled = true;
  options.keyframes.time_threshold = 0;
  std::optional<KeyframeReadView> view;
  {
    LivoSystem system(options);
    EXPECT_THROW(system.GetKeyframeReadView().ReadTimestamp(0),
                 std::out_of_range);
    for (int frame = 0; frame < 3; ++frame) {
      system.AddLidar(test::Scan(100 + frame * .25));
      system.ProcessAvailable({});
    }
    view = system.GetKeyframeReadView();
    ASSERT_GT(view->frame_count(), 0u);
    system.Finish();
  }
  EXPECT_TRUE(std::isfinite(view->ReadTimestamp(0)));
  EXPECT_FALSE(view->ReadBodyCloud(0, 1000000)->empty());
  fs::remove_all(options.exporter.directory);
}

}  // namespace
}  // namespace livo_atlas
