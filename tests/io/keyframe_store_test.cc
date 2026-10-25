#include "livo_atlas/io/keyframe_store.h"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <opencv2/imgcodecs.hpp>
#include <sstream>
#include <stdexcept>
#include <string>

#include "mapping/keyframe_fixtures.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;

std::string Contents(const fs::path& path) {
  std::ifstream file(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(file), {}};
}
void ReplaceValue(const fs::path& path, const std::string& key,
                  const std::string& value) {
  std::istringstream input(Contents(path));
  std::string line, output;
  bool found = false;
  while (std::getline(input, line)) {
    if (line.find(key + ":") == 0) {
      line = key + ": " + value;
      found = true;
    }
    output += line + "\n";
  }
  ASSERT_TRUE(found) << path;
  std::ofstream(path) << output;
}

class KeyframeStoreTest : public testing::Test {
 protected:
  void SetUp() override {
    root_ = fs::temp_directory_path() /
            ("p5-keyframes-" +
             std::to_string(
                 std::chrono::steady_clock::now().time_since_epoch().count()));
    ASSERT_TRUE(fs::create_directories(root_));
    directory_ = root_ / "dataset";
    calibration_.camera = {"Pinhole",
                           12,
                           8,
                           .5,
                           4.1234567890123456,
                           5.5,
                           6,
                           4,
                           {.1, -.2, .001, -.002}};
    calibration_.imu_from_lidar.translation << .1, .2, .3;
  }
  void TearDown() override { fs::remove_all(root_); }
  Keyframe Frame(std::uint64_t id, bool visual = true) {
    return CaptureKeyframe(
        id, test_data::KeyframeResult(visual ? UpdateKind::kVisual
                                             : UpdateKind::kLidarOnly));
  }
  std::vector<KeyframePose> Poses(std::size_t count, double x = 10) {
    std::vector<KeyframePose> poses;
    for (std::size_t i = 0; i < count; ++i) {
      RigidTransform pose;
      pose.translation << x, 0, 0;
      poses.push_back({i, pose});
    }
    return poses;
  }
  fs::path root_, directory_;
  Calibration calibration_;
};

TEST_F(KeyframeStoreTest,
       ReopensImagesCloudsCalibrationAndExactDoubleMetadata) {
  auto frame = Frame(0);
  frame.timestamp = 1000000000.1234567;
  frame.cloud_timestamp = 1000000000.1234565;
  frame.image_timestamp = 1000000000.1234566;
  {
    KeyframeStore writer(directory_, calibration_);
    writer.Append(frame);
    writer.Append(Frame(1, false));
  }
  KeyframeStore reader(directory_);
  EXPECT_EQ(reader.ids(), (std::vector<std::uint64_t>{0, 1}));
  const auto loaded = reader.Read(0);
  EXPECT_EQ(loaded.timestamp, 1000000000.1234567);
  EXPECT_EQ(loaded.cloud_timestamp, 1000000000.1234565);
  EXPECT_EQ(loaded.image_timestamp, 1000000000.1234566);
  EXPECT_EQ(loaded.inverse_exposure_time, .8);
  EXPECT_EQ(loaded.odom_from_body.rotation, frame.odom_from_body.rotation);
  EXPECT_EQ(loaded.camera_from_odom->translation, Eigen::Vector3d(7, 8, 9));
  EXPECT_EQ(reader.calibration().camera.width, 12);
  EXPECT_EQ(reader.calibration().camera.scale, .5);
  EXPECT_EQ(reader.calibration().camera.fx, 4.1234567890123456);
  EXPECT_EQ(reader.calibration().camera.distortion,
            (std::array<double, 4>{.1, -.2, .001, -.002}));
  EXPECT_EQ(reader.calibration().imu_from_lidar.translation,
            Eigen::Vector3d(.1, .2, .3));
  ASSERT_EQ(loaded.image.rows, 4);
  ASSERT_EQ(loaded.image.cols, 6);
  EXPECT_EQ(loaded.image.at<cv::Vec3b>(3, 5), cv::Vec3b(7, 8, 9));
  ASSERT_EQ(loaded.cloud->size(), 1u);
  EXPECT_FLOAT_EQ(loaded.cloud->front().x, 1);
  EXPECT_FLOAT_EQ(loaded.cloud->front().intensity, 42);
  const auto plain = reader.Read(1);
  EXPECT_FALSE(plain.image_timestamp);
  EXPECT_TRUE(plain.image.empty());
  EXPECT_FALSE(fs::exists(directory_ / "frames/1/image.png"));
}

TEST_F(KeyframeStoreTest,
       OptimizedVersionsRebuildCloudsWithoutOverwritingRawData) {
  KeyframeStore writer(directory_, calibration_);
  writer.Append(Frame(0, false));
  const auto original = Contents(directory_ / "frames/0/frame.yml");
  writer.WritePoses(1, Poses(1));
  constexpr std::uint64_t kLargeVersion = 4294967297ULL;
  writer.WritePoses(kLargeVersion, Poses(1, 20));
  KeyframeStore reader(directory_);
  const auto frame = reader.Read(0);
  const auto poses = reader.ReadPoses(1);
  ASSERT_EQ(poses.size(), 1u);
  EXPECT_EQ(poses[0].id, 0u);
  const auto rebuilt = TransformKeyframeCloud(frame, poses[0].map_from_body);
  EXPECT_FLOAT_EQ(rebuilt[0].x, 11);
  EXPECT_FLOAT_EQ(rebuilt[0].y, 0);
  EXPECT_FLOAT_EQ(rebuilt[0].z, 0);
  EXPECT_EQ(reader.ReadPoses(kLargeVersion)[0].map_from_body.translation.x(),
            20);
  EXPECT_EQ(Contents(directory_ / "frames/0/frame.yml"), original);
  EXPECT_EQ(frame.odom_from_body.translation, Eigen::Vector3d(1, 2, 3));
  EXPECT_FLOAT_EQ(frame.cloud->front().x, 1);
}

TEST_F(KeyframeStoreTest, PoseVersionsKeepTheirCommittedFrameFrontier) {
  KeyframeStore writer(directory_, calibration_);
  writer.Append(Frame(0, false));
  writer.WritePoses(1, Poses(1));
  writer.Append(Frame(1, false));
  writer.WritePoses(2, Poses(2));
  KeyframeStore reader(directory_);
  EXPECT_EQ(reader.ids().size(), 2u);
  EXPECT_EQ(reader.ReadPoses(1).size(), 1u);
  EXPECT_EQ(reader.ReadPoses(2).size(), 2u);
}

TEST_F(KeyframeStoreTest,
       ExistingDirectoryAndInvalidCalibrationHaveNoSideEffects) {
  fs::create_directory(directory_);
  std::ofstream(directory_ / "keep.txt") << "original";
  EXPECT_THROW(KeyframeStore(directory_, calibration_), std::exception);
  EXPECT_EQ(Contents(directory_ / "keep.txt"), "original");
  calibration_.camera.fx = 0;
  const auto fresh = root_ / "invalid";
  EXPECT_THROW(KeyframeStore(fresh, calibration_), std::invalid_argument);
  EXPECT_FALSE(fs::exists(fresh));
}

TEST_F(KeyframeStoreTest, InvalidFrameDoesNotAdvanceIdsOrCreatePartialFiles) {
  KeyframeStore writer(directory_, calibration_);
  EXPECT_THROW(writer.Append(Frame(1)), std::invalid_argument);
  auto wrong = Frame(0);
  wrong.image = cv::Mat(4, 5, CV_8UC3);
  EXPECT_THROW(writer.Append(wrong), std::invalid_argument);
  EXPECT_TRUE(fs::is_empty(directory_ / "frames"));
  writer.Append(Frame(0));
  const auto original = Contents(directory_ / "frames/0/frame.yml");
  EXPECT_THROW(writer.Append(Frame(0)), std::invalid_argument);
  EXPECT_THROW(writer.Append(Frame(2)), std::invalid_argument);
  wrong = Frame(1);
  wrong.calibration_id = 1;
  EXPECT_THROW(writer.Append(wrong), std::invalid_argument);
  EXPECT_EQ(Contents(directory_ / "frames/0/frame.yml"), original);
  writer.Append(Frame(1));
  EXPECT_EQ(writer.ids().size(), 2u);
}

TEST_F(KeyframeStoreTest,
       PoseVersionsRequireExactlyTheCommittedIdsAndValidPoses) {
  KeyframeStore writer(directory_, calibration_);
  writer.Append(Frame(0, false));
  writer.Append(Frame(1, false));
  EXPECT_THROW(writer.WritePoses(0, Poses(2)), std::invalid_argument);
  EXPECT_THROW(writer.WritePoses(1, Poses(1)), std::invalid_argument);
  auto poses = Poses(2);
  poses[1].id = 0;
  EXPECT_THROW(writer.WritePoses(1, poses), std::invalid_argument);
  poses[1].id = 2;
  EXPECT_THROW(writer.WritePoses(1, poses), std::invalid_argument);
  poses = Poses(2);
  poses[1].map_from_body.rotation.setZero();
  EXPECT_THROW(writer.WritePoses(1, poses), std::invalid_argument);
  EXPECT_TRUE(fs::is_empty(directory_ / "poses"));
  writer.WritePoses(1, Poses(2));
  const auto original = Contents(directory_ / "poses/1.yml");
  EXPECT_THROW(writer.WritePoses(1, Poses(2, 99)), std::invalid_argument);
  EXPECT_EQ(Contents(directory_ / "poses/1.yml"), original);
}

TEST_F(KeyframeStoreTest,
       ReopenedStoreKeepsFramesReadOnlyAndAllowsNewPoseVersion) {
  KeyframeStore writer(directory_, calibration_);
  writer.Append(Frame(0));
  KeyframeStore reader(directory_);
  EXPECT_THROW(reader.Append(Frame(1)), std::logic_error);
  reader.WritePoses(1, Poses(1));
  EXPECT_EQ(reader.ReadPoses(1).size(), 1u);
  EXPECT_THROW(reader.Read(1), std::out_of_range);
  EXPECT_THROW(reader.ReadPoses(0), std::invalid_argument);
  EXPECT_THROW(reader.ReadPoses(2), std::runtime_error);
}

TEST_F(KeyframeStoreTest, PendingDirectoriesAreIgnoredButCommittedGapsFail) {
  KeyframeStore writer(directory_, calibration_);
  writer.Append(Frame(0));
  fs::create_directory(directory_ / "frames/.pending-1");
  std::ofstream(directory_ / "frames/.pending-1/image.png") << "partial";
  EXPECT_EQ(KeyframeStore(directory_).ids().size(), 1u);
  fs::rename(directory_ / "frames/.pending-1", root_ / "interrupted");
  writer.Append(Frame(1));
  fs::rename(directory_ / "frames/0", root_ / "saved-frame");
  EXPECT_THROW(KeyframeStore{directory_}, std::runtime_error);
}

TEST_F(KeyframeStoreTest, RejectsUnknownSchemaAndInvalidFloatingMetadata) {
  KeyframeStore writer(directory_, calibration_);
  writer.Append(Frame(0));
  ReplaceValue(directory_ / "frames/0/frame.yml", "timestamp", "\"nan\"");
  EXPECT_THROW(writer.Read(0), std::runtime_error);
  ReplaceValue(directory_ / "dataset.yml", "schema_version", "999");
  EXPECT_THROW(KeyframeStore{directory_}, std::runtime_error);
}

TEST_F(KeyframeStoreTest, RejectsMissingWrongTypeAndWrongSizedImageAssets) {
  KeyframeStore writer(directory_, calibration_);
  const auto frame = Frame(0);
  writer.Append(frame);
  const auto file = directory_ / "frames/0/image.png";
  fs::remove(file);
  EXPECT_THROW(writer.Read(0), std::runtime_error);
  ASSERT_TRUE(
      cv::imwrite(file.string(), cv::Mat(4, 6, CV_8UC1, cv::Scalar(1))));
  EXPECT_THROW(writer.Read(0), std::runtime_error);
  ASSERT_TRUE(
      cv::imwrite(file.string(), cv::Mat(4, 5, CV_8UC3, cv::Scalar(1))));
  EXPECT_THROW(writer.Read(0), std::runtime_error);
  ASSERT_TRUE(cv::imwrite(file.string(), frame.image));
  EXPECT_EQ(writer.Read(0).image.at<cv::Vec3b>(0, 0), cv::Vec3b(7, 8, 9));
}

TEST_F(KeyframeStoreTest, RejectsTruncatedMissingAndWrongFieldClouds) {
  KeyframeStore writer(directory_, calibration_);
  writer.Append(Frame(0, false));
  const auto path = directory_ / "frames/0/cloud.pcd";
  const auto original = Contents(path);
  fs::resize_file(path, original.size() - 1);
  EXPECT_THROW(writer.Read(0), std::runtime_error);
  fs::remove(path);
  EXPECT_THROW(writer.Read(0), std::runtime_error);
  std::ofstream file(path, std::ios::binary);
  file << "VERSION 0.7\nFIELDS x y q intensity\nSIZE 4 4 4 4\nTYPE F F F F\n"
          "COUNT 1 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA binary\n";
  const std::string body(16, '\0');
  file.write(body.data(), body.size());
  file.close();
  EXPECT_THROW(writer.Read(0), std::runtime_error);
}

TEST_F(KeyframeStoreTest, OversizedCloudHeaderFailsBeforePointAllocation) {
  KeyframeStore writer(directory_, calibration_);
  writer.Append(Frame(0, false));
  ReplaceValue(directory_ / "frames/0/frame.yml", "cloud_points",
               "\"1000000000\"");
  std::ofstream file(directory_ / "frames/0/cloud.pcd", std::ios::binary);
  file << "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n"
          "COUNT 1 1 1 1\nWIDTH 1000000000\nHEIGHT 1\nPOINTS 1000000000\nDATA "
          "binary\n";
  file.close();
  EXPECT_THROW(writer.Read(0), std::runtime_error);
}

TEST_F(KeyframeStoreTest,
       WriteFailurePreservesCommittedFramesAndLatchesWriter) {
  KeyframeStore writer(directory_, calibration_);
  writer.Append(Frame(0));
  const auto original = Contents(directory_ / "frames/0/frame.yml");
  fs::rename(directory_ / "frames", root_ / "saved-frames");
  std::ofstream(directory_ / "frames") << "blocking file";
  EXPECT_THROW(writer.Append(Frame(1)), std::runtime_error);
  fs::remove(directory_ / "frames");
  fs::rename(root_ / "saved-frames", directory_ / "frames");
  EXPECT_THROW(writer.Append(Frame(1)), std::logic_error);
  EXPECT_EQ(Contents(directory_ / "frames/0/frame.yml"), original);
  KeyframeStore reader(directory_);
  EXPECT_EQ(reader.ids().size(), 1u);
  EXPECT_EQ(reader.Read(0).image.at<cv::Vec3b>(0, 0), cv::Vec3b(7, 8, 9));
}
}  // namespace
}  // namespace livo_atlas
