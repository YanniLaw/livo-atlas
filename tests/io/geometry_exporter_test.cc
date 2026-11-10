#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <opencv2/core/persistence.hpp>
#include <string>

#include "livo_atlas/io/geometry_exporter.h"
#include "livo_atlas/io/keyframe_store.h"
#include "livo_atlas/io/online_submap_store.h"
#include "mapping/keyframe_fixtures.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;

std::string ReadBytes(const fs::path& path) {
  std::ifstream stream(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(stream), {}};
}

class GeometryExporterTest : public testing::Test {
 protected:
  void SetUp() override {
    root_ = fs::temp_directory_path() /
            ("geometry-export-" +
             std::to_string(
                 std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(root_);
    Calibration calibration;
    calibration.camera = {"Pinhole", 12, 8, .5, 4, 5, 6, 4, {0, 0, 0, 0}};
    keyframes_ =
        std::make_unique<KeyframeStore>(root_ / "keyframes", calibration);
    store_ = std::make_unique<OnlineSubmapStore>(
        root_ / "keyframes/online_submaps", *keyframes_, 2,
        OnlineSubmapOptions{});
    OnlineSubmapBuilder builder(2, {});
    std::vector<RigidTransform> raw;
    for (std::uint64_t id = 0; id < 3; ++id) {
      auto frame = CaptureKeyframe(
          id, test_data::KeyframeResult(UpdateKind::kLidarOnly));
      frame.timestamp += id;
      frame.cloud_timestamp += id;
      frame.odom_from_body.translation.x() += id;
      keyframes_->Append(frame);
      raw.push_back(frame.odom_from_body);
      for (const auto& revision :
           builder.AddKeyframe(id, frame.timestamp, frame.odom_from_body)) {
        store_->AppendRevision(revision);
      }
    }
    PoseUpdate update;
    update.optimized_frame_count = 3;
    for (std::uint64_t id = 0; id < 3; ++id) {
      auto pose = raw[id];
      pose.translation.x() += 10;
      update.poses.push_back({id, pose});
    }
    store_->WritePoseSnapshot(MakePoseSnapshot(update, {}, raw));
    // One committed raw frame deliberately has no online membership yet.
    auto tail =
        CaptureKeyframe(3, test_data::KeyframeResult(UpdateKind::kLidarOnly));
    tail.timestamp += 3;
    tail.cloud_timestamp += 3;
    tail.odom_from_body.translation.x() += 3;
    keyframes_->Append(tail);
  }

  void TearDown() override { fs::remove_all(root_); }

  fs::path root_;
  std::unique_ptr<KeyframeStore> keyframes_;
  std::unique_ptr<OnlineSubmapStore> store_;
};

TEST_F(GeometryExporterTest, ExportsSelectedVersionWithoutChangingSources) {
  const auto dataset = root_ / "keyframes";
  const auto metadata = dataset / "frames/0/frame.yml";
  const auto cloud = dataset / "frames/0/cloud.pcd";
  const auto before_metadata = ReadBytes(metadata);
  const auto before_cloud = ReadBytes(cloud);
  ExportGeometry(dataset, root_ / "v0", 0, {});
  ExportGeometry(dataset, root_ / "v1", 1, {});
  EXPECT_EQ(ReadBytes(metadata), before_metadata);
  EXPECT_EQ(ReadBytes(cloud), before_cloud);
  EXPECT_TRUE(fs::exists(root_ / "v0/manifest.yml"));
  cv::FileStorage manifest((root_ / "v1/manifest.yml").string(),
                           cv::FileStorage::READ);
  EXPECT_EQ(manifest["stored_frame_count"].string(), "3");
  EXPECT_EQ(manifest["frame_count"].string(), "4");
  EXPECT_EQ(manifest["unindexed_frames"].string(), "1");
  EXPECT_TRUE(fs::exists(root_ / "v0/submaps/0/voxels.csv"));
  EXPECT_TRUE(fs::exists(root_ / "v0/submaps/1/contributors.csv"));
  EXPECT_EQ(ReadBytes(root_ / "v0/submaps/0/voxels.csv"),
            ReadBytes(root_ / "v1/submaps/0/voxels.csv"));
  EXPECT_EQ(ReadBytes(root_ / "v0/submaps/0/contributors.csv"),
            ReadBytes(root_ / "v1/submaps/0/contributors.csv"));
  cv::FileStorage v0((root_ / "v0/submaps/0/metadata.yml").string(),
                     cv::FileStorage::READ);
  cv::FileStorage v1((root_ / "v1/submaps/0/metadata.yml").string(),
                     cv::FileStorage::READ);
  EXPECT_EQ(v0["parent_frame"].string(), "odom");
  EXPECT_EQ(v1["parent_frame"].string(), "map");
  EXPECT_THROW(ExportGeometry(dataset, root_ / "missing", 2, {}),
               std::exception);
}

TEST_F(GeometryExporterTest, RefusesExistingOrNestedOutputs) {
  const auto dataset = root_ / "keyframes";
  fs::create_directory(root_ / "existing");
  std::ofstream(root_ / "existing/keep") << "keep";
  EXPECT_THROW(ExportGeometry(dataset, root_ / "existing", 0, {}),
               std::exception);
  EXPECT_EQ(ReadBytes(root_ / "existing/keep"), "keep");
  EXPECT_THROW(ExportGeometry(dataset, dataset / "nested", 0, {}),
               std::exception);
  EXPECT_FALSE(fs::exists(dataset / "nested"));
  fs::create_directory_symlink(dataset, root_ / "source-link");
  EXPECT_THROW(ExportGeometry(dataset, root_ / "source-link/nested", 0, {}),
               std::exception);
  std::ofstream(root_ / "file") << "not a directory";
  EXPECT_THROW(ExportGeometry(dataset, root_ / "file/output", 0, {}),
               std::exception);
  EXPECT_FALSE(fs::exists(root_ / "file/output"));
}

}  // namespace
}  // namespace livo_atlas
