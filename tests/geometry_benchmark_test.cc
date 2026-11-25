#include "geometry_benchmark.h"

#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core/persistence.hpp>

#include "livo_atlas/io/keyframe_store.h"
#include "livo_atlas/io/online_submap_store.h"

namespace livo_atlas {
namespace {

class GeometryBenchmarkTest : public testing::Test {
 protected:
  void SetUp() override {
    root_ = std::filesystem::temp_directory_path() /
            ("geometry benchmark " + std::to_string(
                std::chrono::steady_clock::now().time_since_epoch().count()));
    WriteDataset(true, 9);
  }

  void WriteDataset(bool seal, int point_count) {
    Calibration calibration;
    calibration.camera = {"Pinhole", 12, 8, .5, 4, 5, 6, 4, {}};
    KeyframeStore frames(root_, calibration);
    OnlineSubmapOptions options;
    options.support_keyframes = 0;
    OnlineSubmapBuilder builder(2, options);
    OnlineSubmapStore store(root_ / "online_submaps", frames, 2, options);
    PointCloud::Ptr cloud(new PointCloud);
    for (int i = 0; i < point_count; ++i) {
      const int x = i % 3;
      const int y = (i / 3) % 3;
      Point point;
      point.x = .1f + .2f * x;
      point.y = .1f + .2f * y;
      point.z = .1f;
      point.intensity = x + y;
      cloud->push_back(point);
    }
    for (std::uint64_t id = 0; id < 3; ++id) {
      Keyframe frame;
      frame.id = id;
      frame.timestamp = frame.cloud_timestamp = 10 + id;
      frame.odom_from_body.translation.x() = .01 * id;
      frame.cloud = cloud;
      frames.Append(frame);
      for (const auto& group : builder.AddKeyframe(
               id, frame.timestamp, frame.odom_from_body))
        store.AppendRevision(group);
    }
    if (seal) {
      for (const auto& group : builder.Finish()) store.AppendRevision(group);
    }
    auto poses = store.MaterializePoseSnapshot(0, 3);
    poses.version = 1;
    poses.optimized_frame_count = 3;
    poses.parent_frame = PoseFrame::kMap;
    for (auto& pose : poses.poses) pose.map_from_body.translation.y() += 2;
    store.WritePoseSnapshot(poses);
  }

  void TearDown() override { std::filesystem::remove_all(root_); }

  std::pair<int, std::string> Run(std::vector<std::string> extra = {}) {
    std::vector<std::string> arguments{"--dataset", root_.string()};
    arguments.insert(arguments.end(), extra.begin(), extra.end());
    testing::internal::CaptureStdout();
    const int code = RunGeometryBenchmark(arguments);
    return {code, testing::internal::GetCapturedStdout()};
  }

  std::filesystem::path root_;
};

TEST_F(GeometryBenchmarkTest, HandlesDifferentSizesVersionsAndPlanarVoxels) {
  const auto stamp = std::filesystem::last_write_time(root_ / "dataset.yml");
  for (const auto* version : {"0", "1"}) {
    const auto result = Run({"--pose-version", version, "--voxel-size", "1",
                              "--rounds", "2"});
    ASSERT_EQ(result.first, 0) << result.second;
    cv::FileStorage json(result.second,
                         cv::FileStorage::READ | cv::FileStorage::MEMORY |
                             cv::FileStorage::FORMAT_JSON);
    EXPECT_EQ(static_cast<int>(json["frame_count"]), 3);
    EXPECT_EQ(static_cast<int>(json["submap_count"]), 2);
    EXPECT_EQ(json["samples"].size(), 2u);
    EXPECT_GT(static_cast<int>(json["normal_voxels"]), 0);
    EXPECT_EQ(static_cast<std::string>(json["reference_fingerprint"]),
              static_cast<std::string>(json["mapper_fingerprint"]));
    EXPECT_EQ(static_cast<int>(json["geometry_matches"]), 1);
  }
  EXPECT_EQ(std::filesystem::last_write_time(root_ / "dataset.yml"), stamp);
}

TEST_F(GeometryBenchmarkTest, RejectsInvalidArgumentsAndResourceLimits) {
  for (const auto& arguments :
       std::vector<std::vector<std::string>>{
           {"--pose-version", "-1"}, {"--rounds", "0"},
           {"--rounds", "1000000"}, {"--voxel-size", "nan"},
           {"--max-preload-mb", "0"}, {"--unknown", "1"},
           {"--dataset"}}) {
    EXPECT_NE(Run(arguments).first, 0);
  }
  EXPECT_NE(Run({"--max-points", "1"}).first, 0);
  EXPECT_NE(Run({"--pose-version", "2"}).first, 0);
  EXPECT_FALSE(std::filesystem::exists(root_ / "summary.json"));
}

TEST_F(GeometryBenchmarkTest, RejectsUnsealedDataAndExcessivePreload) {
  std::filesystem::remove_all(root_);
  WriteDataset(false, 9);
  EXPECT_NE(Run().first, 0);
  std::filesystem::remove_all(root_);
  WriteDataset(true, 20000);
  EXPECT_NE(Run({"--max-preload-mb", "1"}).first, 0);
}

}  // namespace
}  // namespace livo_atlas
