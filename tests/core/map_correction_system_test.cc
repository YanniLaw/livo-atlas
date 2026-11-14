#include "livo_atlas/core/livo_system.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "core/fixtures.h"
#include "core/online_submap_manager.h"
#include "livo_atlas/mapping/map_correction.h"
#include "mapping/keyframe_fixtures.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;

Eigen::Matrix3d RawRotation() {
  Eigen::Matrix3d rotation;
  rotation << 0, 0, 1, 0, 1, 0, -1, 0, 0;
  return rotation;
}

PoseUpdate ProjectedUpdate() {
  PoseUpdate update;
  update.optimized_frame_count = 2;
  update.tail_policy = PoseTailPolicy::kProjectedLastOptimizedAnchor;
  RigidTransform second;
  second.rotation << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  second.translation << 1, 2, 3;
  update.poses = {{0, {}}, {1, second}};
  return update;
}

std::string Bytes(const fs::path& path) {
  std::ifstream stream(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(stream), {}};
}

class MapCorrectionManagerTest : public testing::Test {
 protected:
  void SetUp() override {
    options_ = test::Options(FrontendMode::kLidarOnly);
    root_ = options_.exporter.directory / "keyframes";
    frames_ =
        std::make_unique<KeyframeStore>(root_, options_.frontend.calibration);
    manager_ = std::make_unique<OnlineSubmapManager>(
        root_ / "online_submaps", *frames_, 2, OnlineSubmapOptions{},
        GeometryOptions{});
  }

  void TearDown() override {
    manager_.reset();
    frames_.reset();
    fs::remove_all(options_.exporter.directory);
  }

  void Append(std::uint64_t id, bool approximate = true) {
    auto frame =
        CaptureKeyframe(id, test_data::KeyframeResult(UpdateKind::kLidarOnly));
    frame.timestamp += id;
    frame.cloud_timestamp += id;
    frame.odom_from_body.rotation = RawRotation();
    if (approximate) frame.odom_from_body.rotation(1, 1) = 1.000045;
    frame.odom_from_body.translation = {1000.0 * id, 0, 0};
    auto owned = std::make_shared<const Keyframe>(std::move(frame));
    frames_->Append(*owned);
    manager_->AppendCommittedKeyframe(std::move(owned));
  }

  SystemOptions options_;
  fs::path root_;
  std::unique_ptr<KeyframeStore> frames_;
  std::unique_ptr<OnlineSubmapManager> manager_;
};

TEST_F(MapCorrectionManagerTest, EmptyAndGrowingVersionZero) {
  auto correction = manager_->GetMapCorrection();
  EXPECT_EQ(correction.pose_version, 0u);
  EXPECT_EQ(correction.frame_count, 0u);
  EXPECT_EQ(correction.optimized_frame_count, 0u);
  EXPECT_TRUE(correction.map_from_odom.rotation.isIdentity());
  EXPECT_TRUE(correction.map_from_odom.translation.isZero());
  for (std::uint64_t id = 0; id < 3; ++id) {
    Append(id);
    correction = manager_->GetMapCorrection();
    EXPECT_EQ(correction.pose_version, 0u);
    EXPECT_EQ(correction.frame_count, id + 1);
    EXPECT_EQ(correction.optimized_frame_count, 0u);
    EXPECT_TRUE(correction.map_from_odom.rotation.isIdentity());
    EXPECT_TRUE(correction.map_from_odom.translation.isZero());
  }
  manager_->Finish();
}

TEST_F(MapCorrectionManagerTest, ProjectedAnchorMatchesMaterializedTail) {
  for (std::uint64_t id = 0; id < 3; ++id) Append(id);
  const auto original = manager_->GetPoseSnapshot();
  const auto metadata = root_ / "frames/1/frame.yml";
  const auto before = Bytes(metadata);
  manager_->ApplyPoseUpdate(ProjectedUpdate());
  const auto frozen = manager_->GetPoseSnapshot();
  // Reading the correction must not need the corresponding frame file.
  fs::rename(metadata, metadata.string() + ".saved");
  const auto correction = manager_->GetMapCorrection();
  fs::rename(metadata.string() + ".saved", metadata);
  EXPECT_EQ(correction.pose_version, 1u);
  EXPECT_EQ(correction.frame_count, 3u);
  EXPECT_EQ(correction.optimized_frame_count, 2u);
  Eigen::Matrix3d expected;
  expected << 0, -1, 0, 0, 0, -1, 1, 0, 0;
  EXPECT_TRUE(correction.map_from_odom.rotation.isApprox(expected, 1e-12));
  EXPECT_TRUE(correction.map_from_odom.translation.isApprox(
      Eigen::Vector3d(1, 2, -997), 1e-12));
  RigidTransform tail{RawRotation(), {2000, 0, 0}};
  const auto global = ApplyMapCorrection(correction, tail);
  EXPECT_TRUE(
      global.rotation.isApprox(frozen->poses[2].map_from_body.rotation, 1e-12));
  EXPECT_TRUE(global.translation.isApprox(Eigen::Vector3d(1, 2, 1003), 1e-12));
  EXPECT_TRUE(global.translation.isApprox(
      frozen->poses[2].map_from_body.translation, 1e-12));
  Append(3);
  const auto extended = manager_->GetMapCorrection();
  EXPECT_EQ(extended.pose_version, 1u);
  EXPECT_EQ(extended.frame_count, 4u);
  EXPECT_EQ(extended.optimized_frame_count, 2u);
  EXPECT_TRUE(extended.map_from_odom.rotation.isApprox(expected, 1e-12));
  EXPECT_TRUE(extended.map_from_odom.translation.isApprox(
      Eigen::Vector3d(1, 2, -997), 1e-12));
  EXPECT_EQ(original->version, 0u);
  EXPECT_DOUBLE_EQ(original->poses[1].map_from_body.rotation(1, 1), 1.000045);
  EXPECT_EQ(frozen->frame_count, 3u);
  EXPECT_EQ(Bytes(metadata), before);
  manager_->Finish();
}

TEST_F(MapCorrectionManagerTest, ReadsAfterPublishedNotificationFailure) {
  manager_->SetCallback([](const OnlineSubmapUpdate& update) {
    if (update.poses->version > 0)
      throw std::runtime_error("notification failed");
  });
  for (std::uint64_t id = 0; id < 3; ++id) Append(id);
  EXPECT_THROW(manager_->ApplyPoseUpdate(ProjectedUpdate()),
               std::runtime_error);
  const auto published = root_ / "online_submaps/poses/1.yml";
  ASSERT_TRUE(fs::is_regular_file(published));
  const auto bytes = Bytes(published);
  const auto correction = manager_->GetMapCorrection();
  EXPECT_EQ(correction.pose_version, 1u);
  EXPECT_EQ(correction.frame_count, 3u);
  EXPECT_EQ(correction.optimized_frame_count, 2u);
  EXPECT_THROW(manager_->Finish(), std::runtime_error);
  EXPECT_THROW(manager_->ApplyPoseUpdate(ProjectedUpdate()),
               std::runtime_error);
  EXPECT_EQ(Bytes(published), bytes);
}

TEST_F(MapCorrectionManagerTest, RejectsLegacyPositiveVersion) {
  for (std::uint64_t id = 0; id < 3; ++id) Append(id, false);
  auto update = ProjectedUpdate();
  update.tail_policy = PoseTailPolicy::kLegacyLastOptimizedAnchor;
  manager_->ApplyPoseUpdate(update);
  EXPECT_THROW(manager_->GetMapCorrection(), std::invalid_argument);
  EXPECT_EQ(manager_->GetPoseSnapshot()->version, 1u);
  EXPECT_NO_THROW(manager_->Finish());
}

TEST_F(MapCorrectionManagerTest, InvalidAnchorUpdateLeavesCorrectionUnchanged) {
  for (std::uint64_t id = 0; id < 3; ++id) Append(id);
  auto update = ProjectedUpdate();
  update.optimized_frame_count = 4;
  EXPECT_THROW(manager_->ApplyPoseUpdate(update), std::invalid_argument);
  EXPECT_EQ(manager_->GetMapCorrection().pose_version, 0u);
  EXPECT_EQ(manager_->GetMapCorrection().frame_count, 3u);
  manager_->ApplyPoseUpdate(ProjectedUpdate());
  EXPECT_EQ(manager_->GetMapCorrection().pose_version, 1u);
  manager_->Finish();
}

TEST(MapCorrectionSystemTest, RejectsNononlineSystem) {
  for (bool groups : {false, true}) {
    auto options = test::Options(FrontendMode::kLidarOnly);
    options.keyframes.enabled = options.submaps.enabled = groups;
    {
      LivoSystem system(options);
      EXPECT_THROW(system.GetMapCorrection(), std::logic_error);
      system.Finish();
    }
    fs::remove_all(options.exporter.directory);
  }
}

TEST(MapCorrectionSystemTest, OrdinaryResultCallbackReadsCommittedFrontier) {
  auto options = test::Options(FrontendMode::kLidarOnly);
  options.keyframes.enabled = options.submaps.enabled = true;
  options.keyframes.time_threshold = 0;
  options.submap_mode = SubmapMode::kOnline;
  options.submaps.max_keyframes = 2;
  {
    LivoSystem system(options);
    std::uint64_t committed = 0, version = 0;
    int callbacks = 0;
    system.SetOnlineSubmapCallback([&](const OnlineSubmapUpdate& update) {
      if (update.keyframe) ++committed;
    });
    const auto collect = [&](const auto&, const auto&, double) {
      const auto correction = system.GetMapCorrection();
      EXPECT_EQ(correction.frame_count, committed);
      EXPECT_EQ(correction.pose_version, version);
      if (version > 0)
        EXPECT_TRUE(correction.map_from_odom.translation.isApprox(
            Eigen::Vector3d(3, 4, 5), 1e-9));
      ++callbacks;
    };
    for (int i = 0; i < 6; ++i) {
      ASSERT_TRUE(system.AddLidar(test::Scan(100 + .25 * i)));
      system.ProcessAvailable(collect);
      if (i == 3) {
        const auto snapshot = system.GetPoseSnapshot();
        ASSERT_GT(snapshot->frame_count, 0u);
        PoseUpdate update{snapshot->version, snapshot->frame_count,
                          snapshot->poses,
                          PoseTailPolicy::kProjectedLastOptimizedAnchor};
        for (auto& pose : update.poses)
          pose.map_from_body.translation += Eigen::Vector3d(3, 4, 5);
        system.ApplyPoseUpdate(update);
        version = 1;
      }
    }
    EXPECT_GT(callbacks, 0);
    EXPECT_GT(committed, 0u);
    EXPECT_EQ(system.GetMapCorrection().pose_version, 1u);
    system.Finish();
    EXPECT_EQ(system.GetMapCorrection().pose_version, 1u);
  }
  fs::remove_all(options.exporter.directory);
}

}  // namespace
}  // namespace livo_atlas
