#include <gtest/gtest.h>

#include <filesystem>

#include "core/system_observations.h"
#include "livo_atlas/core/livo_system.h"

namespace livo_atlas {
namespace {

SystemOptions GeometryOptionsForSystem() {
  auto options = test::Options(FrontendMode::kLidarOnly);
  options.keyframes.enabled = options.submaps.enabled = true;
  options.keyframes.time_threshold = 0;
  options.submap_mode = SubmapMode::kOnline;
  options.submaps.max_keyframes = 2;
  options.geometry.enabled = true;
  return options;
}

void FeedGeometry(LivoSystem& system, int frame) {
  ASSERT_TRUE(system.AddLidar(test::Scan(100 + frame * .25)));
  system.ProcessAvailable({});
}

TEST(GeometrySystemTest, RejectsIncompatibleOptionsBeforeOutput) {
  auto options = GeometryOptionsForSystem();
  options.submap_mode = SubmapMode::kKeyframeGroups;
  EXPECT_THROW(LivoSystem{options}, std::invalid_argument);
  EXPECT_FALSE(std::filesystem::exists(options.exporter.directory));
  options.submap_mode = SubmapMode::kOnline;
  options.submaps.max_keyframes = 65;
  EXPECT_THROW(LivoSystem{options}, std::invalid_argument);
  EXPECT_FALSE(std::filesystem::exists(options.exporter.directory));
}

TEST(GeometrySystemTest, OnlineGeometryFollowsCommittedObservations) {
  auto options = GeometryOptionsForSystem();
  {
    LivoSystem system(options);
    int notifications = 0;
    system.SetOnlineSubmapCallback([&](const auto& update) {
      if (update.keyframe) ++notifications;
    });
    for (int i = 0; i < 4; ++i) FeedGeometry(system, i);
    ASSERT_GT(notifications, 0);
    const auto first = system.GetGeometrySubmap(0);
    ASSERT_TRUE(first.data);
    EXPECT_GT(first.data->input_point_count, 0u);
    EXPECT_EQ(first.pose_version, system.GetPoseSnapshot()->version);
    EXPECT_THROW(system.GetGeometrySubmap(1000), std::out_of_range);
    system.Finish();
    EXPECT_TRUE(system.GetGeometrySubmap(0).data);
  }
  std::filesystem::remove_all(options.exporter.directory);
}

TEST(GeometrySystemTest, GeometryFailureStopsCommittedWriter) {
  auto options = GeometryOptionsForSystem();
  options.geometry.max_voxels_per_submap = 1;
  {
    LivoSystem system(options);
    int notifications = 0;
    system.SetOnlineSubmapCallback([&](const auto&) { ++notifications; });
    bool failed = false;
    for (int i = 0; i < 4 && !failed; ++i) {
      try {
        FeedGeometry(system, i);
      } catch (const std::exception&) {
        failed = true;
      }
    }
    EXPECT_TRUE(failed);
    EXPECT_EQ(notifications, 0);
    EXPECT_THROW(system.ProcessAvailable({}), std::exception);
    EXPECT_THROW(system.GetGeometrySubmap(0), std::exception);
    EXPECT_THROW(system.Finish(), std::exception);
  }
  std::filesystem::remove_all(options.exporter.directory);
}

TEST(GeometrySystemTest, GeometryDoesNotChangeFrontendResults) {
  for (const auto mode :
       {FrontendMode::kLidarOnly, FrontendMode::kLidarInertial,
        FrontendMode::kLidarInertialVisual}) {
    auto options = test::Options(mode);
    options.keyframes.enabled = options.submaps.enabled = true;
    options.keyframes.time_threshold = 0;
    options.submap_mode = SubmapMode::kOnline;
    const auto baseline = test::RunSystemWithKeyframes(options);
    const auto root = options.exporter.directory;
    options.exporter.directory /= "geometry";
    options.geometry.enabled = true;
    const auto geometry = test::RunSystemWithKeyframes(options);
    EXPECT_EQ(geometry.states, baseline.states);
    EXPECT_EQ(geometry.clouds, baseline.clouds);
    EXPECT_EQ(geometry.image_hashes, baseline.image_hashes);
    std::filesystem::remove_all(root);
  }
}

TEST(GeometrySystemTest, PoseCorrectionsPreserveLocalDataWhenRigid) {
  auto options = GeometryOptionsForSystem();
  {
    LivoSystem system(options);
    for (int i = 0; i < 5; ++i) FeedGeometry(system, i);
    const auto original = system.GetGeometrySubmap(0);
    const auto poses = system.GetPoseSnapshot();
    PoseUpdate update{poses->version, poses->frame_count, poses->poses};
    for (auto& pose : update.poses) pose.map_from_body.translation.x() += 10;
    system.ApplyPoseUpdate(update);
    const auto corrected = system.GetGeometrySubmap(0);
    EXPECT_EQ(corrected.data.get(), original.data.get());
    EXPECT_EQ(corrected.pose_version, 1u);
    EXPECT_EQ(corrected.representation_revision,
              original.representation_revision);
    EXPECT_NEAR(corrected.parent_from_submap.translation.x() -
                    original.parent_from_submap.translation.x(),
                10, 1e-9);
    FeedGeometry(system, 5);
    system.Finish();
  }
  std::filesystem::remove_all(options.exporter.directory);
}

}  // namespace
}  // namespace livo_atlas
