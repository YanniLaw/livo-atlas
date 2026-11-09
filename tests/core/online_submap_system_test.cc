#include <gtest/gtest.h>

#include <fstream>
#include <iterator>

#include "core/system_observations.h"
#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/io/online_submap_store.h"
namespace livo_atlas {
namespace {
namespace fs = std::filesystem;
SystemOptions Options() {
  auto options = test::Options(FrontendMode::kLidarOnly);
  options.keyframes.enabled = options.submaps.enabled = true;
  options.keyframes.time_threshold = 0;
  options.submap_mode = SubmapMode::kOnline;
  options.submaps.max_keyframes = 2;
  return options;
}
void Feed(LivoSystem& system, int frame) {
  ASSERT_TRUE(system.AddLidar(test::Scan(100 + frame * .25)));
  system.ProcessAvailable({});
}
void Populate(LivoSystem& system) {
  for (int i = 0; i < 4; ++i) Feed(system, i);
  ASSERT_GT(system.GetPoseSnapshot()->frame_count, 0u);
}
PoseUpdate Update(const PoseSnapshot& snapshot) {
  PoseUpdate update{snapshot.version, snapshot.frame_count, snapshot.poses};
  for (auto& pose : update.poses) pose.map_from_body.translation.x() += 10;
  return update;
}
TEST(OnlineSubmapSystemTest, AllModesPreserveFrontendAndRawAssets) {
  for (auto mode : {FrontendMode::kLidarOnly, FrontendMode::kLidarInertial,
                    FrontendMode::kLidarInertialVisual}) {
    auto options = test::Options(mode);
    options.keyframes.enabled = true;
    options.keyframes.time_threshold = 0;
    const auto baseline = test::RunSystemWithKeyframes(options);
    const auto original = options.exporter.directory;
    options.exporter.directory /= "online";
    options.submaps.enabled = true;
    options.submap_mode = SubmapMode::kOnline;
    const auto online = test::RunSystemWithKeyframes(options, true);
    EXPECT_EQ(baseline.states, online.states);
    EXPECT_EQ(baseline.clouds, online.clouds);
    EXPECT_EQ(baseline.image_hashes, online.image_hashes);
    EXPECT_TRUE(
        fs::exists(options.exporter.directory / "keyframes/online_submaps"));
    EXPECT_FALSE(fs::exists(options.exporter.directory / "keyframes/submaps"));
    for (const auto& entry :
         fs::recursive_directory_iterator(original / "keyframes")) {
      if (!entry.is_regular_file()) continue;
      std::ifstream before(entry.path(), std::ios::binary);
      std::ifstream after(
          options.exporter.directory / "keyframes" /
              fs::relative(entry.path(), original / "keyframes"),
          std::ios::binary);
      const std::string old_bytes{std::istreambuf_iterator<char>(before), {}};
      const std::string new_bytes{std::istreambuf_iterator<char>(after), {}};
      EXPECT_EQ(old_bytes, new_bytes);
    }
    fs::remove_all(original);
  }
}
TEST(OnlineSubmapSystemTest, OwnedObservationIsPublishedAfterCommit) {
  auto options = Options();
  OnlineSubmapUpdate saved;
  {
    LivoSystem system(options);
    system.SetOnlineSubmapCallback([&](const OnlineSubmapUpdate& update) {
      if (!update.keyframe) return;
      KeyframeStore frames(options.exporter.directory / "keyframes");
      OnlineSubmapStore store(
          options.exporter.directory / "keyframes/online_submaps", frames);
      EXPECT_EQ(store.indexed_frame_count(), update.poses->frame_count);
      EXPECT_EQ(frames.Read(update.keyframe->id).cloud->size(),
                update.keyframe->cloud->size());
      saved = update;
    });
    Populate(system);
    const auto count = saved.poses->frame_count;
    Feed(system, 4);
    EXPECT_GT(system.GetPoseSnapshot()->frame_count, count);
    system.SetOnlineSubmapCallback({});
    const auto cloud = saved.keyframe->cloud;
    Feed(system, 5);
    EXPECT_EQ(saved.keyframe->cloud, cloud);
    EXPECT_THROW(system.SetOnlineSubmapCallback([](const auto&) {}),
                 std::logic_error);
    system.Finish();
    system.Finish();
  }
  ASSERT_TRUE(saved.keyframe);
  EXPECT_FALSE(saved.keyframe->cloud->empty());
  EXPECT_TRUE(saved.keyframe->image.empty());
  fs::remove_all(options.exporter.directory);
}
TEST(OnlineSubmapSystemTest, PoseUpdateExtendsTailAndRejectsStaleInput) {
  auto options = Options();
  {
    LivoSystem system(options);
    Populate(system);
    const auto old = system.GetPoseSnapshot();
    auto update = Update(*old);
    system.ApplyPoseUpdate(update);
    EXPECT_EQ(system.GetPoseSnapshot()->version, 1u);
    EXPECT_EQ(old->version, 0u);
    EXPECT_THROW(system.ApplyPoseUpdate(update), std::invalid_argument);
    Feed(system, 4);
    EXPECT_EQ(system.GetPoseSnapshot()->optimized_frame_count,
              old->frame_count);
    EXPECT_GT(system.GetPoseSnapshot()->frame_count, old->frame_count);
    system.Finish();
    EXPECT_THROW(system.ApplyPoseUpdate(Update(*system.GetPoseSnapshot())),
                 std::logic_error);
  }
  fs::remove_all(options.exporter.directory);
}
TEST(OnlineSubmapSystemTest, RawSaveFailureBlocksOnlineMutations) {
  auto options = Options();
  {
    LivoSystem system(options);
    int notifications = 0;
    system.SetOnlineSubmapCallback([&](const auto&) { ++notifications; });
    Populate(system);
    const auto before = system.GetPoseSnapshot();
    const auto count = notifications;
    const auto pending = options.exporter.directory / "keyframes/frames" /
                         (".pending-" + std::to_string(before->frame_count));
    fs::create_directory(pending);
    std::ofstream(pending / "keep") << "original";
    std::string failure;
    try {
      Feed(system, 4);
    } catch (const std::runtime_error& error) {
      failure = error.what();
    }
    ASSERT_FALSE(failure.empty());
    try {
      system.ApplyPoseUpdate(Update(*before));
      ADD_FAILURE() << "Pose update must preserve the raw-save terminal error";
    } catch (const std::runtime_error& error) {
      EXPECT_EQ(error.what(), failure);
    }
    EXPECT_EQ(system.GetPoseSnapshot()->version, before->version);
    EXPECT_EQ(system.GetPoseSnapshot()->frame_count, before->frame_count);
    EXPECT_EQ(notifications, count);
    EXPECT_FALSE(fs::exists(options.exporter.directory /
                            "keyframes/online_submaps/poses/1.yml"));
    EXPECT_NO_THROW(system.SetOnlineSubmapCallback({}));
    try {
      system.SetOnlineSubmapCallback([](const auto&) {});
      ADD_FAILURE() << "Nonempty registration must preserve terminal state";
    } catch (const std::exception& error) {
      EXPECT_EQ(error.what(), failure);
    }
    EXPECT_THROW(system.Finish(), std::runtime_error);
  }
  fs::remove_all(options.exporter.directory);
}
TEST(OnlineSubmapSystemTest, FailedPoseCommitKeepsPublishedVersion) {
  auto options = Options();
  {
    LivoSystem system(options);
    Populate(system);
    system.ApplyPoseUpdate(Update(*system.GetPoseSnapshot()));
    const auto last = system.GetPoseSnapshot();
    fs::create_directory(options.exporter.directory /
                         "keyframes/online_submaps/poses/.pending-2.yml");
    EXPECT_THROW(system.ApplyPoseUpdate(Update(*last)), std::runtime_error);
    EXPECT_EQ(system.GetPoseSnapshot()->version, 1u);
    EXPECT_THROW(system.ProcessAvailable({}), std::runtime_error);
    EXPECT_THROW(system.Finish(), std::runtime_error);
  }
  fs::remove_all(options.exporter.directory);
}
TEST(OnlineSubmapSystemTest, CallbackExceptionRemainsVisible) {
  auto options = Options();
  {
    LivoSystem system(options);
    system.SetOnlineSubmapCallback(
        [](const auto&) { throw std::runtime_error("consumer failed"); });
    EXPECT_THROW(Populate(system), std::runtime_error);
    EXPECT_THROW(system.ProcessAvailable({}), std::runtime_error);
    EXPECT_THROW(system.Finish(), std::runtime_error);
  }
  fs::remove_all(options.exporter.directory);
}
TEST(OnlineSubmapSystemTest, FailedRevisionPublishesNoPartialEvent) {
  auto options = Options();
  {
    LivoSystem system(options);
    int events = 0;
    system.SetOnlineSubmapCallback([&](const auto&) { ++events; });
    Populate(system);
    const int committed_events = events;
    const auto last = system.GetPoseSnapshot();
    const auto records =
        options.exporter.directory / "keyframes/online_submaps/records";
    fs::rename(records, records.parent_path() / "saved-records");
    std::ofstream(records) << "blocked";
    EXPECT_THROW(Feed(system, 4), std::runtime_error);
    EXPECT_EQ(events, committed_events);
    EXPECT_EQ(system.GetPoseSnapshot()->frame_count, last->frame_count);
    EXPECT_THROW(system.Finish(), std::runtime_error);
  }
  fs::remove_all(options.exporter.directory);
}
TEST(OnlineSubmapSystemTest, OptimizationDoesNotFeedBackIntoFrontend) {
  std::vector<std::vector<double>> reference;
  for (bool optimize : {false, true}) {
    auto options = Options();
    std::vector<std::vector<double>> states;
    {
      LivoSystem system(options);
      for (int frame = 0; frame < 7; ++frame) {
        system.AddLidar(test::Scan(100 + frame * .25));
        system.ProcessAvailable([&](const FrontendResult& result, const auto&,
                                    double) {
          const auto& state = result.estimate;
          std::vector<double> values(state.cov.data(),
                                     state.cov.data() + state.cov.size());
          values.insert(values.end(), state.rot_end.data(),
                        state.rot_end.data() + 9);
          for (const auto* v :
               {&state.pos_end, &state.vel_end, &state.bias_g, &state.bias_a})
            values.insert(values.end(), v->data(), v->data() + 3);
          states.push_back(std::move(values));
        });
        if (optimize && frame == 3) {
          auto update = Update(*system.GetPoseSnapshot());
          update.poses.back().map_from_body.translation.y() += 2;
          system.ApplyPoseUpdate(update);
        }
      }
      system.Finish();
    }
    if (optimize)
      EXPECT_EQ(states, reference);
    else
      reference = states;
    fs::remove_all(options.exporter.directory);
  }
}
TEST(OnlineSubmapSystemTest, OwnedVisualImageSurvivesSourceMutation) {
  auto options = test::Options(FrontendMode::kLidarInertialVisual);
  options.keyframes.enabled = options.submaps.enabled = true;
  options.keyframes.time_threshold = 0;
  options.submap_mode = SubmapMode::kOnline;
  std::shared_ptr<const Keyframe> held;
  std::uint64_t hash = 0;
  {
    LivoSystem system(options);
    system.SetOnlineSubmapCallback([&](const OnlineSubmapUpdate& update) {
      if (update.keyframe && !held) {
        held = update.keyframe;
        hash = test_data::ImageHash(held->image);
      }
    });
    for (int frame = 0; frame < 6; ++frame) {
      const auto timestamp = 100 + frame * .25;
      system.AddLidar(test::Scan(timestamp));
      for (double offset : {.0625, .125, .1875})
        system.AddImu(
            {timestamp + offset, Eigen::Vector3d::Zero(), {0, 0, -9.81}});
      auto image = test_data::MakeImage(512, 512, 3);
      system.AddImage({timestamp + .1, image});
      system.ProcessAvailable({});
      image.setTo(cv::Scalar::all(0));
    }
    system.Finish();
  }
  ASSERT_TRUE(held);
  EXPECT_FALSE(held->image.empty());
  EXPECT_EQ(test_data::ImageHash(held->image), hash);
  fs::remove_all(options.exporter.directory);
}
TEST(OnlineSubmapSystemTest, NotificationsInvalidateStaleConsumerResults) {
  auto options = Options();
  {
    LivoSystem system(options);
    OnlineSubmapUpdate latest;
    std::vector<OnlineSubmap> groups;
    system.SetOnlineSubmapCallback([&](const auto& update) {
      latest = update;
      for (const auto& revision : update.revisions) {
        if (revision.primary.id == groups.size())
          groups.push_back(revision);
        else
          groups.at(revision.primary.id) = revision;
      }
    });
    Populate(system);
    int next_frame = 4;
    while ((groups.size() < 2 ||
            groups.back().state == OnlineSubmapState::kSealed) &&
           next_frame < 10)
      Feed(system, next_frame++);
    ASSERT_GE(groups.size(), 2u);
    ASSERT_EQ(groups.back().state, OnlineSubmapState::kActive);
    const auto group_id = groups.back().primary.id;
    const auto old_revision = groups[group_id].membership_revision;
    const auto old_version = latest.poses->version;
    const auto skipped = system.GetPoseSnapshot();
    const auto existing = groups[0];
    std::uint64_t representation_revision = 0;
    const auto commit = [&](std::uint64_t id, std::uint64_t membership,
                            std::uint64_t pose_version) {
      if (id >= groups.size() || groups[id].membership_revision != membership ||
          latest.poses->version != pose_version)
        return false;
      ++representation_revision;
      return true;
    };
    auto update = Update(*system.GetPoseSnapshot());
    update.poses[1].map_from_body.translation.y() += 2;
    system.ApplyPoseUpdate(update);
    ASSERT_EQ(latest.poses->version, 1u);
    ASSERT_FALSE(latest.pose_changes.empty());
    EXPECT_EQ(latest.pose_changes[0].changed_keyframe_ids,
              (std::vector<std::uint64_t>{1}));
    EXPECT_FALSE(commit(group_id, old_revision, old_version));
    Feed(system, next_frame++);
    EXPECT_FALSE(commit(group_id, old_revision, 1));
    EXPECT_FALSE(commit(100, old_revision, 1));
    EXPECT_EQ(representation_revision, 0u);
    EXPECT_TRUE(commit(group_id, groups[group_id].membership_revision, 1));
    EXPECT_EQ(representation_revision, 1u);
    system.ApplyPoseUpdate(Update(*system.GetPoseSnapshot()));
    ASSERT_EQ(latest.poses->version, 2u);
    EXPECT_FALSE(commit(group_id, groups[group_id].membership_revision, 1));
    EXPECT_EQ(representation_revision, 1u);
    KeyframeStore frames(options.exporter.directory / "keyframes");
    std::vector<RigidTransform> raw;
    for (auto id : frames.ids()) raw.push_back(frames.ReadRawBodyPose(id));
    const auto old_view = MaterializePoseSnapshot(*skipped, raw, raw.size());
    EXPECT_EQ(CompareSubmapPoses(existing, old_view, *latest.poses)
                  .changed_keyframe_ids,
              (std::vector<std::uint64_t>{1}));
    system.Finish();
  }
  fs::remove_all(options.exporter.directory);
}
TEST(OnlineSubmapSystemTest, ValidatesConfigurationBeforeIo) {
  auto options = Options();
  options.online_submaps.support_keyframes = 3;
  EXPECT_THROW(LivoSystem{options}, std::invalid_argument);
  EXPECT_FALSE(fs::exists(options.exporter.directory));
  options.submap_mode = SubmapMode::kKeyframeGroups;
  EXPECT_NO_THROW(LivoSystem{options});
  fs::remove_all(options.exporter.directory);
}
}  // namespace
}  // namespace livo_atlas
