#include "livo_atlas/backend/loop_task.h"

#include <gtest/gtest.h>

#if defined(__unix__)
#include <unistd.h>
#endif

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "backend/loop_dataset_fixtures.h"
#include "livo_atlas/io/keyframe_store.h"
#include "livo_atlas/io/online_submap_store.h"
#include "livo_atlas/mapping/geometry_mapper.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;

std::map<std::string, std::string> Bytes(const fs::path& root) {
  std::map<std::string, std::string> result;
  for (const auto& entry : fs::recursive_directory_iterator(root)) {
    if (!entry.is_regular_file()) continue;
    std::ifstream stream(entry.path(), std::ios::binary);
    result.emplace(fs::relative(entry.path(), root).string(),
                   std::string(std::istreambuf_iterator<char>(stream), {}));
  }
  return result;
}

class LoopTaskTest : public testing::Test {
 protected:
  void SetUp() override {
    root_ = fs::temp_directory_path() /
            ("c2b-loop-task-" +
             std::to_string(
                 std::chrono::steady_clock::now().time_since_epoch().count()));
    Calibration calibration;
    calibration.camera = {"Pinhole", 12, 8, .5, 4, 5, 6, 4, {}};
    frames_ = std::make_unique<KeyframeStore>(root_, calibration);
    OnlineSubmapOptions options;
    options.support_keyframes = 0;
    store_ = std::make_unique<OnlineSubmapStore>(root_ / "online_submaps",
                                                 *frames_, 1, options);
    builder_ = std::make_unique<OnlineSubmapBuilder>(1, options);
    cloud_ = loop_test::DenseCloud();
    for (std::uint64_t id = 0; id < 4; ++id) Append(id);
  }

  void TearDown() override { fs::remove_all(root_); }

  void Append(std::uint64_t id) {
    auto frame = loop_test::DenseFrame(id, cloud_);
    frames_->Append(frame);
    for (const auto& group :
         builder_->AddKeyframe(id, frame.timestamp, frame.odom_from_body))
      store_->AppendRevision(group);
  }

  LoopTask Task(std::uint64_t count = 4) {
    auto view = frames_->CaptureReadView();
    LoopAdmissionInput input;
    input.query_id = 2;
    input.poses = store_->MaterializePoseSnapshot(0, count);
    for (const auto& pose : input.poses.poses)
      input.raw_poses.push_back(pose.map_from_body);
    for (std::uint64_t id = 0; id + 1 < count; ++id) {
      auto group = store_->Read(id);
      input.submaps.push_back({group, 10.0 + id, 10.0 + id,
                               input.poses.poses[id].map_from_body, 0, count,
                               0});
    }
    std::vector<double> timestamps;
    for (std::uint64_t id = 0; id < count; ++id) timestamps.push_back(10 + id);
    return {5, std::move(input), std::move(timestamps), std::move(view)};
  }

  fs::path root_;
  PointCloud::ConstPtr cloud_;
  std::unique_ptr<KeyframeStore> frames_;
  std::unique_ptr<OnlineSubmapStore> store_;
  std::unique_ptr<OnlineSubmapBuilder> builder_;
};

TEST_F(LoopTaskTest, DispatchFrontierAndGeometryViews) {
  const auto task = Task();
  Append(4);
  // Later live index changes must not be scanned by the captured task.
  std::ofstream(root_ / "online_submaps/dataset.yml") << "invalid";
  fs::remove(root_ / "frames/4/cloud.pcd");
  const auto before = Bytes(root_);
  const auto result = RunLoopTask(task, loop_test::NearbyOptions());
  ASSERT_EQ(result.status, LoopAdmissionStatus::kProposed);
  ASSERT_TRUE(result.proposal);
  EXPECT_EQ(result.proposal->update.optimized_frame_count, 4u);
  EXPECT_EQ(result.proposal->update.poses.size(), 4u);
  EXPECT_EQ(result.proposal->loop.target.primary.id, 0u);
  EXPECT_EQ(result.proposal->loop.source.primary.id, 2u);
  EXPECT_EQ(task.input.poses.version, 0u);
  EXPECT_DOUBLE_EQ(task.input.raw_poses.back().translation.x(), .06);
  EXPECT_EQ(Bytes(root_), before);

  std::vector<OnlineSubmap> groups;
  for (const auto& info : task.input.submaps) groups.push_back(info.membership);
  GeometryOptions geometry;
  geometry.enabled = true;
  const auto direct = ProposeLoopUpdate(
      task.input,
      [&](std::uint64_t id, const PoseSnapshot& poses) {
        GeometryMapper mapper(geometry, [&](std::uint64_t frame) {
          return task.frames.ReadBodyCloud(frame,
                                           geometry.max_points_per_keyframe);
        });
        mapper.Update(groups, std::make_shared<const PoseSnapshot>(poses));
        return mapper.GetSubmap(id);
      },
      loop_test::NearbyOptions());
  ASSERT_TRUE(direct.proposal);
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_LT((direct.proposal->update.poses[i].map_from_body.translation -
               result.proposal->update.poses[i].map_from_body.translation)
                  .norm(),
              1e-9);
  }
  // A later task may keep the same version while extending its frontier.
  const auto later = RunLoopTask(Task(5), loop_test::NearbyOptions());
  ASSERT_TRUE(later.proposal);
  EXPECT_EQ(later.proposal->update.optimized_frame_count, 5u);
  EXPECT_EQ(Bytes(root_), before);
}

TEST_F(LoopTaskTest, KeepsGraphBudgetAheadOfCloudReads) {
  auto task = Task();
  fs::remove(root_ / "frames/0/cloud.pcd");
  auto options = loop_test::NearbyOptions();
  options.max_keyframes = 3;
  EXPECT_EQ(RunLoopTask(task, options).status,
            LoopAdmissionStatus::kResourceLimit);
}

TEST_F(LoopTaskTest, EnforcesConfiguredGeometryCloudLimit) {
  auto options = loop_test::NearbyOptions();
  options.geometry.max_points_per_keyframe = 1;
  EXPECT_THROW(RunLoopTask(Task(), options), std::runtime_error);
}

TEST_F(LoopTaskTest, RejectsDifferentTaskAndReadFrontiers) {
  auto task = Task();
  task.input.poses.frame_count = 3;
  task.input.poses.poses.resize(3);
  task.input.raw_poses.resize(3);
  task.timestamps.resize(3);
  for (auto& info : task.input.submaps) info.frame_count = 3;
  EXPECT_THROW(RunLoopTask(task, loop_test::NearbyOptions()),
               std::invalid_argument);
  task = Task();
  task.timestamps.pop_back();
  EXPECT_THROW(RunLoopTask(task, loop_test::NearbyOptions()),
               std::invalid_argument);
}

TEST_F(LoopTaskTest, RejectsChangedCommittedSourceMetadata) {
  auto task = Task();
  task.input.raw_poses[0].translation.x() = .01;
  task.input.poses.poses[0].map_from_body.translation.x() = .01;
  task.input.submaps[0].parent_from_submap.translation.x() = .01;
  EXPECT_THROW(RunLoopTask(task, loop_test::NearbyOptions()),
               std::invalid_argument);
  task = Task();
  task.timestamps[0] = 10.25;
  task.input.submaps[0].first_timestamp = 10.25;
  task.input.submaps[0].last_timestamp = 10.25;
  EXPECT_THROW(RunLoopTask(task, loop_test::NearbyOptions()),
               std::invalid_argument);
}

#if defined(__unix__)
TEST_F(LoopTaskTest, RejectsMalformedMembershipBeforeIndexingTimestamps) {
  auto task = Task();
  task.input.submaps[0].membership.primary.keyframe_ids = {
      std::uint64_t{1} << 52, 0};
  // A malformed ID must reach validation, rather than an unchecked read.
  ASSERT_EXIT(
      {
        try {
          RunLoopTask(task, loop_test::NearbyOptions());
        } catch (const std::invalid_argument&) {
          _exit(0);
        }
        _exit(1);
      },
      testing::ExitedWithCode(0), "");
}
#endif

}  // namespace
}  // namespace livo_atlas
