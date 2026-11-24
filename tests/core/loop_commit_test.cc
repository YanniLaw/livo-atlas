#include "pose_graph/loop_commit.h"

#include <gtest/gtest.h>

#include <filesystem>

#include "core/loop_backend_fixtures.h"

namespace livo_atlas {
namespace {
class LoopCommitTest : public testing::Test {
 protected:
  void SetUp() override {
    options_ = loop_test::OnlineOptions();
    system_ = std::make_unique<LivoSystem>(options_);
    cloud_ = loop_test::DenseCloud();
    for (std::uint64_t id = 0; id < 4; ++id) {
      auto frame = loop_test::DenseFrame(id, cloud_);
      if (id == 1) frame.odom_from_body.rotation(0, 0) = 1.000045;
      LoopBackendTestPeer::Append(*system_, std::move(frame));
    }
  }

  void TearDown() override {
    std::filesystem::remove_all(options_.exporter.directory);
  }

  SystemOptions options_;
  PointCloud::ConstPtr cloud_;
  std::unique_ptr<LivoSystem> system_;
};

TEST_F(LoopCommitTest, TailGrowthPreservesTaskPrefix) {
  const auto task = loop_test::CaptureTask(*system_, 2);
  const auto result = RunLoopTask(task, loop_test::NearbyOptions());
  ASSERT_TRUE(result.proposal);
  LoopBackendTestPeer::Append(*system_, loop_test::DenseFrame(4, cloud_));
  const auto current = loop_test::CaptureTask(*system_, 2);
  const auto check =
      CheckLoopCommit(task, result, current.input, current.timestamps);
  ASSERT_EQ(check.status, LoopCommitStatus::kEligible);
  ASSERT_TRUE(check.poses);
  EXPECT_EQ(check.poses->frame_count, 5u);
  EXPECT_EQ(check.poses->optimized_frame_count, 4u);
  const auto expected = MakePoseSnapshot(
      result.proposal->update, current.input.poses, current.input.raw_poses);
  for (std::size_t i = 0; i < 5; ++i) {
    EXPECT_TRUE(check.poses->poses[i].map_from_body.rotation.isApprox(
        expected.poses[i].map_from_body.rotation, 1e-12));
    EXPECT_TRUE(check.poses->poses[i].map_from_body.translation.isApprox(
        expected.poses[i].map_from_body.translation, 1e-12));
  }
  EXPECT_DOUBLE_EQ(task.input.raw_poses[1].rotation(0, 0), 1.000045);
  EXPECT_EQ(system_->GetPoseSnapshot()->version, 0u);
}

TEST_F(LoopCommitTest, RejectsStaleVersionRawAndMembership) {
  const auto task = loop_test::CaptureTask(*system_, 2);
  const auto result = RunLoopTask(task, loop_test::NearbyOptions());
  ASSERT_TRUE(result.proposal);
  for (int change = 0; change < 5; ++change) {
    auto current = task.input;
    auto times = task.timestamps;
    if (change == 0) ++current.poses.version;
    if (change == 1) current.raw_poses[1].rotation(0, 0) += 0.000001;
    if (change == 2) ++current.submaps[0].membership.membership_revision;
    if (change == 3) times[0] += .01;
    if (change == 4) current.accepted_loops.push_back(result.proposal->loop);
    const auto check = CheckLoopCommit(task, result, current, times);
    EXPECT_EQ(check.status, LoopCommitStatus::kStale) << change;
    EXPECT_FALSE(check.poses);
  }
  auto broken = result;
  broken.proposal.reset();
  EXPECT_THROW(CheckLoopCommit(task, broken, task.input, task.timestamps),
               std::invalid_argument);
  broken = result;
  --broken.proposal->update.optimized_frame_count;
  EXPECT_THROW(CheckLoopCommit(task, broken, task.input, task.timestamps),
               std::invalid_argument);
}
}  // namespace
}  // namespace livo_atlas
