#include <livo_atlas/GeometryMap.h>
#include <livo_atlas/KeyframeTrajectory.h>
#include <opencv2/core.hpp>

#include "core/fixtures.h"
#include "geometry_map_fixture.h"
#include "livo_atlas/ros1/geometry_map_publisher.h"
#include "livo_atlas/ros1/keyframe_trajectory_publisher.h"
#include "livo_atlas/ros1/livo_node.h"
#include "ros1/global_output_test_support.h"
#include "trajectory_fixture.h"

namespace livo_atlas::ros1::test_support {
namespace {

// ros::Time is process-global; this executable never shares the old wall-time
// tests.
class GlobalClockTest : public GlobalOutputTest {
 protected:
  void SetUp() override {
    ros::Time::setNow(ros::Time(0));
    GlobalOutputTest::SetUp();
  }

  void SetTime(int seconds) {
    ros::Time::setNow(ros::Time(seconds, seconds == 0 ? 0 : 12345));
  }
};

TEST_F(GlobalClockTest, ZeroRepeatRollbackAndResume) {
  Open();
  const auto result = LidarResult();
  const std::vector<int> times{0, 10, 10, 9, 11};
  for (std::size_t i = 0; i < times.size(); ++i) {
    SetTime(times[i]);
    auto correction = Correction();
    correction.pose_version += i;
    correction.map_from_odom.translation.x() += static_cast<double>(i);
    publisher_->Publish(result, nullptr, correction);
    ASSERT_TRUE(Wait([&] {
      return odometry_.size() == i + 1 && transforms_.size() == i + 1;
    }));
    Drain();
    const std::size_t expected_count = i == 0 ? 0 : (i == 4 ? 2 : 1);
    ASSERT_EQ(global_.size(), expected_count);
    const auto& tf = transforms_.back();
    const bool new_output = i == 1 || i == 4;
    ASSERT_EQ(tf.transforms.size(), new_output ? 2 : 1);
    if (new_output) {
      EXPECT_EQ(global_.back().header.stamp, odometry_.back().header.stamp);
      EXPECT_EQ(tf.transforms[0].header.stamp, odometry_.back().header.stamp);
      EXPECT_EQ(tf.transforms[1].header.stamp, odometry_.back().header.stamp);
      EXPECT_DOUBLE_EQ(tf.transforms[0].transform.translation.x,
                       1 + static_cast<double>(i));
      EXPECT_DOUBLE_EQ(global_.back().pose.position.x,
                       -4 + static_cast<double>(i));
    }
  }
  EXPECT_TRUE(global_[0].header.stamp.toNSec() <
              global_[1].header.stamp.toNSec());
}

TEST_F(GlobalClockTest, HistoricalVersionsDoNotSynthesizeCurrentOutput) {
  Open();
  SetTime(10);
  publisher_->Publish(LidarResult(), nullptr, MapCorrection{});
  ASSERT_TRUE(
      Wait([&] { return global_.size() == 1 && transforms_.size() == 1; }));
  KeyframeTrajectoryPublisher history(node_, "map");
  GeometryMapPublisher geometry(node_, "map");
  std::vector<::livo_atlas::KeyframeTrajectory> messages;
  std::vector<::livo_atlas::GeometryMap> maps;
  auto subscription = node_.subscribe<::livo_atlas::KeyframeTrajectory>(
      "/keyframe_trajectory", 10,
      [&](const ::livo_atlas::KeyframeTrajectory::ConstPtr& msg) {
        messages.push_back(*msg);
      });
  auto map_subscription = node_.subscribe<::livo_atlas::GeometryMap>(
      "/geometry_map", 10, [&](const ::livo_atlas::GeometryMap::ConstPtr& msg) {
        maps.push_back(*msg);
      });
  ASSERT_TRUE(Wait([&] {
    return subscription.getNumPublishers() > 0 &&
           map_subscription.getNumPublishers() > 0;
  }));
  const int times[] = {0, 10, 10, 9, 11};
  for (std::size_t i = 0; i < 5; ++i) {
    SetTime(times[i]);
    auto source = ::livo_atlas::test::TrajectoryFixture();
    source.revision = i + 1;
    source.pose_version = i;
    source.optimized_frame_count = i ? 2 : 0;
    source.parent_frame = i ? PoseFrame::kMap : PoseFrame::kOdom;
    history.Publish(source);
    auto map = ::livo_atlas::test::GeometryMapFixture();
    map.revision = i + 1;
    map.pose_version = i;
    map.optimized_frame_count = i ? 3 : 0;
    map.parent_frame = i ? PoseFrame::kMap : PoseFrame::kOdom;
    geometry.Publish(map);
    ASSERT_TRUE(
        Wait([&] { return messages.size() == i + 1 && maps.size() == i + 1; }));
    EXPECT_EQ(messages.back().revision, i + 1);
    EXPECT_EQ(messages.back().pose_version, i);
    EXPECT_EQ(messages.back().header.stamp.toNSec(), 9500000000LL);
    EXPECT_EQ(maps.back().revision, i + 1);
    EXPECT_EQ(maps.back().pose_version, i);
    EXPECT_EQ(maps.back().header.stamp.toNSec(), 9500000000LL);
  }
  Drain();
  EXPECT_EQ(global_.size(), 1);
  EXPECT_EQ(transforms_.size(), 1);
  EXPECT_EQ(odometry_.size(), 1);
}

TEST_F(GlobalClockTest, VisualWaitingAndPropagationDoNotAdvanceGlobalOutput) {
  Open();
  SetTime(10);
  auto result = LidarResult();
  result.updated = false;
  publisher_->Publish(result, nullptr);
  result.updated = true;
  result.kind = UpdateKind::kVisual;
  result.tracking_image = cv::Mat(2, 3, CV_8UC3, cv::Scalar(1, 2, 3));
  publisher_->Publish(result, nullptr);
  publisher_->PublishImuPropagation(result.estimate, ros::Time(10));
  ASSERT_TRUE(Wait([&] { return images_ == 1 && propagations_ == 1; }));
  Drain();
  EXPECT_TRUE(global_.empty());
  EXPECT_TRUE(transforms_.empty());
  EXPECT_TRUE(odometry_.empty());
  publisher_->Publish(LidarResult(), nullptr, MapCorrection{});
  ASSERT_TRUE(Wait([&] {
    return global_.size() == 1 && odometry_.size() == 1 &&
           transforms_.size() == 1;
  }));
  EXPECT_DOUBLE_EQ(global_[0].pose.position.x, 4);
}

TEST_F(GlobalClockTest, NodePollContinuesDuringZeroAndPausedTime) {
  NodeOptions parameters;
  auto options = ::livo_atlas::test::Options(FrontendMode::kLidarOnly);
  options.keyframes.enabled = options.submaps.enabled = true;
  options.submap_mode = SubmapMode::kOnline;
  parameters.livo.system = PrepareSystemOptions(options);
  parameters.livo.global_output.enabled = true;
  {
    LivoSystem system(parameters.livo.system);
    int polls = 0;
    LivoNode mapper(node_, parameters, system, [&] { ++polls; });
    ASSERT_TRUE(Wait([&] {
      return odom_sub_.getNumPublishers() && tf_sub_.getNumPublishers();
    }));
    ASSERT_TRUE(system.AddLidar(::livo_atlas::test::Scan(100)));
    ASSERT_EQ(mapper.ProcessNext(), SyncStatus::kReady);
    ASSERT_TRUE(Wait([&] { return odometry_.size() == 1; }));
    const int before = polls;
    for (int i = 0; i < 3; ++i)
      EXPECT_EQ(mapper.ProcessNext(), SyncStatus::kWaiting);
    EXPECT_GE(polls, before + 3);
    Drain();
    EXPECT_TRUE(global_.empty());
    SetTime(10);
    ASSERT_TRUE(system.AddLidar(::livo_atlas::test::Scan(100.25)));
    ASSERT_EQ(mapper.ProcessNext(), SyncStatus::kReady);
    ASSERT_TRUE(
        Wait([&] { return global_.size() == 1 && odometry_.size() == 2; }));
    EXPECT_EQ(system.GetMapCorrection().pose_version, 0u);
    EXPECT_NEAR(global_[0].pose.position.x, odometry_[1].pose.pose.position.x,
                1e-9);
    system.Finish();
  }
  std::filesystem::remove_all(options.exporter.directory);
}
}  // namespace
}  // namespace livo_atlas::ros1::test_support

int main(int argc, char** argv) {
  ros::init(argc, argv, "global_output_clock_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
