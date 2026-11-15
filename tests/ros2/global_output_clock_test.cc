#include <opencv2/core.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include "core/fixtures.h"
#include "geometry_map_fixture.h"
#include "livo_atlas/ros2/geometry_map_publisher.h"
#include "livo_atlas/ros2/keyframe_trajectory_publisher.h"
#include "livo_atlas/ros2/livo_node.h"
#include "ros2/global_output_test_support.h"
#include "trajectory_fixture.h"

namespace livo_atlas::ros2::test_support {
namespace {

class GlobalClockTest : public GlobalOutputTest {
 protected:
  void SetUp() override {
    GlobalOutputTest::SetUp();
    node_->set_parameter(rclcpp::Parameter("use_sim_time", true));
    clock_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    ASSERT_TRUE(Wait([&] { return clock_->get_subscription_count() > 0; }));
    SetTime(0);
  }

  void SetTime(int seconds) {
    rosgraph_msgs::msg::Clock clock;
    clock.clock.sec = seconds;
    clock.clock.nanosec = seconds == 0 ? 0 : 12345;
    clock_->publish(clock);
    ASSERT_TRUE(Wait([&] {
      return node_->now().nanoseconds() ==
             static_cast<std::int64_t>(clock.clock.sec) * 1000000000 +
                 clock.clock.nanosec;
    }));
  }

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_;
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
  EXPECT_TRUE(rclcpp::Time(global_[0].header.stamp).nanoseconds() <
              rclcpp::Time(global_[1].header.stamp).nanoseconds());
}

TEST_F(GlobalClockTest, HistoricalVersionsDoNotSynthesizeCurrentOutput) {
  Open();
  SetTime(10);
  publisher_->Publish(LidarResult(), nullptr, MapCorrection{});
  ASSERT_TRUE(
      Wait([&] { return global_.size() == 1 && transforms_.size() == 1; }));
  KeyframeTrajectoryPublisher history(*node_, "map");
  GeometryMapPublisher geometry(*node_, "map");
  using Message = ::livo_atlas::msg::KeyframeTrajectory;
  using GeometryMessage = ::livo_atlas::msg::GeometryMap;
  std::vector<Message> messages;
  std::vector<GeometryMessage> maps;
  auto subscription = node_->create_subscription<Message>(
      "/keyframe_trajectory", rclcpp::QoS(1).reliable().transient_local(),
      [&](Message::ConstSharedPtr msg) { messages.push_back(*msg); });
  auto map_subscription = node_->create_subscription<GeometryMessage>(
      "/geometry_map", rclcpp::QoS(1).reliable().transient_local(),
      [&](GeometryMessage::ConstSharedPtr msg) { maps.push_back(*msg); });
  ASSERT_TRUE(Wait([&] {
    return subscription->get_publisher_count() > 0 &&
           map_subscription->get_publisher_count() > 0;
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
    EXPECT_EQ(messages.back().header.stamp.sec, 9);
    EXPECT_EQ(messages.back().header.stamp.nanosec, 500000000);
    EXPECT_EQ(maps.back().revision, i + 1);
    EXPECT_EQ(maps.back().pose_version, i);
    EXPECT_EQ(maps.back().header.stamp.sec, 9);
    EXPECT_EQ(maps.back().header.stamp.nanosec, 500000000u);
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
  publisher_->PublishImuPropagation(result.estimate, 10);
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
    LivoNode mapper(*node_, parameters, system, [&] { ++polls; });
    ASSERT_TRUE(Wait([&] {
      return odom_sub_->get_publisher_count() && tf_sub_->get_publisher_count();
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
}  // namespace livo_atlas::ros2::test_support
