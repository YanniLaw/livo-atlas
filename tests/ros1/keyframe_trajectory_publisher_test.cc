#include "livo_atlas/ros1/keyframe_trajectory_publisher.h"

#include <gtest/gtest.h>
#include <livo_atlas/KeyframeTrajectory.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <ros/serialization.h>

#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <vector>

#include "trajectory_fixture.h"

#include "livo_atlas/common/logging.h"

namespace livo_atlas {
namespace {

bool WaitForTrajectory(const std::function<bool()>& ready) {
  const auto deadline = ros::WallTime::now() + ros::WallDuration(5);
  while (ros::ok() && ros::WallTime::now() < deadline) {
    ros::spinOnce();
    if (ready()) return true;
    ros::WallDuration(.001).sleep();
  }
  return false;
}

TEST(KeyframeTrajectoryPublisherTest, LatchedVersionedTrajectoryAndPath) {
  ros::NodeHandle node;
  ros1::KeyframeTrajectoryPublisher publisher(node, "atlas/map");
  auto source = test::TrajectoryFixture();
  publisher.Publish(source);
  std::vector<KeyframeTrajectory> messages;
  std::vector<nav_msgs::Path> paths;
  auto trajectory = node.subscribe<KeyframeTrajectory>(
      "/keyframe_trajectory", 10, [&](const KeyframeTrajectory::ConstPtr& msg) {
        messages.push_back(*msg);
      });
  auto path = node.subscribe<nav_msgs::Path>(
      "/keyframe_path", 10,
      [&](const nav_msgs::Path::ConstPtr& msg) { paths.push_back(*msg); });
  ASSERT_TRUE(WaitForTrajectory(
      [&] { return messages.size() == 1 && paths.size() == 1; }));
  EXPECT_EQ(messages[0].header.frame_id, "camera_init");
  source.pose_version = 2;
  source.optimized_frame_count = 1;
  source.parent_frame = PoseFrame::kMap;
  source.revision = 2;
  publisher.Publish(source);
  ASSERT_TRUE(WaitForTrajectory(
      [&] { return messages.size() == 2 && paths.size() == 2; }));
  const auto& message = messages.back();
  EXPECT_EQ(message.producer_id, std::string(32, 'a'));
  EXPECT_EQ(message.revision, 2);
  EXPECT_EQ(message.pose_version, 2);
  EXPECT_EQ(message.frame_count, 2);
  EXPECT_EQ(message.optimized_frame_count, 1);
  EXPECT_EQ(message.status, KeyframeTrajectory::READY);
  EXPECT_EQ(message.header.frame_id, "atlas/map");
  EXPECT_EQ(message.header.stamp.toNSec(), 9500000000LL);
  ASSERT_EQ(message.keyframes.size(), 2);
  ASSERT_EQ(paths.back().poses.size(), 2);
  EXPECT_EQ(message.keyframes[1].id, 1);
  EXPECT_DOUBLE_EQ(message.keyframes[0].timestamp, 10.25);
  EXPECT_DOUBLE_EQ(message.keyframes[1].timestamp, 9.5);
  EXPECT_DOUBLE_EQ(message.keyframes[1].pose.position.x, -4);
  EXPECT_DOUBLE_EQ(message.keyframes[1].pose.position.y, 6);
  EXPECT_DOUBLE_EQ(message.keyframes[1].pose.position.z, 9);
  EXPECT_NEAR(message.keyframes[1].pose.orientation.z, std::sqrt(.5), 1e-12);
  EXPECT_NEAR(message.keyframes[1].pose.orientation.w, std::sqrt(.5), 1e-12);
  EXPECT_EQ(paths.back().header.frame_id, "atlas/map");
  EXPECT_EQ(paths.back().poses[0].header.stamp.toNSec(), 10250000000LL);
  EXPECT_EQ(paths.back().poses[1].header.stamp.toNSec(), 9500000000LL);
  EXPECT_DOUBLE_EQ(paths.back().poses[1].pose.position.x, -4);
  source.status = TrajectoryStatus::kResourceLimited;
  source.frame_count = 20001;
  source.keyframes.clear();
  source.revision = 3;
  publisher.Publish(source);
  ASSERT_TRUE(WaitForTrajectory(
      [&] { return messages.size() == 3 && paths.size() == 3; }));
  EXPECT_EQ(messages.back().status, KeyframeTrajectory::RESOURCE_LIMITED);
  EXPECT_EQ(messages.back().frame_count, 20001);
  EXPECT_TRUE(messages.back().keyframes.empty());
  EXPECT_TRUE(paths.back().poses.empty());
  EXPECT_TRUE(messages.back().header.stamp.isZero());
  source.status = TrajectoryStatus::kError;
  source.error.assign(2048, 'e');
  source.revision = 4;
  publisher.Publish(source);
  ASSERT_TRUE(WaitForTrajectory([&] { return messages.size() == 4; }));
  EXPECT_EQ(messages.back().status, KeyframeTrajectory::ERROR);
  EXPECT_EQ(messages.back().error.size(), 1024);
}

TEST(KeyframeTrajectoryPublisherTest, InvalidTimesClearThePublishedState) {
  ros::NodeHandle node;
  ros1::KeyframeTrajectoryPublisher publisher(node, "map");
  std::vector<KeyframeTrajectory> messages;
  std::vector<nav_msgs::Path> paths;
  auto trajectory = node.subscribe<KeyframeTrajectory>(
      "/keyframe_trajectory", 10, [&](const KeyframeTrajectory::ConstPtr& msg) {
        messages.push_back(*msg);
      });
  auto path = node.subscribe<nav_msgs::Path>(
      "/keyframe_path", 10,
      [&](const nav_msgs::Path::ConstPtr& msg) { paths.push_back(*msg); });
  ASSERT_TRUE(WaitForTrajectory([&] {
    return trajectory.getNumPublishers() && path.getNumPublishers();
  }));
  std::size_t count = 0;
  for (double time :
       {-1.0, 2147483648.0, std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()}) {
    auto source = test::TrajectoryFixture();
    source.revision = ++count;
    source.keyframes[0].timestamp = time;
    EXPECT_THROW(publisher.Publish(source), std::invalid_argument);
    ASSERT_TRUE(WaitForTrajectory(
        [&] { return messages.size() == count && paths.size() == count; }));
    EXPECT_EQ(messages.back().status, KeyframeTrajectory::ERROR);
    EXPECT_TRUE(messages.back().keyframes.empty());
    EXPECT_TRUE(paths.back().poses.empty());
    EXPECT_TRUE(messages.back().header.stamp.isZero());
  }
}

TEST(KeyframeTrajectoryPublisherTest,
     SerializesTheDefaultTwentyThousandFrameBudget) {
  ros::NodeHandle node;
  ros1::KeyframeTrajectoryPublisher publisher(node, "map");
  auto source = test::TrajectoryFixture();
  source.frame_count = 20000;
  source.keyframes.resize(20000);
  for (std::size_t i = 0; i < source.keyframes.size(); ++i)
    source.keyframes[i] = {i, 100 + .01 * i, {}};
  publisher.Publish(source);
  KeyframeTrajectory received;
  bool done = false;
  auto subscription = node.subscribe<KeyframeTrajectory>(
      "/keyframe_trajectory", 1, [&](const KeyframeTrajectory::ConstPtr& msg) {
        received = *msg;
        done = true;
      });
  ASSERT_TRUE(WaitForTrajectory([&] { return done; }));
  ASSERT_EQ(received.keyframes.size(), 20000);
  const auto serialized = ros::serialization::serializeMessage(received);
  EXPECT_GT(serialized.num_bytes, 1440000u);
  EXPECT_LT(serialized.num_bytes, 2u * 1024 * 1024);
  LOG(INFO) << "trajectory serialized bytes=" << serialized.num_bytes;
}

}  // namespace
}  // namespace livo_atlas
