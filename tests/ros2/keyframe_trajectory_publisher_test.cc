#include "livo_atlas/ros2/keyframe_trajectory_publisher.h"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
#include <vector>

#include "trajectory_fixture.h"

namespace livo_atlas::ros2 {
namespace {
using Message = livo_atlas::msg::KeyframeTrajectory;
using Path = nav_msgs::msg::Path;

TEST(KeyframeTrajectoryPublisherTest, ClosedNodeContextSkipsConversion) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>(
      "trajectory_shutdown_test", rclcpp::NodeOptions().context(context));
  KeyframeTrajectoryPublisher publisher(*node, "map");
  context->shutdown("test shutdown");
  // The process-wide default context is still valid in this test.
  EXPECT_TRUE(rclcpp::ok());
  auto source = test::TrajectoryFixture();
  source.keyframes[0].timestamp = -1;
  EXPECT_NO_THROW(publisher.Publish(source));
}

bool WaitForTrajectory(rclcpp::Executor& executor,
                       const std::function<bool()>& ready) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (ready()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

TEST(KeyframeTrajectoryPublisherTest, LatchedVersionedTrajectoryAndPath) {
  auto node = std::make_shared<rclcpp::Node>("trajectory_publisher_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  KeyframeTrajectoryPublisher publisher(*node, "atlas/map");
  auto source = test::TrajectoryFixture();
  publisher.Publish(source);
  const auto qos = rclcpp::QoS(1).reliable().transient_local();
  std::vector<Message> messages;
  std::vector<Path> paths;
  auto trajectory = node->create_subscription<Message>(
      "/keyframe_trajectory", qos,
      [&](Message::ConstSharedPtr msg) { messages.push_back(*msg); });
  auto path = node->create_subscription<Path>(
      "/keyframe_path", qos,
      [&](Path::ConstSharedPtr msg) { paths.push_back(*msg); });
  ASSERT_TRUE(WaitForTrajectory(
      executor, [&] { return messages.size() == 1 && paths.size() == 1; }));
  EXPECT_EQ(messages[0].header.frame_id, "camera_init");
  source.pose_version = 2;
  source.optimized_frame_count = 1;
  source.parent_frame = PoseFrame::kMap;
  source.revision = 2;
  publisher.Publish(source);
  ASSERT_TRUE(WaitForTrajectory(
      executor, [&] { return messages.size() == 2 && paths.size() == 2; }));
  const auto& message = messages.back();
  EXPECT_EQ(message.producer_id, std::string(32, 'a'));
  EXPECT_EQ(message.revision, 2);
  EXPECT_EQ(message.pose_version, 2);
  EXPECT_EQ(message.frame_count, 2);
  EXPECT_EQ(message.optimized_frame_count, 1);
  EXPECT_EQ(message.status, Message::READY);
  EXPECT_EQ(message.header.frame_id, "atlas/map");
  EXPECT_EQ(message.header.stamp.sec, 9);
  EXPECT_EQ(message.header.stamp.nanosec, 500000000);
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
  EXPECT_EQ(paths.back().poses[0].header.stamp.sec, 10);
  EXPECT_EQ(paths.back().poses[0].header.stamp.nanosec, 250000000);
  EXPECT_DOUBLE_EQ(paths.back().poses[1].pose.position.x, -4);
  source.status = TrajectoryStatus::kResourceLimited;
  source.frame_count = 20001;
  source.keyframes.clear();
  source.revision = 3;
  publisher.Publish(source);
  ASSERT_TRUE(WaitForTrajectory(
      executor, [&] { return messages.size() == 3 && paths.size() == 3; }));
  EXPECT_EQ(messages.back().status, Message::RESOURCE_LIMITED);
  EXPECT_EQ(messages.back().frame_count, 20001);
  EXPECT_TRUE(messages.back().keyframes.empty());
  EXPECT_TRUE(paths.back().poses.empty());
  EXPECT_EQ(messages.back().header.stamp.sec, 0);
  EXPECT_EQ(messages.back().header.stamp.nanosec, 0);
  source.status = TrajectoryStatus::kError;
  source.error.assign(2048, 'e');
  source.revision = 4;
  publisher.Publish(source);
  ASSERT_TRUE(
      WaitForTrajectory(executor, [&] { return messages.size() == 4; }));
  EXPECT_EQ(messages.back().status, Message::ERROR);
  EXPECT_EQ(messages.back().error.size(), 1024);
}

TEST(KeyframeTrajectoryPublisherTest, InvalidTimesClearThePublishedState) {
  auto node = std::make_shared<rclcpp::Node>("trajectory_invalid_time_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  KeyframeTrajectoryPublisher publisher(*node, "map");
  const auto qos = rclcpp::QoS(1).reliable().transient_local();
  std::vector<Message> messages;
  std::vector<Path> paths;
  auto trajectory = node->create_subscription<Message>(
      "/keyframe_trajectory", qos,
      [&](Message::ConstSharedPtr msg) { messages.push_back(*msg); });
  auto path = node->create_subscription<Path>(
      "/keyframe_path", qos,
      [&](Path::ConstSharedPtr msg) { paths.push_back(*msg); });
  ASSERT_TRUE(WaitForTrajectory(executor, [&] {
    return node->count_publishers("/keyframe_trajectory") &&
           node->count_publishers("/keyframe_path");
  }));
  std::size_t count = 0;
  for (double time :
       {-1.0, 2147483648.0, std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()}) {
    auto source = test::TrajectoryFixture();
    source.revision = ++count;
    source.keyframes[0].timestamp = time;
    EXPECT_THROW(publisher.Publish(source), std::invalid_argument);
    ASSERT_TRUE(WaitForTrajectory(executor, [&] {
      return messages.size() == count && paths.size() == count;
    }));
    EXPECT_EQ(messages.back().status, Message::ERROR);
    EXPECT_TRUE(messages.back().keyframes.empty());
    EXPECT_TRUE(paths.back().poses.empty());
    EXPECT_EQ(messages.back().header.stamp.sec, 0);
    EXPECT_EQ(messages.back().header.stamp.nanosec, 0);
  }
}

TEST(KeyframeTrajectoryPublisherTest,
     SerializesTheDefaultTwentyThousandFrameBudget) {
  auto node = std::make_shared<rclcpp::Node>("trajectory_serialization_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  KeyframeTrajectoryPublisher publisher(*node, "map");
  auto source = test::TrajectoryFixture();
  source.frame_count = 20000;
  source.keyframes.resize(20000);
  for (std::size_t i = 0; i < source.keyframes.size(); ++i)
    source.keyframes[i] = {i, 100 + .01 * i, {}};
  publisher.Publish(source);
  Message received;
  bool done = false;
  auto subscription = node->create_subscription<Message>(
      "/keyframe_trajectory", rclcpp::QoS(1).reliable().transient_local(),
      [&](Message::ConstSharedPtr msg) {
        received = *msg;
        done = true;
      });
  ASSERT_TRUE(WaitForTrajectory(executor, [&] { return done; }));
  ASSERT_EQ(received.keyframes.size(), 20000);
  rclcpp::Serialization<Message> serialization;
  rclcpp::SerializedMessage serialized;
  serialization.serialize_message(&received, &serialized);
  EXPECT_GT(serialized.size(), 1440000u);
  EXPECT_LT(serialized.size(), 2u * 1024 * 1024);
  std::cout << "trajectory serialized bytes=" << serialized.size() << '\n';
}

}  // namespace
}  // namespace livo_atlas::ros2
