#include "livo_atlas/ros2/gaussian_render_publisher.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>

#include "gaussian/render_publisher_test_helpers.h"
#include "livo_atlas/gaussian/gaussian_render_output.h"

namespace livo_atlas::ros2 {
namespace {
using Message = livo_atlas::msg::GaussianRenderFrame;
using Image = sensor_msgs::msg::Image;

bool Wait(rclcpp::Executor& executor, const std::function<bool()>& ready) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (ready()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

TEST(GaussianRenderPublisherTest, LateReaderGetsReadyThenClearedInvalidation) {
  auto node = std::make_shared<rclcpp::Node>("late", "/gaussian_late");
  node->set_parameter(rclcpp::Parameter("use_sim_time", true));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  // Runtime parameter events are asynchronous; establish the paused clock
  // before testing publication, as in the existing global clock fixture.
  ASSERT_TRUE(Wait(executor, [&] { return node->now().nanoseconds() == 0; }));
  GaussianRenderPublisher publisher(*node, "atlas/gaussian_optical", "map");
  auto source = test::ColoredPublication();
  ASSERT_TRUE(publisher.Publish(source));
  const auto qos = rclcpp::QoS(1).reliable().transient_local();
  std::vector<Message> frames;
  std::vector<Image> images;
  auto reader = node->create_subscription<Message>(
      "gaussian_render", qos,
      [&](Message::ConstSharedPtr value) { frames.push_back(*value); });
  auto mirror = node->create_subscription<Image>(
      "gaussian_render/image", qos,
      [&](Image::ConstSharedPtr value) { images.push_back(*value); });
  ASSERT_TRUE(
      Wait(executor, [&] { return frames.size() == 1 && images.size() == 1; }));
  test::CheckPublishedRender(frames.back());
  EXPECT_EQ(frames.back().header.stamp.sec, 100);
  EXPECT_EQ(frames.back().header.stamp.nanosec, 10000000u);
  EXPECT_EQ(frames.back().image.data, images.back().data);
  EXPECT_EQ(frames.back().header, images.back().header);
  EXPECT_EQ(node->now().nanoseconds(), 0);
  source.revision = 2;
  auto render = std::make_shared<GaussianRender>(*source.render);
  ++render->request.job_id;
  source.render = render;
  EXPECT_TRUE(publisher.Publish(source));
  ASSERT_TRUE(
      Wait(executor, [&] { return frames.size() == 2 && images.size() == 2; }));
  EXPECT_EQ(frames.back().job_id, 9007199254740994ULL);
  EXPECT_EQ(frames.back().header.stamp, frames.front().header.stamp);
  EXPECT_EQ(frames.back().revision, 2u);
  source.revision = 3;
  EXPECT_TRUE(publisher.Publish(source));
  ASSERT_TRUE(
      Wait(executor, [&] { return frames.size() == 3 && images.size() == 3; }));
  EXPECT_EQ(frames.back().revision, 3u);
  EXPECT_EQ(frames.back().job_id, 9007199254740994ULL);
  source.revision = 4;
  source.status = GaussianRenderStatus::kInvalidated;
  source.reason = "source_changed";
  source.render.reset();
  EXPECT_TRUE(publisher.Publish(source));
  ASSERT_TRUE(
      Wait(executor, [&] { return frames.size() == 4 && images.size() == 4; }));
  test::CheckClearedRender(frames.back());
  EXPECT_TRUE(images.back().data.empty());
  EXPECT_EQ(frames.back().header.stamp.sec, 0);
  EXPECT_EQ(frames.back().header.stamp.nanosec, 0u);
  reader.reset();
  bool late = false;
  auto late_reader = node->create_subscription<Message>(
      "gaussian_render", qos, [&](Message::ConstSharedPtr value) {
        EXPECT_EQ(value->status, Message::INVALIDATED);
        EXPECT_EQ(value->revision, 4u);
        test::CheckClearedRender(*value);
        late = true;
      });
  EXPECT_TRUE(Wait(executor, [&] { return late; }));
}

TEST(GaussianRenderPublisherTest, BadReadyPayloadReplacesBothCachesWithError) {
  auto node = std::make_shared<rclcpp::Node>("bad", "/gaussian_bad");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  GaussianRenderPublisher publisher(*node, "atlas/gaussian_optical", "map");
  const auto qos = rclcpp::QoS(1).reliable().transient_local();
  for (int scenario = 0; scenario < 12; ++scenario) {
    auto source = test::MalformedPublication(scenario);
    source.revision = scenario + 1;
    EXPECT_THROW(publisher.Publish(source), std::invalid_argument);
    Message received;
    Image image;
    bool frame_done = false, image_done = false;
    auto reader = node->create_subscription<Message>(
        "gaussian_render", qos, [&](Message::ConstSharedPtr value) {
          received = *value;
          frame_done = true;
        });
    auto mirror = node->create_subscription<Image>(
        "gaussian_render/image", qos, [&](Image::ConstSharedPtr value) {
          image = *value;
          image_done = true;
        });
    ASSERT_TRUE(Wait(executor, [&] { return frame_done && image_done; }));
    EXPECT_EQ(received.status, Message::ERROR);
    EXPECT_EQ(received.revision, static_cast<std::uint64_t>(scenario + 1));
    test::CheckClearedRender(received);
    EXPECT_TRUE(image.data.empty());
    EXPECT_EQ(received.header.stamp.sec, 0);
    EXPECT_EQ(received.header.stamp.nanosec, 0u);
  }
}

TEST(GaussianRenderPublisherTest, SensorTimeLimitsAndMapPoseRemainExact) {
  auto node = std::make_shared<rclcpp::Node>("time", "/gaussian_time");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  GaussianRenderPublisher publisher(*node, "atlas/gaussian_optical",
                                    "atlas/map");
  std::vector<Message> frames;
  auto reader = node->create_subscription<Message>(
      "gaussian_render", rclcpp::QoS(1).reliable().transient_local(),
      [&](Message::ConstSharedPtr value) { frames.push_back(*value); });
  const double times[] = {0, .9999999996, std::nextafter(2147483648., 0.)};
  const std::uint64_t ns[] = {0, 1000000000ULL, 2147483647999999762ULL};
  for (std::size_t i = 0; i < 3; ++i) {
    auto source = test::PublicationSnapshot();
    auto render = std::make_shared<GaussianRender>(*source.render);
    source.revision = i + 1;
    source.pose_version = render->request.pose_version = 9007199254740993ULL;
    source.optimized_frame_count = render->request.optimized_frame_count = 4;
    source.parent_frame = render->request.parent_frame = PoseFrame::kMap;
    for (auto& block : render->request.blocks) {
      block.target.pose_version = source.pose_version;
      block.target.optimized_frame_count = 4;
      block.target.parent_frame = PoseFrame::kMap;
    }
    render->capture_time = times[i];
    source.render = render;
    EXPECT_TRUE(publisher.Publish(source));
    ASSERT_TRUE(Wait(executor, [&] { return frames.size() == i + 1; }));
    EXPECT_EQ((static_cast<std::uint64_t>(frames.back().header.stamp.sec) *
                   1000000000ULL +
               frames.back().header.stamp.nanosec),
              ns[i]);
    EXPECT_EQ(frames.back().pose_version, 9007199254740993ULL);
    EXPECT_EQ(frames.back().parent_frame, "atlas/map");
    EXPECT_DOUBLE_EQ(frames.back().parent_from_camera.position.x, 5);
  }
}

TEST(GaussianRenderPublisherTest, LiveContextReceivesJoinedStoppedFrame) {
  auto node = std::make_shared<rclcpp::Node>("stop", "/gaussian_stop");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  GaussianRenderPublisher publisher(*node, "atlas/gaussian_optical", "map");
  GaussianRenderOutputOptions options;
  options.enabled = true;
  GaussianRenderOutput output(
      options, [&](const auto& source) { return publisher.Publish(source); });
  output.Observe(test::PublicationSnapshot());
  output.Finish();
  bool stopped = false;
  auto reader = node->create_subscription<Message>(
      "gaussian_render", rclcpp::QoS(1).reliable().transient_local(),
      [&](Message::ConstSharedPtr value) {
        stopped = value->status == Message::STOPPED;
        test::CheckClearedRender(*value);
      });
  EXPECT_TRUE(Wait(executor, [&] { return stopped; }));
  EXPECT_GE(output.GetStats().published, 1u);
  EXPECT_EQ(output.GetStats().unavailable, 0u);
  EXPECT_TRUE(output.GetStats().finished);
}

TEST(GaussianRenderPublisherTest, ClosedContextDoesNotClaimPublication) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>(
      "gaussian_shutdown", rclcpp::NodeOptions().context(context));
  GaussianRenderPublisher publisher(*node, "atlas/gaussian_optical", "map");
  context->shutdown("test shutdown");
  EXPECT_FALSE(publisher.Publish(test::MalformedPublication(0)));
}

}  // namespace
}  // namespace livo_atlas::ros2
