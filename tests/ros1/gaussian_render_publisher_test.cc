#include "livo_atlas/ros1/gaussian_render_publisher.h"

#include <gtest/gtest.h>
#include <livo_atlas/GaussianRenderFrame.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <cmath>
#include <functional>
#include <vector>

#include "gaussian/render_publisher_test_helpers.h"
#include "livo_atlas/gaussian/gaussian_render_output.h"

namespace livo_atlas {
namespace {
using Message = GaussianRenderFrame;

bool Wait(const std::function<bool()>& ready) {
  const auto deadline = ros::WallTime::now() + ros::WallDuration(5);
  while (ros::ok() && ros::WallTime::now() < deadline) {
    ros::spinOnce();
    if (ready()) return true;
    ros::WallDuration(.001).sleep();
  }
  return false;
}

TEST(GaussianRenderPublisherTest, LateReaderGetsReadyThenClearedInvalidation) {
  ros::NodeHandle node("/gaussian_late");
  ros1::GaussianRenderPublisher publisher(node, "atlas/gaussian_optical",
                                          "map");
  auto source = test::ColoredPublication();
  ASSERT_TRUE(publisher.Publish(source));
  std::vector<Message> frames;
  std::vector<sensor_msgs::Image> images;
  auto subscription = node.subscribe<Message>(
      "gaussian_render", 10,
      [&](const Message::ConstPtr& value) { frames.push_back(*value); });
  auto mirror = node.subscribe<sensor_msgs::Image>(
      "gaussian_render/image", 10,
      [&](const sensor_msgs::Image::ConstPtr& value) {
        images.push_back(*value);
      });
  ASSERT_TRUE(Wait([&] { return frames.size() == 1 && images.size() == 1; }));
  test::CheckPublishedRender(frames.back());
  EXPECT_EQ(frames.back().header.stamp.toNSec(), 100010000000ULL);
  EXPECT_EQ(frames.back().image.data, images.back().data);
  EXPECT_EQ(frames.back().header, images.back().header);
  source.revision = 2;
  auto render = std::make_shared<GaussianRender>(*source.render);
  ++render->request.job_id;
  source.render = render;
  EXPECT_TRUE(publisher.Publish(source));
  ASSERT_TRUE(Wait([&] { return frames.size() == 2 && images.size() == 2; }));
  EXPECT_EQ(frames.back().job_id, 9007199254740994ULL);
  EXPECT_EQ(frames.back().header.stamp, frames.front().header.stamp);
  EXPECT_EQ(frames.back().revision, 2u);
  source.revision = 3;
  EXPECT_TRUE(publisher.Publish(source));
  ASSERT_TRUE(Wait([&] { return frames.size() == 3 && images.size() == 3; }));
  EXPECT_EQ(frames.back().revision, 3u);
  EXPECT_EQ(frames.back().job_id, 9007199254740994ULL);
  source.revision = 4;
  source.status = GaussianRenderStatus::kInvalidated;
  source.reason = "source_changed";
  source.render.reset();
  EXPECT_TRUE(publisher.Publish(source));
  ASSERT_TRUE(Wait([&] { return frames.size() == 4 && images.size() == 4; }));
  test::CheckClearedRender(frames.back());
  EXPECT_TRUE(images.back().data.empty());
  EXPECT_TRUE(frames.back().header.stamp.isZero());
  subscription.shutdown();
  bool late = false;
  auto reader = node.subscribe<Message>(
      "gaussian_render", 1, [&](const Message::ConstPtr& value) {
        EXPECT_EQ(value->status, Message::INVALIDATED);
        EXPECT_EQ(value->revision, 4u);
        test::CheckClearedRender(*value);
        late = true;
      });
  EXPECT_TRUE(Wait([&] { return late; }));
}

TEST(GaussianRenderPublisherTest, BadReadyPayloadReplacesBothCachesWithError) {
  ros::NodeHandle node("/gaussian_bad");
  ros1::GaussianRenderPublisher publisher(node, "atlas/gaussian_optical",
                                          "map");
  for (int scenario = 0; scenario < 9; ++scenario) {
    auto source = test::MalformedPublication(scenario);
    source.revision = scenario + 1;
    EXPECT_THROW(publisher.Publish(source), std::invalid_argument);
    Message received;
    sensor_msgs::Image image;
    bool frame_done = false, image_done = false;
    auto reader = node.subscribe<Message>("gaussian_render", 1,
                                          [&](const Message::ConstPtr& value) {
                                            received = *value;
                                            frame_done = true;
                                          });
    auto mirror = node.subscribe<sensor_msgs::Image>(
        "gaussian_render/image", 1,
        [&](const sensor_msgs::Image::ConstPtr& value) {
          image = *value;
          image_done = true;
        });
    ASSERT_TRUE(Wait([&] { return frame_done && image_done; }));
    EXPECT_EQ(received.status, Message::ERROR);
    EXPECT_EQ(received.revision, static_cast<std::uint64_t>(scenario + 1));
    test::CheckClearedRender(received);
    EXPECT_TRUE(image.data.empty());
    EXPECT_TRUE(received.header.stamp.isZero());
  }
}

TEST(GaussianRenderPublisherTest, SensorTimeLimitsAndMapPoseRemainExact) {
  ros::NodeHandle node("/gaussian_time");
  ros1::GaussianRenderPublisher publisher(node, "atlas/gaussian_optical",
                                          "atlas/map");
  std::vector<Message> frames;
  auto reader = node.subscribe<Message>(
      "gaussian_render", 10,
      [&](const Message::ConstPtr& value) { frames.push_back(*value); });
  ASSERT_TRUE(Wait([&] { return reader.getNumPublishers() > 0; }));
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
    ASSERT_TRUE(Wait([&] { return frames.size() == i + 1; }));
    EXPECT_EQ(frames.back().header.stamp.toNSec(), ns[i]);
    EXPECT_EQ(frames.back().pose_version, 9007199254740993ULL);
    EXPECT_EQ(frames.back().parent_frame, "atlas/map");
    EXPECT_DOUBLE_EQ(frames.back().parent_from_camera.position.x, 5);
  }
}

TEST(GaussianRenderPublisherTest, LiveContextReceivesJoinedStoppedFrame) {
  ros::NodeHandle node("/gaussian_stop");
  ros1::GaussianRenderPublisher publisher(node, "atlas/gaussian_optical",
                                          "map");
  GaussianRenderOutputOptions options;
  options.enabled = true;
  GaussianRenderOutput output(
      options, [&](const auto& source) { return publisher.Publish(source); });
  output.Observe(test::PublicationSnapshot());
  output.Finish();
  bool stopped = false;
  auto reader = node.subscribe<Message>(
      "gaussian_render", 1, [&](const Message::ConstPtr& value) {
        stopped = value->status == Message::STOPPED;
        test::CheckClearedRender(*value);
      });
  EXPECT_TRUE(Wait([&] { return stopped; }));
  EXPECT_GE(output.GetStats().published, 1u);
  EXPECT_EQ(output.GetStats().unavailable, 0u);
  EXPECT_TRUE(output.GetStats().finished);
}

}  // namespace
}  // namespace livo_atlas

int main(int argc, char** argv) {
  ros::init(argc, argv, "gaussian_render_publisher_test");
  ros::Time::setNow(ros::Time(0));  // Deliberately paused simulation clock.
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
