#include "livo_atlas/ros2/geometry_map_publisher.h"

#include <chrono>
#include <cstddef>
#include <cstring>
#include <functional>
#include <limits>
#include <stdexcept>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include "geometry_map_fixture.h"

#include "livo_atlas/common/logging.h"

namespace livo_atlas::ros2 {
namespace {
using Message = livo_atlas::msg::GeometryMap;
using Cloud = sensor_msgs::msg::PointCloud2;

bool WaitForGeometry(rclcpp::Executor& executor,
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

TEST(GeometryMapPublisherTest, ClosedNodeContextSkipsConversion) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>(
      "geometry_shutdown_test", rclcpp::NodeOptions().context(context));
  GeometryMapPublisher publisher(*node, "map");
  context->shutdown("test shutdown");
  auto source = test::GeometryMapFixture();
  source.timestamp = -1;
  EXPECT_NO_THROW(publisher.Publish(source));
}

TEST(GeometryMapPublisherTest, LatchedVersionedMapAndStandardCloud) {
  auto node = std::make_shared<rclcpp::Node>("geometry_publisher_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  GeometryMapPublisher publisher(*node, "atlas/map");
  auto source = test::GeometryMapFixture();
  publisher.Publish(source);
  const auto qos = rclcpp::QoS(1).reliable().transient_local();
  std::vector<Message> messages;
  std::vector<Cloud> clouds;
  auto map = node->create_subscription<Message>(
      "/geometry_map", qos,
      [&](Message::ConstSharedPtr msg) { messages.push_back(*msg); });
  auto cloud = node->create_subscription<Cloud>(
      "/global_map", qos,
      [&](Cloud::ConstSharedPtr msg) { clouds.push_back(*msg); });
  ASSERT_TRUE(WaitForGeometry(
      executor, [&] { return messages.size() == 1 && clouds.size() == 1; }));
  const auto& raw = messages.back();
  EXPECT_EQ(raw.header.frame_id, "camera_init");
  EXPECT_EQ(raw.header.stamp.sec, 9);
  EXPECT_EQ(raw.header.stamp.nanosec, 500000000u);
  EXPECT_EQ(raw.representation, "voxel_moments_v1");
  EXPECT_EQ(raw.fusion_policy, "primary_only");
  EXPECT_EQ(raw.sampling_policy, "geometry_rank_v1");
  EXPECT_EQ(raw.source_voxel_count, 5u);
  EXPECT_TRUE(raw.sampled);
  ASSERT_EQ(raw.sources.size(), 2u);
  EXPECT_EQ(raw.sources[0].membership_revision, 3u);
  EXPECT_EQ(raw.sources[1].anchor_keyframe_id, 2u);
  EXPECT_EQ(raw.sources[1].point_offset, 2u);
  EXPECT_EQ(raw.sources[1].point_count, 1u);
  EXPECT_EQ(raw.cloud.header, clouds.back().header);
  EXPECT_EQ(raw.cloud.data, clouds.back().data);
  EXPECT_EQ(raw.cloud.point_step, 16u);
  ASSERT_EQ(raw.cloud.fields.size(), 4u);
  EXPECT_EQ(raw.cloud.fields[3].name, "intensity");
  EXPECT_EQ(raw.cloud.fields[3].offset, 12u);
  ASSERT_EQ(raw.cloud.data.size(), 48u);
  float x;
  std::memcpy(&x, raw.cloud.data.data(), sizeof(x));
  EXPECT_FLOAT_EQ(x, 1.25f);
  source.pose_version = 2;
  source.optimized_frame_count = 3;
  source.parent_frame = PoseFrame::kMap;
  source.revision = 2;
  publisher.Publish(source);
  ASSERT_TRUE(WaitForGeometry(
      executor, [&] { return messages.size() == 2 && clouds.size() == 2; }));
  EXPECT_EQ(messages.back().header.frame_id, "atlas/map");
  EXPECT_EQ(messages.back().pose_version, 2u);
  EXPECT_EQ(messages.back().header, messages.back().cloud.header);
  EXPECT_EQ(messages.back().header, clouds.back().header);
  source.status = GeometryMapStatus::kResourceLimited;
  source.sources.clear();
  source.points.clear();
  source.source_voxel_count = 0;
  source.sampled = false;
  source.timestamp = 0;
  source.revision = 3;
  publisher.Publish(source);
  ASSERT_TRUE(WaitForGeometry(
      executor, [&] { return messages.size() == 3 && clouds.size() == 3; }));
  EXPECT_EQ(messages.back().status, Message::RESOURCE_LIMITED);
  EXPECT_TRUE(messages.back().sources.empty());
  EXPECT_TRUE(clouds.back().data.empty());
  EXPECT_EQ(clouds.back().fields.size(), 4u);
  EXPECT_EQ(messages.back().header.stamp.sec, 0);
  EXPECT_EQ(messages.back().header.stamp.nanosec, 0u);
}

TEST(GeometryMapPublisherTest, InvalidPayloadClearsBothTopics) {
  auto node = std::make_shared<rclcpp::Node>("geometry_invalid_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  GeometryMapPublisher publisher(*node, "map");
  const auto qos = rclcpp::QoS(1).reliable().transient_local();
  std::vector<Message> messages;
  std::vector<Cloud> clouds;
  auto map = node->create_subscription<Message>(
      "/geometry_map", qos,
      [&](Message::ConstSharedPtr msg) { messages.push_back(*msg); });
  auto cloud = node->create_subscription<Cloud>(
      "/global_map", qos,
      [&](Cloud::ConstSharedPtr msg) { clouds.push_back(*msg); });
  ASSERT_TRUE(WaitForGeometry(executor, [&] {
    return node->count_publishers("/geometry_map") &&
           node->count_publishers("/global_map");
  }));
  for (int scenario = 0; scenario < 5; ++scenario) {
    auto source = test::GeometryMapFixture();
    source.revision = scenario + 1;
    if (scenario == 0)
      source.points[0].position.x() = std::numeric_limits<double>::max();
    if (scenario == 1) source.timestamp = -1;
    if (scenario == 2) source.timestamp = 2147483648.0;
    if (scenario == 3)
      source.timestamp = std::numeric_limits<double>::quiet_NaN();
    if (scenario == 4) source.sources[1].point_offset = 1;
    EXPECT_THROW(publisher.Publish(source), std::invalid_argument);
    ASSERT_TRUE(WaitForGeometry(executor, [&] {
      return messages.size() == static_cast<std::size_t>(scenario + 1) &&
             clouds.size() == static_cast<std::size_t>(scenario + 1);
    }));
    EXPECT_EQ(messages.back().status, Message::ERROR);
    EXPECT_TRUE(messages.back().sources.empty());
    EXPECT_TRUE(clouds.back().data.empty());
    EXPECT_EQ(messages.back().header.stamp.sec, 0);
  }
}

TEST(GeometryMapPublisherTest, SerializesOneHundredThousandPointsWithinBudget) {
  auto node = std::make_shared<rclcpp::Node>("geometry_serialization_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  GeometryMapPublisher publisher(*node, "map");
  auto source = test::GeometryMapFixture();
  source.options.max_points = 100000;
  source.effective_max_points = 100000;
  source.source_voxel_count = 100000;
  source.sampled = false;
  source.sources[0].source_voxel_count = 50000;
  source.sources[0].point_count = 50000;
  source.sources[1].source_voxel_count = 50000;
  source.sources[1].point_offset = 50000;
  source.sources[1].point_count = 50000;
  source.points.resize(100000);
  publisher.Publish(source);
  Message received;
  bool done = false;
  auto subscription = node->create_subscription<Message>(
      "/geometry_map", rclcpp::QoS(1).reliable().transient_local(),
      [&](Message::ConstSharedPtr msg) {
        received = *msg;
        done = true;
      });
  ASSERT_TRUE(WaitForGeometry(executor, [&] { return done; }));
  EXPECT_EQ(received.cloud.width, 100000u);
  rclcpp::Serialization<Message> serialization;
  rclcpp::SerializedMessage serialized;
  serialization.serialize_message(&received, &serialized);
  EXPECT_LE(serialized.size(), 4096u + 128u * 2u + 16u * 100000u);
  LOG(INFO) << "geometry serialized bytes=" << serialized.size();
}

}  // namespace
}  // namespace livo_atlas::ros2
