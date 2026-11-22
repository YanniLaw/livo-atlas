#include "livo_atlas/ros1/geometry_map_publisher.h"

#include <cstddef>
#include <cstring>
#include <functional>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>
#include <livo_atlas/GeometryMap.h>
#include <ros/ros.h>
#include <ros/serialization.h>
#include <sensor_msgs/PointCloud2.h>

#include "geometry_map_fixture.h"

#include "livo_atlas/common/logging.h"

namespace livo_atlas {
namespace {

bool WaitForGeometry(const std::function<bool()>& ready) {
  const auto deadline = ros::WallTime::now() + ros::WallDuration(5);
  while (ros::ok() && ros::WallTime::now() < deadline) {
    ros::spinOnce();
    if (ready()) return true;
    ros::WallDuration(.001).sleep();
  }
  return false;
}

TEST(GeometryMapPublisherTest, LatchedVersionedMapAndStandardCloud) {
  ros::NodeHandle node;
  ros1::GeometryMapPublisher publisher(node, "atlas/map");
  auto source = test::GeometryMapFixture();
  publisher.Publish(source);
  std::vector<GeometryMap> messages;
  std::vector<sensor_msgs::PointCloud2> clouds;
  auto map = node.subscribe<GeometryMap>(
      "/geometry_map", 10,
      [&](const GeometryMap::ConstPtr& msg) { messages.push_back(*msg); });
  auto cloud = node.subscribe<sensor_msgs::PointCloud2>(
      "/global_map", 10, [&](const sensor_msgs::PointCloud2::ConstPtr& msg) {
        clouds.push_back(*msg);
      });
  ASSERT_TRUE(WaitForGeometry(
      [&] { return messages.size() == 1 && clouds.size() == 1; }));
  const auto& raw = messages.back();
  EXPECT_EQ(raw.header.frame_id, "camera_init");
  EXPECT_EQ(raw.header.stamp.toNSec(), 9500000000LL);
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
      [&] { return messages.size() == 2 && clouds.size() == 2; }));
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
      [&] { return messages.size() == 3 && clouds.size() == 3; }));
  EXPECT_EQ(messages.back().status, GeometryMap::RESOURCE_LIMITED);
  EXPECT_TRUE(messages.back().sources.empty());
  EXPECT_TRUE(clouds.back().data.empty());
  EXPECT_EQ(clouds.back().fields.size(), 4u);
  EXPECT_TRUE(messages.back().header.stamp.isZero());
}

TEST(GeometryMapPublisherTest, InvalidPayloadClearsBothTopics) {
  ros::NodeHandle node;
  ros1::GeometryMapPublisher publisher(node, "map");
  std::vector<GeometryMap> messages;
  std::vector<sensor_msgs::PointCloud2> clouds;
  auto map = node.subscribe<GeometryMap>(
      "/geometry_map", 10,
      [&](const GeometryMap::ConstPtr& msg) { messages.push_back(*msg); });
  auto cloud = node.subscribe<sensor_msgs::PointCloud2>(
      "/global_map", 10, [&](const sensor_msgs::PointCloud2::ConstPtr& msg) {
        clouds.push_back(*msg);
      });
  ASSERT_TRUE(WaitForGeometry(
      [&] { return map.getNumPublishers() && cloud.getNumPublishers(); }));
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
    ASSERT_TRUE(WaitForGeometry([&] {
      return messages.size() == static_cast<std::size_t>(scenario + 1) &&
             clouds.size() == static_cast<std::size_t>(scenario + 1);
    }));
    EXPECT_EQ(messages.back().status, GeometryMap::ERROR);
    EXPECT_TRUE(messages.back().sources.empty());
    EXPECT_TRUE(clouds.back().data.empty());
    EXPECT_TRUE(messages.back().header.stamp.isZero());
  }
}

TEST(GeometryMapPublisherTest, SerializesOneHundredThousandPointsWithinBudget) {
  ros::NodeHandle node;
  ros1::GeometryMapPublisher publisher(node, "map");
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
  GeometryMap received;
  bool done = false;
  auto subscription = node.subscribe<GeometryMap>(
      "/geometry_map", 1, [&](const GeometryMap::ConstPtr& msg) {
        received = *msg;
        done = true;
      });
  ASSERT_TRUE(WaitForGeometry([&] { return done; }));
  EXPECT_EQ(received.cloud.width, 100000u);
  const auto bytes = ros::serialization::serializationLength(received);
  EXPECT_LE(bytes, 4096u + 128u * 2u + 16u * 100000u);
  LOG(INFO) << "geometry serialized bytes=" << bytes;
}

}  // namespace
}  // namespace livo_atlas
