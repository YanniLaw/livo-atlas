#include <gtest/gtest.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "core/fixtures.h"
#include "livo_atlas/ros1/livo_node.h"
#include "livo_atlas/ros1/measurement_conversions.h"
namespace livo_atlas::ros1 {
namespace {
bool WaitFor(const std::function<bool()>& condition) {
  const auto until = ros::WallTime::now() + ros::WallDuration(3);
  while (ros::ok() && ros::WallTime::now() < until) {
    ros::spinOnce();
    if (condition()) return true;
    ros::WallDuration(.005).sleep();
  }
  return false;
}
void PumpCallbacks() {
  const auto until = ros::WallTime::now() + ros::WallDuration(.1);
  while (ros::WallTime::now() < until) {
    ros::spinOnce();
    ros::WallDuration(.002).sleep();
  }
}
NodeOptions Options(int lidar_type) {
  NodeOptions params;
  params.livo.system =
      PrepareSystemOptions(test::Options(FrontendMode::kLidarOnly));
  params.livo.system.preprocess.lidar_type = lidar_type;
  const auto name =
      "/convergence_adapter_" + std::to_string(ros::WallTime::now().toNSec());
  params.livo.lidar_topic = name + "/lidar";
  params.livo.imu_topic = name + "/imu";
  params.livo.image_topic = name + "/image";
  return params;
}
void VerifyRollbackPipeline(int lidar_type) {
  ros::NodeHandle node;
  auto parameters = Options(lidar_type);
  LivoSystem session(parameters.livo.system);
  LivoNode adapter(node, parameters, session);
  std::vector<nav_msgs::Odometry> poses;
  std::vector<PointCloud> clouds;
  auto odom = node.subscribe<nav_msgs::Odometry>(
      "/aft_mapped_to_init", 10,
      [&](const nav_msgs::Odometry::ConstPtr& msg) { poses.push_back(*msg); });
  auto cloud = node.subscribe<sensor_msgs::PointCloud2>(
      "/cloud_registered", 10,
      [&](const sensor_msgs::PointCloud2::ConstPtr& msg) {
        PointCloud points;
        pcl::fromROSMsg(*msg, points);
        clouds.push_back(std::move(points));
      });
  ros::Publisher publisher = lidar_type == kLivox
                                 ? node.advertise<livox_ros_driver::CustomMsg>(
                                       parameters.livo.lidar_topic, 10)
                                 : node.advertise<sensor_msgs::PointCloud2>(
                                       parameters.livo.lidar_topic, 10);
  ASSERT_TRUE(WaitFor([&] {
    return publisher.getNumSubscribers() == 1 && odom.getNumPublishers() &&
           cloud.getNumPublishers();
  }));
  auto publish = [&](double time, int marker, int invalid = 0) {
    const auto scan = test::Scan(time);
    if (lidar_type == kLivox) {
      livox_ros_driver::CustomMsg msg;
      msg.header.stamp.fromSec(time);
      for (const auto& p : std::get<std::vector<LivoxPoint>>(scan.points)) {
        livox_ros_driver::CustomPoint q;
        q.x = p.x;
        q.y = p.y;
        q.z = p.z;
        q.tag = p.tag;
        q.line = p.line;
        q.offset_time = p.offset_time;
        q.reflectivity = marker;
        msg.points.push_back(q);
      }
      msg.point_num = msg.points.size();
      if (invalid == 1) ++msg.point_num;
      if (invalid == 2) {
        msg.points.clear();
        msg.point_num = 0;
      }
      publisher.publish(msg);
    } else {
      pcl::PointCloud<VelodynePoint> points;
      for (const auto& p : std::get<std::vector<LivoxPoint>>(scan.points)) {
        VelodynePoint q{};
        q.x = p.x;
        q.y = p.y;
        q.z = p.z;
        q.intensity = marker;
        q.ring = p.line;
        q.time = p.offset_time * 1e-9;
        points.push_back(q);
      }
      sensor_msgs::PointCloud2 msg;
      pcl::toROSMsg(points, msg);
      msg.header.stamp.fromSec(time);
      if (invalid == 1) msg.data.pop_back();
      if (invalid == 2) {
        msg.width = 0;
        msg.row_step = 0;
        msg.data.clear();
      }
      publisher.publish(msg);
    }
  };
  publish(100, 23);
  publish(90, 44, 1);
  publish(91, 44, 2);
  PumpCallbacks();
  ASSERT_EQ(adapter.ProcessNext(), SyncStatus::kReady);
  ASSERT_TRUE(WaitFor([&] { return poses.size() == 1 && clouds.size() == 1; }));
  ASSERT_FALSE(clouds.back().empty());
  for (const auto& p : clouds.back()) EXPECT_FLOAT_EQ(p.intensity, 23);
  publish(200, 55);
  publish(199, 77);
  PumpCallbacks();
  ASSERT_EQ(adapter.ProcessNext(), SyncStatus::kReady);
  ASSERT_TRUE(WaitFor([&] { return poses.size() == 2 && clouds.size() == 2; }));
  for (const auto& p : clouds.back()) EXPECT_FLOAT_EQ(p.intensity, 77);
  for (const auto& p : poses)
    EXPECT_TRUE(std::isfinite(p.pose.pose.position.x));
  EXPECT_EQ(adapter.ProcessNext(), SyncStatus::kWaiting);
  session.Finish();
  testing::internal::CaptureStdout();
  adapter.ReportInputs();
  EXPECT_NE(
      testing::internal::GetCapturedStdout().find("lidar=5 imu=0 image=0"),
      std::string::npos);
}
TEST(Ros1NodeTest, LivoxTransportRejectsBadRollbackWithoutErasingPendingInput) {
  VerifyRollbackPipeline(kLivox);
}
TEST(Ros1NodeTest,
     StandardTransportRejectsBadRollbackWithoutErasingPendingInput) {
  VerifyRollbackPipeline(kVelodyne);
}
TEST(Ros1NodeTest, PublishesFromTheInjectedSystem) {
  ros::NodeHandle node;
  auto parameters = Options(kLivox);
  LivoSystem session(parameters.livo.system);
  LivoNode adapter(node, parameters, session);
  int poses = 0;
  auto sub = node.subscribe<nav_msgs::Odometry>(
      "/aft_mapped_to_init", 10, [&](const nav_msgs::Odometry::ConstPtr& msg) {
        EXPECT_TRUE(std::isfinite(msg->pose.pose.position.x));
        ++poses;
      });
  ASSERT_TRUE(WaitFor([&] { return sub.getNumPublishers() > 0; }));
  ASSERT_TRUE(session.AddLidar(test::Scan(100)));
  ASSERT_EQ(adapter.ProcessNext(), SyncStatus::kReady);
  ASSERT_TRUE(WaitFor([&] { return poses == 1; }));
  session.Finish();
}
}  // namespace
}  // namespace livo_atlas::ros1
int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_callback_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
