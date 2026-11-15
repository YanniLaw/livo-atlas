#include "livo_atlas/ros1/livo_node.h"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/core.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include "core/fixtures.h"
#include "livo_atlas/ros1/measurement_conversions.h"
#include "ros1/global_output_test_support.h"

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

TEST(Ros1NodeTest, ReportsStartupBufferOverflow) {
  ros::NodeHandle node;
  auto parameters = Options(kLivox);
  parameters.livo.system = test::Options(FrontendMode::kLidarInertialVisual);
  LivoSystem session(parameters.livo.system);
  LivoNode adapter(node, parameters, session);
  const cv::Mat image(1, 1, CV_8UC3, cv::Scalar(1, 2, 3));
  for (int i = 0; i < 2001; ++i)
    ASSERT_TRUE(session.AddImu(
        {100 + i * .001, Eigen::Vector3d::Zero(), {0, 0, -9.81}}));
  for (int i = 0; i < 9; ++i)
    ASSERT_TRUE(session.AddImage({100 + i * .03, image}));
  session.Finish();
  testing::internal::CaptureStdout();
  adapter.ReportInputs();
  const auto output = testing::internal::GetCapturedStdout();
  EXPECT_NE(output.find("lidar=0 imu=0 image=0"), std::string::npos);
  EXPECT_NE(output.find("imu_buffered=2001 image_buffered=9"),
            std::string::npos);
  EXPECT_NE(output.find("imu_evicted=1 image_evicted=1 imu_rejected=0"),
            std::string::npos);
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

TEST(Ros1NodeTest, ProcessHookRunsWhileWaitingAndPropagatesErrors) {
  ros::NodeHandle node;
  const auto parameters = Options(kLivox);
  LivoSystem system(parameters.livo.system);
  int calls = 0;
  LivoNode adapter(node, parameters, system, [&] {
    if (++calls == 2) throw 47;
  });
  EXPECT_EQ(adapter.ProcessNext(), SyncStatus::kWaiting);
  EXPECT_EQ(calls, 1);
  EXPECT_THROW(adapter.ProcessNext(), int);
  EXPECT_EQ(calls, 2);
  system.Finish();
}
}  // namespace
}  // namespace livo_atlas::ros1

namespace livo_atlas::ros1::test_support {
namespace {

class GlobalNodeTest : public GlobalOutputTest {
 protected:
  void OpenNode() {
    NodeOptions parameters;
    auto options = ::livo_atlas::test::Options(FrontendMode::kLidarOnly);
    options.keyframes.enabled = options.submaps.enabled = true;
    options.keyframes.time_threshold = 0;
    options.submap_mode = SubmapMode::kOnline;
    directory_ = options.exporter.directory;
    parameters.livo.system = PrepareSystemOptions(options);
    parameters.livo.global_output = {true, true, "map"};
    system_ = std::make_unique<LivoSystem>(parameters.livo.system);
    mapper_ = std::make_unique<LivoNode>(node_, parameters, *system_, [this] {
      ++polls_;
      if (pending_shift_ != 0) {
        Commit(pending_shift_);
        pending_shift_ = 0;
      }
    });
    ASSERT_TRUE(Wait([&] {
      return odom_sub_.getNumPublishers() && tf_sub_.getNumPublishers();
    }));
    // The node must advertise this, independent of backend build options.
    ASSERT_TRUE(Wait([&] { return global_sub_.getNumPublishers(); }));
  }

  void Commit(double shift) {
    const auto snapshot = system_->GetPoseSnapshot();
    PoseUpdate update{snapshot->version, snapshot->frame_count, snapshot->poses,
                      PoseTailPolicy::kProjectedLastOptimizedAnchor};
    for (auto& pose : update.poses) pose.map_from_body.translation.x() += shift;
    system_->ApplyPoseUpdate(update);
  }

  void Sample(int i) {
    ASSERT_TRUE(system_->AddLidar(::livo_atlas::test::Scan(100 + .25 * i)));
    ASSERT_EQ(mapper_->ProcessNext(), SyncStatus::kReady);
    ASSERT_TRUE(Wait(
        [&] { return odometry_.size() == static_cast<std::size_t>(i + 1); }));
    Drain();
  }

  void TearDown() override {
    mapper_.reset();
    if (system_) system_->Finish();
    system_.reset();
    std::filesystem::remove_all(directory_);
  }

  std::filesystem::path directory_;
  std::unique_ptr<LivoSystem> system_;
  std::unique_ptr<LivoNode> mapper_;
  int polls_ = 0;
  double pending_shift_ = 0;
};

TEST_F(GlobalNodeTest, GlobalOutputTracksOnlyCommittedOwnerVersion) {
  OpenNode();
  ASSERT_FALSE(HasFatalFailure());
  pending_shift_ = 3;
  Sample(0);
  ASSERT_FALSE(HasFatalFailure());
  ASSERT_EQ(global_.size(), 1);
  ASSERT_EQ(transforms_.size(), 1);
  ASSERT_EQ(transforms_[0].transforms.size(), 2);
  EXPECT_DOUBLE_EQ(transforms_[0].transforms[0].transform.translation.x, 0);
  EXPECT_NEAR(global_[0].pose.position.x, odometry_[0].pose.pose.position.x,
              1e-9);
  EXPECT_EQ(system_->GetMapCorrection().pose_version, 1u);
  Sample(1);
  ASSERT_FALSE(HasFatalFailure());
  ASSERT_EQ(global_.size(), 2);
  EXPECT_NEAR(global_[1].pose.position.x, odometry_[1].pose.pose.position.x + 3,
              1e-9);
  EXPECT_NEAR(transforms_[1].transforms[0].transform.translation.x, 3, 1e-9);
  const auto first_count = system_->GetMapCorrection().frame_count;
  pending_shift_ = 4;
  const int before = polls_;
  EXPECT_EQ(mapper_->ProcessNext(), SyncStatus::kWaiting);
  EXPECT_GT(polls_, before);
  EXPECT_EQ(system_->GetMapCorrection().pose_version, 2u);
  Drain();
  EXPECT_EQ(global_.size(), 2);
  EXPECT_EQ(transforms_.size(), 2);
  Sample(2);
  ASSERT_FALSE(HasFatalFailure());
  ASSERT_EQ(global_.size(), 3);
  EXPECT_NEAR(global_[2].pose.position.x, odometry_[2].pose.pose.position.x + 7,
              1e-9);
  EXPECT_NEAR(transforms_[2].transforms[0].transform.translation.x, 7, 1e-9);
  EXPECT_GE(system_->GetMapCorrection().frame_count, first_count);
  EXPECT_EQ(system_->GetMapCorrection().pose_version, 2u);
}

TEST_F(GlobalNodeTest, FinalDrainDoesNotInventPublication) {
  OpenNode();
  ASSERT_FALSE(HasFatalFailure());
  Sample(0);
  ASSERT_FALSE(HasFatalFailure());
  ASSERT_EQ(global_.size(), 1);
  // Main has left its callback loop before draining the owner. Stop the test
  // node timers too, while the observer remains alive to catch extra output.
  mapper_.reset();
  system_->SealInput();
  // The owner can still commit a finished backend result after input is sealed.
  Commit(5);
  system_->Finish();
  EXPECT_EQ(system_->GetMapCorrection().pose_version, 1u);
  EXPECT_TRUE(std::filesystem::is_regular_file(
      directory_ / "keyframes/online_submaps/poses/1.yml"));
  Drain();
  EXPECT_EQ(global_.size(), 1);
  EXPECT_EQ(transforms_.size(), 1);
  EXPECT_EQ(odometry_.size(), 1);
}

}  // namespace
}  // namespace livo_atlas::ros1::test_support

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_callback_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
