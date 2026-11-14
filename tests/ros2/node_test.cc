#include "ros2/global_output_test_support.h"
#include <gtest/gtest.h>
#include <filesystem>
#include <memory>

#include <chrono>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <thread>
#include <vector>

#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/ros2/livo_node.h"
#include "livo_atlas/ros2/measurement_conversions.h"
#include "livo_atlas/ros2/node_options.h"
#include "ros2/fixtures.h"
#include "core/fixtures.h"

namespace livo_atlas::ros2 {
TEST(Ros2NodeTest, ProcessHookRunsWhileWaitingAndPropagatesErrors) {
  auto params = test::Parameters(
      (std::filesystem::temp_directory_path() /
       ("c2b-hook-" +
        std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count())))
          .string());
  rclcpp::Node node("c2b_hook",
                    rclcpp::NodeOptions().parameter_overrides(params));
  const auto config = ReadNodeOptions(node);
  LivoSystem system(config.livo.system);
  int calls = 0;
  LivoNode adapter(node, config, system, [&] {
    if (++calls == 2) throw 47;
  });
  EXPECT_EQ(adapter.ProcessNext(), SyncStatus::kWaiting);
  EXPECT_EQ(calls, 1);
  EXPECT_THROW(adapter.ProcessNext(), int);
  EXPECT_EQ(calls, 2);
  system.Finish();
}

TEST(Ros2NodeTest, ActualDdsPublishesLidarPoseCloudAndTransform) {
  auto params = test::Parameters(
      (std::filesystem::temp_directory_path() / "livo_atlas_tests/dds")
          .string());
  params.emplace_back("common.img_en", 0);
  params.emplace_back("common.imu_topic", "/p4_test/imu");
  params.emplace_back("common.lid_topic", "/p4_test/lidar");
  params.emplace_back("preprocess.point_filter_num", 1);
  auto node = std::make_shared<rclcpp::Node>(
      "laserMapping", rclcpp::NodeOptions().parameter_overrides(params));
  const auto config = ReadNodeOptions(*node);
  LivoSystem session(config.livo.system);
  LivoNode mapper(*node, config, session);
  auto peer = std::make_shared<rclcpp::Node>("p4_dds_peer");
  int poses = 0, clouds = 0, transforms = 0;
  auto odom = peer->create_subscription<nav_msgs::msg::Odometry>(
      "/aft_mapped_to_init", 10,
      [&](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        EXPECT_EQ(message->header.frame_id, "camera_init");
        EXPECT_EQ(message->child_frame_id, "aft_mapped");
        EXPECT_TRUE(std::isfinite(message->pose.pose.position.x));
        ++poses;
      });
  auto cloud = peer->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud_registered", 10,
      [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        EXPECT_EQ(message->header.frame_id, "camera_init");
        EXPECT_FALSE(message->fields.empty());
        ++clouds;
      });
  auto tf = peer->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", 10, [&](tf2_msgs::msg::TFMessage::ConstSharedPtr message) {
        if (!message->transforms.empty()) {
          EXPECT_EQ(message->transforms.front().child_frame_id, "aft_mapped");
          ++transforms;
        }
      });
  auto pub = peer->create_publisher<livox_ros_driver2::msg::CustomMsg>(
      "/p4_test/lidar", rclcpp::SensorDataQoS());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(peer);
  executor.add_node(node);
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(4);
  while (pub->get_subscription_count() == 0 &&
         std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(pub->get_subscription_count(), 1u);
  for (int frame = 0; frame < 3; ++frame) {
    livox_ros_driver2::msg::CustomMsg message;
    message.header.stamp = FromSeconds(100 + frame * .25);
    const auto scan = test::Scan(100);
    for (const auto& p : std::get<std::vector<LivoxPoint>>(scan.points)) {
      livox_ros_driver2::msg::CustomPoint q;
      q.x = p.x;
      q.y = p.y;
      q.z = p.z;
      q.line = p.line;
      q.tag = p.tag;
      q.offset_time = p.offset_time;
      message.points.push_back(q);
    }
    message.point_num = message.points.size();
    pub->publish(message);
    for (int i = 0; i < 20; ++i) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  EXPECT_EQ(poses, 3);
  EXPECT_EQ(clouds, 3);
  EXPECT_EQ(transforms, 3);
  testing::internal::CaptureStdout();
  session.Finish();
  mapper.ReportInputs();
  const auto output = testing::internal::GetCapturedStdout();
  EXPECT_NE(output.find("lidar=3 imu=0 image=0"), std::string::npos);
}
}  // namespace livo_atlas::ros2

namespace livo_atlas::ros2 {
TEST(Ros2NodeTest, PublishesFromTheInjectedSystem) {
  auto params = test::Parameters(
      (std::filesystem::temp_directory_path() / "livo_atlas_tests/injected")
          .string());
  params.emplace_back("common.img_en", 0);
  params.emplace_back("preprocess.point_filter_num", 1);
  auto node = std::make_shared<rclcpp::Node>(
      "injected_runtime", rclcpp::NodeOptions().parameter_overrides(params));
  const auto config = ReadNodeOptions(*node);
  LivoSystem session(config.livo.system);
  LivoNode adapter(*node, config, session);
  auto peer = std::make_shared<rclcpp::Node>("injected_peer");
  int poses = 0;
  auto sub = peer->create_subscription<nav_msgs::msg::Odometry>(
      "/aft_mapped_to_init", 10,
      [&](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        EXPECT_TRUE(std::isfinite(msg->pose.pose.position.x));
        ++poses;
      });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(peer);
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (sub->get_publisher_count() == 0 &&
         std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_GT(sub->get_publisher_count(), 0u);
  ASSERT_TRUE(session.AddLidar(test::Scan(100)));
  while (poses == 0 && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_EQ(poses, 1);
  session.Finish();
}
}  // namespace livo_atlas::ros2

namespace livo_atlas::ros2::test_support {
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
    mapper_ = std::make_unique<LivoNode>(*node_, parameters, *system_, [this] {
      ++polls_;
      if (pending_shift_ != 0) {
        Commit(pending_shift_);
        pending_shift_ = 0;
      }
    });
    ASSERT_TRUE(Wait([&] {
      return odom_sub_->get_publisher_count() && tf_sub_->get_publisher_count();
    }));
    // The node must advertise this, independent of backend build options.
    ASSERT_TRUE(Wait([&] { return global_sub_->get_publisher_count(); }));
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
}  // namespace livo_atlas::ros2::test_support
