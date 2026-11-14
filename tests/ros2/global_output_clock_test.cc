#include "ros2/global_output_test_support.h"

#include <rosgraph_msgs/msg/clock.hpp>
#include <opencv2/core.hpp>

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

}  // namespace
}  // namespace livo_atlas::ros2::test_support
