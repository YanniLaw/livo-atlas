#include "ros1/global_output_test_support.h"

#include <opencv2/core.hpp>

namespace livo_atlas::ros1::test_support {
namespace {

// ros::Time is process-global; this executable never shares the old wall-time
// tests.
class GlobalClockTest : public GlobalOutputTest {
 protected:
  void SetUp() override {
    ros::Time::setNow(ros::Time(0));
    GlobalOutputTest::SetUp();
  }

  void SetTime(int seconds) {
    ros::Time::setNow(ros::Time(seconds, seconds == 0 ? 0 : 12345));
  }
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
  EXPECT_TRUE(global_[0].header.stamp.toNSec() <
              global_[1].header.stamp.toNSec());
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
  publisher_->PublishImuPropagation(result.estimate, ros::Time(10));
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
}  // namespace livo_atlas::ros1::test_support

int main(int argc, char** argv) {
  ros::init(argc, argv, "global_output_clock_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
