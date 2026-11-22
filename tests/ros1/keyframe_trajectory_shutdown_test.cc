#include "livo_atlas/ros1/keyframe_trajectory_publisher.h"

#include <gtest/gtest.h>
#include <ros/ros.h>

#include "trajectory_fixture.h"

#include "livo_atlas/common/logging.h"

TEST(KeyframeTrajectoryShutdownTest, ClosedRosSkipsConversionAndPublication) {
  ros::NodeHandle node;
  livo_atlas::ros1::KeyframeTrajectoryPublisher publisher(node, "map");
  auto source = livo_atlas::test::TrajectoryFixture();
  source.keyframes[0].timestamp = -1;
  ros::shutdown();
  EXPECT_NO_THROW(publisher.Publish(source));
}

int main(int argc, char** argv) {
  livo_atlas::InitializeLogging(argv[0]);
  ros::init(argc, argv, "keyframe_trajectory_shutdown_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
