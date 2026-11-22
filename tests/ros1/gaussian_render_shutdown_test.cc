#include <gtest/gtest.h>
#include <ros/ros.h>

#include "gaussian/render_publisher_test_helpers.h"
#include "livo_atlas/ros1/gaussian_render_publisher.h"

#include "livo_atlas/common/logging.h"

TEST(GaussianRenderShutdownTest, ClosedRosDoesNotClaimPublication) {
  ros::NodeHandle node;
  livo_atlas::ros1::GaussianRenderPublisher publisher(
      node, "atlas/gaussian_optical", "map");
  ros::shutdown();
  EXPECT_FALSE(publisher.Publish(livo_atlas::test::MalformedPublication(0)));
}

int main(int argc, char** argv) {
  livo_atlas::InitializeLogging(argv[0]);
  ros::init(argc, argv, "gaussian_render_shutdown_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
