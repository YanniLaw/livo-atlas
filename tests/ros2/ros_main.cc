#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "livo_atlas/common/logging.h"

int main(int argc, char** argv) {
  livo_atlas::InitializeLogging(argv[0]);
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
