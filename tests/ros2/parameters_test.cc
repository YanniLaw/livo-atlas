#include <gtest/gtest.h>

#include <filesystem>
#include <limits>

#include "ros2/fixtures.h"
#include "ros2/mapper_node.h"
#include "ros2/parameters.h"
namespace livo_atlas::ros2 {
TEST(Ros2ParametersTest, AcceptsMixedNumericCalibrationAndDeclaresReadOnly) {
  auto params = test::Parameters("/tmp/livo_atlas_p4/tests/parameter-test");
  rclcpp::Node node("parameter_test",
                    rclcpp::NodeOptions().parameter_overrides(params));
  const auto config = ReadParameters(node);
  EXPECT_EQ(config.session.frontend.calibration.camera.model, "Pinhole");
  EXPECT_DOUBLE_EQ(config.session.frontend.calibration.camera.fx, 200);
  ASSERT_TRUE(node.has_parameter("cam_fx"));
  EXPECT_TRUE(node.describe_parameter("cam_fx").read_only);
  EXPECT_FALSE(
      node.set_parameter(rclcpp::Parameter("cam_fx", 300.0)).successful);
}
TEST(Ros2ParametersTest, RejectsBadStartupBeforeCreatingOutput) {
  int index = 0;
  for (const auto& invalid : std::vector<rclcpp::Parameter>{
           {"cam_width", 0},
           {"cam_fx", 0},
           {"vio.grid_size", 0},
           {"vio.patch_size", 7},
           {"extrin_calib.extrinsic_R", std::vector<double>{1, 2}},
           {"extrin_calib.Pcl", std::vector<double>{0, 0}},
           {"output.directory", ""},
           {"publish.pub_scan_num", 0},
           {"qos.imu_depth", 0},
           {"qos.reliability", "invalid"},
           {"preprocess.point_filter_num", 0},
           {"preprocess.scan_line", 129},
           {"time_offset.imu_time_offset",
            std::numeric_limits<double>::infinity()}}) {
    SCOPED_TRACE(invalid.get_name());
    const auto directory =
        std::filesystem::temp_directory_path() /
        ("livo_atlas_p4/tests/invalid-" + std::to_string(++index));
    ASSERT_FALSE(std::filesystem::exists(directory));
    auto params = test::Parameters(directory.string());
    for (auto it = params.begin(); it != params.end();)
      it->get_name() == invalid.get_name() ? it = params.erase(it) : ++it;
    params.push_back(invalid);
    EXPECT_THROW(std::make_shared<MapperNode>(
                     rclcpp::NodeOptions().parameter_overrides(params)),
                 std::exception);
    EXPECT_FALSE(std::filesystem::exists(directory));
  }
}
}  // namespace livo_atlas::ros2
