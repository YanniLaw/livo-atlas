#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>

#include "ros2/fixtures.h"
#include "ros2/mapper_node.h"
#include "ros2/parameters.h"
namespace livo_atlas::ros2 {
TEST(Ros2ParametersTest, AcceptsMixedNumericCalibrationAndDeclaresReadOnly) {
  auto params = test::Parameters("/tmp/livo_atlas_p4/tests/parameter-test");
  params.emplace_back("common.imu_topic", "~/imu");
  auto options = rclcpp::NodeOptions().parameter_overrides(params);
  options.arguments({"--ros-args", "-r", "/parameter_test/imu:=/review/imu"});
  rclcpp::Node node("parameter_test", options);
  const auto config = ReadParameters(node);
  EXPECT_EQ(config.session.frontend.calibration.camera.model, "Pinhole");
  EXPECT_EQ(
      node.get_node_topics_interface()->resolve_topic_name(config.imu_topic),
      "/review/imu");
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

namespace livo_atlas::ros2 {
TEST(Ros2ParametersTest, InvalidTopicsAndCovariancesPreserveExistingOutputs) {
  const auto seed = std::chrono::steady_clock::now().time_since_epoch().count();
  int index = 0;
  for (const auto& invalid :
       std::vector<rclcpp::Parameter>{{"common.lid_topic", "bad topic"},
                                      {"common.imu_topic", "bad topic"},
                                      {"common.img_topic", "bad topic"},
                                      {"vio.img_point_cov", 0.0},
                                      {"vio.img_point_cov", -1.0},
                                      {"imu.acc_cov", -1.0},
                                      {"imu.gyr_cov", -1.0},
                                      {"vio.inv_expo_cov", -1.0}}) {
    SCOPED_TRACE(invalid.get_name());
    const auto directory =
        std::filesystem::temp_directory_path() /
        ("livo_atlas_p4/tests/preserve-" + std::to_string(seed) + "-" +
         std::to_string(++index));
    const std::vector<std::string> names = {
        "mat_pre.txt", "mat_out.txt", "imu.txt", "Colmap/sparse/0/images.txt",
        "Colmap/images/keep.png"};
    for (const auto& name : names) {
      std::filesystem::create_directories((directory / name).parent_path());
      std::ofstream stream(directory / name);
      stream << "keep existing output";
    }
    auto params = test::Parameters(directory.string());
    params.emplace_back("pcd_save.colmap_output_en", true);
    params.push_back(invalid);
    EXPECT_THROW(std::make_shared<MapperNode>(
                     rclcpp::NodeOptions().parameter_overrides(params)),
                 std::exception);
    for (const auto& name : names) {
      std::ifstream stream(directory / name);
      const std::string contents((std::istreambuf_iterator<char>(stream)), {});
      EXPECT_EQ(contents, "keep existing output") << name;
    }
  }
}
TEST(Ros2ParametersTest,
     InvalidTopicsAndCovariancesDoNotCreateOutputDirectories) {
  const auto seed = std::chrono::steady_clock::now().time_since_epoch().count();
  int index = 0;
  for (const auto& invalid :
       std::vector<rclcpp::Parameter>{{"common.lid_topic", "bad topic"},
                                      {"common.imu_topic", ""},
                                      {"common.img_topic", "bad topic"},
                                      {"vio.img_point_cov", 0.0},
                                      {"imu.acc_cov", -1.0},
                                      {"imu.gyr_cov", -1.0},
                                      {"vio.inv_expo_cov", -1.0}}) {
    SCOPED_TRACE(invalid.get_name());
    const auto directory =
        std::filesystem::temp_directory_path() /
        ("livo_atlas_p4/tests/absent-" + std::to_string(seed) + "-" +
         std::to_string(++index));
    auto params = test::Parameters(directory.string());
    params.push_back(invalid);
    EXPECT_THROW(std::make_shared<MapperNode>(
                     rclcpp::NodeOptions().parameter_overrides(params)),
                 std::exception);
    EXPECT_FALSE(std::filesystem::exists(directory));
  }
}
}  // namespace livo_atlas::ros2
