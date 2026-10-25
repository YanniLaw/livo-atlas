#ifndef LIVO_ATLAS_TESTS_ROS2_FIXTURES_H_
#define LIVO_ATLAS_TESTS_ROS2_FIXTURES_H_
#include <cstdint>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

#include "core/fixtures.h"
#include "livo_atlas/core/livo_system.h"
namespace livo_atlas::ros2::test {
using livo_atlas::test::Options;
using livo_atlas::test::Scan;
inline std::vector<rclcpp::Parameter> Parameters(const std::string& output) {
  return {{"cam_model", "Pinhole"},
          {"cam_width", 512},
          {"cam_height", 512},
          {"cam_fx", 200},
          {"cam_fy", 200.0},
          {"cam_cx", 256},
          {"cam_cy", 256},
          {"vio.grid_size", 32},
          {"output.directory", output},
          {"extrin_calib.extrinsic_R",
           std::vector<int64_t>{1, 0, 0, 0, 1, 0, 0, 0, 1}},
          {"extrin_calib.extrinsic_T", std::vector<double>{0, 0, 0}},
          {"extrin_calib.Rcl", std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1}},
          {"extrin_calib.Pcl", std::vector<int64_t>{0, 0, 0}}};
}
}  // namespace livo_atlas::ros2::test
#endif  // LIVO_ATLAS_TESTS_ROS2_FIXTURES_H_
