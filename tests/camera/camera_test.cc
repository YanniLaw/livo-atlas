#include "livo_atlas/camera/camera_factory.h"

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "livo_atlas/camera/equidistant_camera.h"
#include "livo_atlas/camera/pinhole_camera.h"
#include "test_images.h"

namespace livo_atlas {
namespace {

std::vector<std::string> Fields(const std::string& line) {
  std::istringstream input(line);
  std::vector<std::string> fields;
  for (std::string field; std::getline(input, field, ',');) {
    fields.push_back(field);
  }
  return fields;
}

CameraConfig Config() {
  return {"Pinhole", 1281, 1025, 0.5, 1293.56944, 1293.3155,
          626.91359, 522.799224, {-0.076160, 0.123001, -0.00113, 0.000251}};
}

TEST(CameraTest, MatchesFrozenProjectionAndUnitBearings) {
  std::ifstream input(std::string(TEST_DATA_DIR) + "/camera/data/legacy_camera.csv");
  ASSERT_TRUE(input.is_open());
  std::unique_ptr<Camera> camera;
  int rows = 0;
  for (std::string line; std::getline(input, line);) {
    SCOPED_TRACE(line);
    const auto fields = Fields(line);
    if (fields[0] == "C") {
      CameraConfig config;
      config.model = fields[2];
      config.width = std::stoi(fields[3]);
      config.height = std::stoi(fields[4]);
      config.scale = std::stod(fields[5]);
      config.fx = std::stod(fields[6]);
      config.fy = std::stod(fields[7]);
      config.cx = std::stod(fields[8]);
      config.cy = std::stod(fields[9]);
      for (int i = 0; i < 4; ++i) config.distortion[i] = std::stod(fields[10 + i]);
      camera = CreateCamera(config);
      ASSERT_NE(camera, nullptr);
      continue;
    }
    ASSERT_NE(camera, nullptr);
    const Eigen::Vector3d point(std::stod(fields[2]), std::stod(fields[3]),
                                std::stod(fields[4]));
    Eigen::Vector3d actual;
    if (fields[0] == "P") {
      actual.head<2>() = camera->Project(point);
      actual.z() = 0;
    } else {
      actual = camera->Unproject(Eigen::Vector2d(point.head<2>()));
    }
    for (int i = 0; i < 3; ++i) {
      const double expected = std::stod(fields[5 + i]);
      if (std::isnan(expected)) EXPECT_TRUE(std::isnan(actual[i]));
      // Frozen vikit used native FMA. The baseline-ISA control is bit exact;
      // native vs baseline differs by at most 3 ULP on the captured platform.
      else EXPECT_DOUBLE_EQ(actual[i], expected);
    }
    ++rows;
  }
  EXPECT_EQ(rows, 1140);
}

TEST(CameraTest, KeepsFrozenUndistortedPixelBytes) {
  std::ifstream input(std::string(TEST_DATA_DIR) + "/camera/data/legacy_undistort.csv");
  ASSERT_TRUE(input.is_open());
  for (std::string line; std::getline(input, line);) {
    const auto fields = Fields(line);
    const int id = std::stoi(fields[0]);
    auto config = Config();
    if (id == 0) config.distortion = {};
    if (id == 4) config.distortion[0] = 0;
    if (id == 6) config.distortion[0] = 1e-7;
    if (id == 7) config.distortion[0] = 1.0000001e-7;
    PinholeCamera camera(config);
    cv::Mat output;
    camera.UndistortImage(test_data::MakeImage(640, 512, 3), &output);
    EXPECT_EQ(test_data::ImageHash(output), std::stoull(fields[1])) << id;
  }
}

TEST(CameraTest, ProjectsHandCalculatedPinholeAndScalesOnce) {
  PinholeCamera simple({"Pinhole", 32, 16, 1, 4, 8, 8, 4, {}});
  EXPECT_EQ(simple.Project(Eigen::Vector3d(0.125, 0.25, 1)),
            Eigen::Vector2d(8.5, 6));
  PinholeCamera camera(Config());
  EXPECT_EQ(camera.width(), 640);
  EXPECT_EQ(camera.height(), 512);
  EXPECT_DOUBLE_EQ(camera.scale(), 0.5);
  EXPECT_DOUBLE_EQ(camera.fx(), 646.78472);
  EXPECT_DOUBLE_EQ(camera.fy(), 646.65775);
  EXPECT_DOUBLE_EQ(camera.cx(), 313.456795);
  EXPECT_DOUBLE_EQ(camera.cy(), 261.399612);
}

TEST(CameraTest, ChecksIntegerFrameAndPyramidBorders) {
  PinholeCamera camera(Config());
  EXPECT_TRUE(camera.IsInFrame({3, 3}, 3));
  EXPECT_TRUE(camera.IsInFrame({636, 508}, 3));
  EXPECT_FALSE(camera.IsInFrame({637, 508}, 3));
  EXPECT_FALSE(camera.IsInFrame({2, 3}, 3));
  EXPECT_TRUE(camera.IsInFrame({316, 252}, 3, 1));
  EXPECT_FALSE(camera.IsInFrame({317, 252}, 3, 1));
  EXPECT_FALSE(camera.IsInFrame({0, 0}, 0, 31));
  EXPECT_FALSE(camera.IsInFrame({0, 0}, -1));
}

TEST(CameraTest, RejectsBadConfigurationBeforeModelAllocation) {
  for (const std::string& model : {"Pinhole", "EquidistantCamera"}) {
    for (int variant = 0; variant < 5; ++variant) {
      auto config = Config();
      config.model = model;
      if (variant == 0) config.width = -1;
      if (variant == 1) config.scale = std::numeric_limits<double>::infinity();
      if (variant == 2) config.scale = 1e100;
      if (variant == 3) config.fx = 0;
      if (variant == 4) config.distortion[3] = std::nan("");
      EXPECT_EQ(CreateCamera(config), nullptr);
      if (model == "Pinhole") EXPECT_THROW(PinholeCamera{config}, std::invalid_argument);
      else EXPECT_THROW(EquidistantCamera{config}, std::invalid_argument);
    }
  }
}

}  // namespace
}  // namespace livo_atlas
