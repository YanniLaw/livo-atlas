#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <vikit/atan_camera.h>
#include <vikit/equidistant_camera.h>
#include <vikit/omni_camera.h>
#include <vikit/pinhole_camera.h>
#include <vikit/polynomial_camera.h>

#include "livo_atlas/core/camera_config.h"
#include "livo_atlas/ros/camera_config.h"

namespace livo_atlas {
namespace {

PinholeCameraConfig ValidPinholeConfig() {
  PinholeCameraConfig config;
  config.width = 1280;
  config.height = 1024;
  config.scale = 0.5;
  config.fx = 1293.56944;
  config.fy = 1293.3155;
  config.cx = 626.91359;
  config.cy = 522.799224;
  config.d0 = -0.076160;
  config.d1 = 0.123001;
  config.d2 = -0.00113;
  config.d3 = 0.000251;
  return config;
}

TEST(CameraConfigTest, PinholeValidationAndScalingMatchVikit) {
  CameraConfig config;
  config.model = ValidPinholeConfig();
  std::string error = "stale error";

  ASSERT_TRUE(ValidateCameraConfig(config, &error)) << error;
  EXPECT_TRUE(error.empty());
  EXPECT_DOUBLE_EQ(0.5, CameraImageResizeScale(config));

  const std::unique_ptr<vk::AbstractCamera> camera =
      ros::MakeVikitCameraModel(config);
  ASSERT_TRUE(camera != nullptr);
  EXPECT_NE(nullptr, dynamic_cast<vk::PinholeCamera*>(camera.get()));
  EXPECT_EQ(640, camera->width());
  EXPECT_EQ(512, camera->height());
  EXPECT_DOUBLE_EQ(646.78472, camera->fx());
  EXPECT_DOUBLE_EQ(646.65775, camera->fy());
  EXPECT_DOUBLE_EQ(313.456795, camera->cx());
  EXPECT_DOUBLE_EQ(261.399612, camera->cy());
}

TEST(CameraConfigTest, EquidistantKeepsItsOwnDistortionAndScale) {
  EquidistantCameraConfig equidistant;
  equidistant.width = 720;
  equidistant.height = 540;
  equidistant.scale = 1.0;
  equidistant.fx = 351.31400364193297;
  equidistant.fy = 351.4911744656785;
  equidistant.cx = 367.8522793375995;
  equidistant.cy = 253.8402144980996;
  equidistant.k1 = -0.03696737352869157;
  equidistant.k2 = -0.008917880497032812;
  equidistant.k3 = 0.008912969593422046;
  equidistant.k4 = -0.0037685977496087313;

  CameraConfig config;
  config.model = equidistant;
  const std::unique_ptr<vk::AbstractCamera> camera =
      ros::MakeVikitCameraModel(config);

  ASSERT_TRUE(camera != nullptr);
  EXPECT_NE(nullptr, dynamic_cast<vk::EquidistantCamera*>(camera.get()));
  EXPECT_EQ(720, camera->width());
  EXPECT_EQ(540, camera->height());
  EXPECT_DOUBLE_EQ(equidistant.fx, camera->fx());
  EXPECT_DOUBLE_EQ(equidistant.fy, camera->fy());
  EXPECT_DOUBLE_EQ(1.0, CameraImageResizeScale(config));
}

TEST(CameraConfigTest, PolynomialDoesNotApplyScale) {
  PolynomialCameraConfig polynomial;
  polynomial.width = 640;
  polynomial.height = 480;
  polynomial.fx = 400.0;
  polynomial.fy = 401.0;
  polynomial.cx = 320.0;
  polynomial.cy = 240.0;
  polynomial.skew = 2.5;
  polynomial.k2 = 0.01;
  polynomial.k3 = -0.02;
  polynomial.k4 = 0.03;
  polynomial.k5 = -0.04;
  polynomial.k6 = 0.05;
  polynomial.k7 = -0.06;

  CameraConfig config;
  config.model = polynomial;
  const std::unique_ptr<vk::AbstractCamera> camera =
      ros::MakeVikitCameraModel(config);

  ASSERT_TRUE(camera != nullptr);
  EXPECT_NE(nullptr, dynamic_cast<vk::PolynomialCamera*>(camera.get()));
  EXPECT_EQ(640, camera->width());
  EXPECT_EQ(480, camera->height());
  EXPECT_DOUBLE_EQ(400.0, camera->fx());
  EXPECT_DOUBLE_EQ(401.0, camera->fy());
  EXPECT_DOUBLE_EQ(1.0, camera->scale());
  EXPECT_DOUBLE_EQ(1.0, CameraImageResizeScale(config));
}

TEST(CameraConfigTest, AtanUsesLegacyNormalizedIntrinsicsAndUnitResize) {
  AtanCameraConfig atan;
  atan.width = 640;
  atan.height = 480;
  atan.fx = 0.5;
  atan.fy = 0.6;
  atan.cx = 0.5;
  atan.cy = 0.5;
  atan.distortion = 0.1;

  CameraConfig config;
  config.model = atan;
  const std::unique_ptr<vk::AbstractCamera> camera =
      ros::MakeVikitCameraModel(config);

  ASSERT_TRUE(camera != nullptr);
  EXPECT_NE(nullptr, dynamic_cast<vk::ATANCamera*>(camera.get()));
  EXPECT_EQ(640, camera->width());
  EXPECT_EQ(480, camera->height());
  EXPECT_DOUBLE_EQ(320.0, camera->fx());
  EXPECT_DOUBLE_EQ(288.0, camera->fy());
  EXPECT_DOUBLE_EQ(319.5, camera->cx());
  EXPECT_DOUBLE_EQ(239.5, camera->cy());
  EXPECT_DOUBLE_EQ(1.0, camera->scale());
  EXPECT_DOUBLE_EQ(1.0, CameraImageResizeScale(config));
}

TEST(CameraConfigTest, OcamUsesCalibrationFileAndUnitResize) {
#ifndef LIVO_ATLAS_TEST_DATA_DIR
#error "LIVO_ATLAS_TEST_DATA_DIR must be set for camera_config_test"
#endif
  OcamCameraConfig ocam;
  ocam.calibration_file =
      std::string(LIVO_ATLAS_TEST_DATA_DIR) +
      "/ocam_camera_test_calibration.txt";
  CameraConfig config;
  config.model = ocam;

  const std::unique_ptr<vk::AbstractCamera> camera =
      ros::MakeVikitCameraModel(config);

  ASSERT_TRUE(camera != nullptr);
  EXPECT_NE(nullptr, dynamic_cast<vk::OmniCamera*>(camera.get()));
  EXPECT_EQ(640, camera->width());
  EXPECT_EQ(480, camera->height());
  // OmniCamera leaves AbstractCamera::scale_ unset. This path must use the
  // explicit legacy resize factor instead of reading that field.
  EXPECT_DOUBLE_EQ(1.0, CameraImageResizeScale(config));
}

TEST(CameraConfigTest, RejectsInvalidDimensionsScaleIntrinsicsAndDistortion) {
  CameraConfig config;
  std::string error;

  PinholeCameraConfig pinhole = ValidPinholeConfig();
  pinhole.width = 0;
  config.model = pinhole;
  EXPECT_FALSE(ValidateCameraConfig(config, &error));
  EXPECT_NE(std::string::npos, error.find("cam_width"));

  pinhole = ValidPinholeConfig();
  pinhole.scale = 0.0001;
  config.model = pinhole;
  EXPECT_FALSE(ValidateCameraConfig(config, &error));
  EXPECT_NE(std::string::npos, error.find("scale"));

  pinhole = ValidPinholeConfig();
  pinhole.scale = std::numeric_limits<double>::infinity();
  config.model = pinhole;
  EXPECT_FALSE(ValidateCameraConfig(config, &error));
  EXPECT_NE(std::string::npos, error.find("scale"));

  pinhole = ValidPinholeConfig();
  pinhole.fx = std::numeric_limits<double>::quiet_NaN();
  config.model = pinhole;
  EXPECT_FALSE(ValidateCameraConfig(config, &error));
  EXPECT_NE(std::string::npos, error.find("cam_fx"));

  pinhole = ValidPinholeConfig();
  pinhole.d2 = std::numeric_limits<double>::infinity();
  config.model = pinhole;
  EXPECT_FALSE(ValidateCameraConfig(config, &error));
  EXPECT_NE(std::string::npos, error.find("cam_d2"));
}

TEST(CameraConfigTest, RejectsNonFiniteModelSpecificCoefficients) {
  CameraConfig config;
  std::string error;

  PolynomialCameraConfig polynomial;
  polynomial.width = 640;
  polynomial.height = 480;
  polynomial.fx = 400.0;
  polynomial.fy = 401.0;
  polynomial.cx = 320.0;
  polynomial.cy = 240.0;
  polynomial.skew = std::numeric_limits<double>::quiet_NaN();
  config.model = polynomial;
  EXPECT_FALSE(ValidateCameraConfig(config, &error));
  EXPECT_NE(std::string::npos, error.find("cam_skew"));

  AtanCameraConfig atan;
  atan.width = 640;
  atan.height = 480;
  atan.fx = 0.5;
  atan.fy = 0.5;
  atan.cx = 0.5;
  atan.cy = 0.5;
  atan.distortion = std::numeric_limits<double>::infinity();
  config.model = atan;
  EXPECT_FALSE(ValidateCameraConfig(config, &error));
  EXPECT_NE(std::string::npos, error.find("cam_d0"));
}

TEST(CameraConfigTest, RequiresAnOcamCalibrationFile) {
  CameraConfig config;
  config.model = OcamCameraConfig{};
  std::string error;

  EXPECT_FALSE(ValidateCameraConfig(config, &error));
  EXPECT_NE(std::string::npos, error.find("cam_calib_file"));
}

}  // namespace
}  // namespace livo_atlas

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
