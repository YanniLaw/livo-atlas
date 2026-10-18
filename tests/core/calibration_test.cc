#include "livo_atlas/frontend/calibration.h"

#include <limits>
#include <vector>

#include <gtest/gtest.h>

namespace livo_atlas {
namespace {

const std::vector<double> kIdentity = {1, 0, 0, 0, 1, 0, 0, 0, 1};
const std::vector<double> kZeroTranslation = {0, 0, 0};

TEST(ExtrinsicsTest, ParsesRowMajorRotationAndTranslationDirection) {
  RigidTransform transform;
  ASSERT_TRUE(ParseExtrinsic({0, -1, 0, 1, 0, 0, 0, 0, 1}, {1, 2, 3},
                            &transform).empty());
  const Eigen::Vector3d point =
      transform.rotation * Eigen::Vector3d(2, 0, 0) + transform.translation;
  EXPECT_TRUE(point.isApprox(Eigen::Vector3d(1, 4, 3), 0));
}

TEST(ExtrinsicsTest, RejectsWrongArrayLengthsWithoutChangingOutput) {
  RigidTransform transform;
  transform.translation = Eigen::Vector3d(4, 5, 6);
  for (int length : {0, 8, 10}) {
    EXPECT_FALSE(ParseExtrinsic(std::vector<double>(length, 0),
                               kZeroTranslation, &transform).empty());
  }
  for (int length : {0, 2, 4}) {
    EXPECT_FALSE(ParseExtrinsic(kIdentity, std::vector<double>(length, 0),
                               &transform).empty());
  }
  EXPECT_TRUE(transform.rotation.isIdentity(0));
  EXPECT_TRUE(transform.translation.isApprox(Eigen::Vector3d(4, 5, 6), 0));
}

TEST(ExtrinsicsTest, PreservesRoundedAviaCalibrationExactly) {
  const std::vector<double> rotation = {
      0.00610193, -0.999863, -0.0154172, -0.00615449, 0.0153796,
      -0.999863, 0.999962, 0.00619598, -0.0060598};
  RigidTransform transform;
  ASSERT_TRUE(ParseExtrinsic(rotation, {0.0194384, 0.104689, -0.0251952},
                            &transform).empty());
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      EXPECT_DOUBLE_EQ(transform.rotation(row, col), rotation[3 * row + col]);
    }
  }
  EXPECT_DOUBLE_EQ(transform.translation(0), 0.0194384);
  EXPECT_DOUBLE_EQ(transform.translation(1), 0.104689);
  EXPECT_DOUBLE_EQ(transform.translation(2), -0.0251952);
}

TEST(ExtrinsicsTest, RejectsReflectionAndNonOrthonormalRotations) {
  RigidTransform transform;
  transform.rotation(0, 0) = -1;
  EXPECT_FALSE(ValidateExtrinsic(transform).empty());
  transform.rotation(0, 0) = 1.01;
  EXPECT_FALSE(ValidateExtrinsic(transform).empty());
  transform.rotation.setZero();
  EXPECT_FALSE(ValidateExtrinsic(transform).empty());
}

TEST(ExtrinsicsTest, RejectsNonFiniteValuesWithoutChangingOutput) {
  for (double invalid : {std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()}) {
    RigidTransform transform;
    transform.translation = Eigen::Vector3d(4, 5, 6);
    std::vector<double> rotation = kIdentity;
    rotation[1] = invalid;
    EXPECT_FALSE(ParseExtrinsic(rotation, kZeroTranslation, &transform).empty());
    EXPECT_FALSE(ParseExtrinsic(kIdentity, {0, invalid, 0}, &transform).empty());
    EXPECT_TRUE(transform.rotation.isIdentity(0));
    EXPECT_TRUE(transform.translation.isApprox(Eigen::Vector3d(4, 5, 6), 0));
    transform.translation(0) = invalid;
    EXPECT_FALSE(ValidateExtrinsic(transform).empty());
  }
}

CameraConfig PinholeConfig() {
  CameraConfig config;
  config.model = "Pinhole";
  config.width = 1280;
  config.height = 1024;
  config.scale = 0.5;
  config.fx = 1293.56944;
  config.fy = 1293.3155;
  config.cx = 626.91359;
  config.cy = 522.799224;
  config.distortion = {-0.076160, 0.123001, -0.00113, 0.000251};
  return config;
}

TEST(CameraValidationTest, AcceptsBothSupportedModelsAndFractionalDimensions) {
  CameraConfig config = PinholeConfig();
  config.width = 1281;
  EXPECT_TRUE(ValidateCameraConfig(config).empty());
  config.model = "EquidistantCamera";
  EXPECT_TRUE(ValidateCameraConfig(config).empty());
}

TEST(CameraValidationTest, RejectsUnsupportedModels) {
  CameraConfig config = PinholeConfig();
  for (const char* model : {"", "pinhole", "Ocam", "ATAN", "PolynomialCamera"}) {
    config.model = model;
    EXPECT_FALSE(ValidateCameraConfig(config).empty()) << model;
  }
}

TEST(CameraValidationTest, RejectsNonPositiveDimensions) {
  for (int dimension : {0, -1}) {
    CameraConfig config = PinholeConfig();
    config.width = dimension;
    EXPECT_FALSE(ValidateCameraConfig(config).empty());
    config = PinholeConfig();
    config.height = dimension;
    EXPECT_FALSE(ValidateCameraConfig(config).empty());
  }
}

TEST(CameraValidationTest, RejectsInvalidScaleAndEmptyScaledImages) {
  for (double scale : {0.0, -1.0, 0.0001,
                       std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()}) {
    CameraConfig config = PinholeConfig();
    config.scale = scale;
    EXPECT_FALSE(ValidateCameraConfig(config).empty());
  }
}

TEST(CameraValidationTest, RejectsDimensionAndPixelIndexOverflow) {
  CameraConfig config = PinholeConfig();
  config.scale = std::numeric_limits<double>::max();
  EXPECT_FALSE(ValidateCameraConfig(config).empty());
  config.scale = 1;
  config.width = 50000;
  config.height = 50000;
  EXPECT_FALSE(ValidateCameraConfig(config).empty());
  config.width = std::numeric_limits<int>::max();
  config.height = 1;
  config.scale = 2;
  EXPECT_FALSE(ValidateCameraConfig(config).empty());
}

TEST(CameraValidationTest, RejectsInvalidFocalLengths) {
  for (double focal : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()}) {
    CameraConfig config = PinholeConfig();
    config.fx = focal;
    EXPECT_FALSE(ValidateCameraConfig(config).empty());
    config = PinholeConfig();
    config.fy = focal;
    EXPECT_FALSE(ValidateCameraConfig(config).empty());
  }
}

TEST(CameraValidationTest, RejectsNonFinitePrincipalPointAndDistortion) {
  for (double invalid : {std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()}) {
    CameraConfig config = PinholeConfig();
    config.cx = invalid;
    EXPECT_FALSE(ValidateCameraConfig(config).empty());
    config = PinholeConfig();
    config.cy = invalid;
    EXPECT_FALSE(ValidateCameraConfig(config).empty());
    for (int index = 0; index < 4; ++index) {
      config = PinholeConfig();
      config.distortion[index] = invalid;
      EXPECT_FALSE(ValidateCameraConfig(config).empty());
    }
  }
}

TEST(CameraValidationTest, AcceptsFinitePrincipalPointsOutsideImage) {
  CameraConfig config = PinholeConfig();
  config.cx = -100;
  config.cy = 10000;
  EXPECT_TRUE(ValidateCameraConfig(config).empty());
}

TEST(CameraValidationTest, RejectsUnrepresentableScaledIntrinsics) {
  for (double CameraConfig::* member :
       {&CameraConfig::fx, &CameraConfig::fy, &CameraConfig::cx,
        &CameraConfig::cy}) {
    CameraConfig config = PinholeConfig();
    config.scale = 2;
    config.*member = std::numeric_limits<double>::max();
    EXPECT_FALSE(ValidateCameraConfig(config).empty());
    config.*member = 1e40;  // vikit stores a float OpenCV camera matrix.
    EXPECT_FALSE(ValidateCameraConfig(config).empty());
  }
  CameraConfig config = PinholeConfig();
  config.fx = std::numeric_limits<double>::denorm_min();
  EXPECT_FALSE(ValidateCameraConfig(config).empty());
  config = PinholeConfig();
  config.distortion[0] = 1e40;
  EXPECT_FALSE(ValidateCameraConfig(config).empty());
}

}  // namespace
}  // namespace livo_atlas
