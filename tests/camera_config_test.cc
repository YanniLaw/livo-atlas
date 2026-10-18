#include "livo_atlas/camera/vikit_camera_factory.h"

#include <gtest/gtest.h>
#include <vikit/equidistant_camera.h>
#include <vikit/pinhole_camera.h>

#include "vio.h"

namespace livo_atlas {
namespace {

CameraConfig CameraCalibration(const std::string& model) {
  CameraConfig config;
  config.model = model;
  config.width = 1281;
  config.height = 1025;
  config.scale = 0.5;
  config.fx = 1293.56944;
  config.fy = 1293.3155;
  config.cx = 626.91359;
  config.cy = 522.799224;
  config.distortion = {-0.076160, 0.123001, -0.00113, 0.000251};
  return config;
}

void ExpectSameCamera(const vk::AbstractCamera& actual,
                     const vk::AbstractCamera& legacy) {
  EXPECT_EQ(actual.width(), 640);
  EXPECT_EQ(actual.height(), 512);
  EXPECT_DOUBLE_EQ(actual.scale(), 0.5);
  EXPECT_DOUBLE_EQ(actual.fx(), 646.78472);
  EXPECT_DOUBLE_EQ(actual.fy(), 646.65775);
  EXPECT_DOUBLE_EQ(actual.cx(), 313.456795);
  EXPECT_DOUBLE_EQ(actual.cy(), 261.399612);
  for (const Eigen::Vector3d& point :
       {Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(1, 2, 10),
        Eigen::Vector3d(-2, -1, 5)}) {
    EXPECT_TRUE(actual.world2cam(point).isApprox(legacy.world2cam(point), 0));
  }
  for (const Eigen::Vector2d& pixel :
       {Eigen::Vector2d(100, 150), Eigen::Vector2d(320, 256),
        Eigen::Vector2d(500, 400)}) {
    EXPECT_TRUE(actual.cam2world(pixel).isApprox(legacy.cam2world(pixel), 0));
  }
}

TEST(CameraFactoryTest, PinholeMatchesLegacyScalingAndProjection) {
  const CameraConfig config = CameraCalibration("Pinhole");
  const auto camera = CreateVikitCamera(config);
  ASSERT_NE(camera, nullptr);
  const vk::PinholeCamera legacy(
      1281, 1025, 0.5, 1293.56944, 1293.3155, 626.91359, 522.799224,
      -0.076160, 0.123001, -0.00113, 0.000251);
  ExpectSameCamera(*camera, legacy);
}

TEST(CameraFactoryTest, EquidistantMatchesLegacyScalingAndProjection) {
  const CameraConfig config = CameraCalibration("EquidistantCamera");
  const auto camera = CreateVikitCamera(config);
  ASSERT_NE(camera, nullptr);
  const vk::EquidistantCamera legacy(
      1281, 1025, 0.5, 1293.56944, 1293.3155, 626.91359, 522.799224,
      -0.076160, 0.123001, -0.00113, 0.000251);
  ExpectSameCamera(*camera, legacy);
}

TEST(CameraFactoryTest, RejectsInvalidCalibrationBeforeConstruction) {
  CameraConfig config = CameraCalibration("unsupported");
  EXPECT_EQ(CreateVikitCamera(config), nullptr);
  config = CameraCalibration("Pinhole");
  config.fx = 0;
  EXPECT_EQ(CreateVikitCamera(config), nullptr);
  config = CameraCalibration("Pinhole");
  config.width = 0;
  EXPECT_EQ(CreateVikitCamera(config), nullptr);
}

TEST(VioExtrinsicsTest, PreservesNonIdentityFrameDirectionsAndTranslations) {
  vk::PinholeCamera camera(512, 512, 1, 200, 200, 256, 256);
  VIOManager manager;
  manager.cam = &camera;
  manager.grid_size = 32;
  manager.raycast_en = false;
  manager.patch_size = 8;
  manager.patch_pyrimid_level = 4;
  Eigen::Matrix3d imu_from_lidar;
  imu_from_lidar << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  manager.setImuToLidarExtrinsic(Eigen::Vector3d(1, 2, 3), imu_from_lidar);
  RigidTransform camera_from_lidar;
  camera_from_lidar.rotation << 1, 0, 0, 0, 0, -1, 0, 1, 0;
  camera_from_lidar.translation = Eigen::Vector3d(4, 5, 6);
  manager.SetLidarToCameraExtrinsic(camera_from_lidar);
  manager.initializeVIO();
  Eigen::Matrix3d camera_from_imu;
  camera_from_imu << 0, 1, 0, 0, 0, -1, -1, 0, 0;
  EXPECT_TRUE(manager.Rci.isApprox(camera_from_imu, 0));
  EXPECT_TRUE(manager.Pci.isApprox(Eigen::Vector3d(2, 8, 7), 0));
  const Eigen::Vector3d camera_center_in_imu =
      -manager.Rci.transpose() * manager.Pci;
  EXPECT_TRUE(camera_center_in_imu.isApprox(Eigen::Vector3d(7, -2, 8), 0));
}

}  // namespace
}  // namespace livo_atlas
