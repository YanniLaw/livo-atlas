#include "livo_atlas/camera/camera_factory.h"

#include <algorithm>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include "livo_atlas/camera/pinhole_camera.h"

#include "livo_atlas/frontend/vision/vio.h"

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

void ExpectScaledCamera(const Camera& actual) {
  EXPECT_EQ(actual.width(), 640);
  EXPECT_EQ(actual.height(), 512);
  EXPECT_DOUBLE_EQ(actual.scale(), 0.5);
  EXPECT_DOUBLE_EQ(actual.fx(), 646.78472);
  EXPECT_DOUBLE_EQ(actual.fy(), 646.65775);
  EXPECT_DOUBLE_EQ(actual.cx(), 313.456795);
  EXPECT_DOUBLE_EQ(actual.cy(), 261.399612);
}

TEST(CameraFactoryTest, PinholePreservesScaledIntrinsics) {
  const auto camera = CreateCamera(CameraCalibration("Pinhole"));
  ASSERT_NE(camera, nullptr);
  ExpectScaledCamera(*camera);
  EXPECT_EQ(camera->Project(Eigen::Vector3d(0, 0, 1)),
            Eigen::Vector2d(313.456795, 261.399612));
}

TEST(CameraFactoryTest, EquidistantPreservesScaledIntrinsicsAndAxisBranch) {
  const auto camera = CreateCamera(CameraCalibration("EquidistantCamera"));
  ASSERT_NE(camera, nullptr);
  ExpectScaledCamera(*camera);
  EXPECT_EQ(camera->Project(Eigen::Vector3d(0, 0, 1)), Eigen::Vector2d(0, 0));
}

TEST(CameraFactoryTest, RejectsInvalidCalibrationBeforeConstruction) {
  CameraConfig config = CameraCalibration("unsupported");
  EXPECT_EQ(CreateCamera(config), nullptr);
  config = CameraCalibration("Pinhole");
  config.fx = 0;
  EXPECT_EQ(CreateCamera(config), nullptr);
  config = CameraCalibration("Pinhole");
  config.width = 0;
  EXPECT_EQ(CreateCamera(config), nullptr);
}

TEST(FrameTest, ConcurrentInstancesReceiveUniqueIds) {
  PinholeCamera camera({"Pinhole", 32, 32, 1, 10, 10, 16, 16, {}});
  const cv::Mat image(32, 32, CV_8UC1, cv::Scalar(0));
  constexpr int kThreads = 8;
  constexpr int kFramesPerThread = 20000;
  std::vector<std::vector<int>> ids(
      kThreads, std::vector<int>(kFramesPerThread));
  std::mutex mutex;
  std::condition_variable condition;
  int ready = 0;
  bool start = false;
  std::vector<std::thread> workers;
  for (int thread = 0; thread < kThreads; ++thread) {
    workers.emplace_back([&, thread] {
      {
        std::unique_lock<std::mutex> lock(mutex);
        ++ready;
        condition.notify_all();
        condition.wait(lock, [&] { return start; });
      }
      for (int i = 0; i < kFramesPerThread; ++i) {
        Frame frame(&camera, image);
        ids[thread][i] = frame.id_;
      }
    });
  }
  {
    std::unique_lock<std::mutex> lock(mutex);
    condition.wait(lock, [&] { return ready == kThreads; });
    start = true;
  }
  condition.notify_all();
  for (auto& worker : workers) worker.join();
  std::vector<int> all_ids;
  for (const auto& sequence : ids) {
    all_ids.insert(all_ids.end(), sequence.begin(), sequence.end());
  }
  std::sort(all_ids.begin(), all_ids.end());
  EXPECT_EQ(std::adjacent_find(all_ids.begin(), all_ids.end()), all_ids.end());
}

TEST(VioExtrinsicsTest, PreservesNonIdentityFrameDirectionsAndTranslations) {
  PinholeCamera camera({"Pinhole", 512, 512, 1, 200, 200, 256, 256, {}});
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
