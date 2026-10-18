#include "vio.h"

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>
#include <vikit/pinhole_camera.h>

namespace livo_atlas {
namespace {

class VioSamplingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    manager_.cam = &camera_;
    manager_.state = &state_;
    manager_.state_propagat = &prior_;
    manager_.grid_size = 32;
    manager_.raycast_en = false;
    manager_.patch_size = 8;
    manager_.patch_pyrimid_level = 4;
    manager_.max_iterations = 1;
    manager_.img_point_cov = 100;
    manager_.exposure_estimate_en = true;
    manager_.inverse_composition_en = false;
    manager_.setImuToLidarExtrinsic(V3D::Zero(), M3D::Identity());
    manager_.SetLidarToCameraExtrinsic(RigidTransform{});
    manager_.initializeVIO();
    manager_.total_points = 1;
    manager_.visual_submap->voxel_points = {&point_};
    manager_.visual_submap->search_levels = {2};
    manager_.visual_submap->warp_patch = {std::vector<float>(256, 10)};
    manager_.visual_submap->inv_expo_list = {1};
    manager_.visual_submap->errors = {0};
    manager_.G.setZero();
  }

  StatesGroup Update(int pixel, int level, std::uint8_t padding,
               bool update_covariance = false) {
    state_ = StatesGroup();
    prior_ = state_;
    point_.pos_ = V3D((pixel - 256.0) / 200, (pixel - 256.0) / 200, 1);
    // Defined adjacent storage exposes reads beyond the logical image without
    // relying on a particular allocator or an actual out-of-allocation read.
    constexpr int kPadding = 192 * 512;
    std::vector<std::uint8_t> storage(512 * 512 + 2 * kPadding, padding);
    cv::Mat image(512, 512, CV_8UC1, storage.data() + kPadding);
    image.setTo(20);
    if (update_covariance) {
      manager_.new_frame_ = std::make_unique<Frame>(&camera_, image);
      manager_.computeJacobianAndUpdateEKF(image);
    } else {
      manager_.updateState(image, level);
    }
    return state_;
  }

  vk::PinholeCamera camera_{512, 512, 1, 200, 200, 256, 256};
  VisualPoint point_{V3D(0, 0, 1)};
  StatesGroup state_;
  StatesGroup prior_;
  VIOManager manager_;
};

TEST_F(VioSamplingTest, UnsafeCoarsePatchIgnoresAdjacentStorage) {
  for (std::uint8_t padding : {0, 255}) {
    SCOPED_TRACE(static_cast<int>(padding));
    const StatesGroup result = Update(96, 3, padding);
    EXPECT_TRUE(result.rot_end.isIdentity(0));
    EXPECT_TRUE(result.pos_end.isZero(0));
    EXPECT_TRUE(result.vel_end.isZero(0));
    EXPECT_DOUBLE_EQ(result.inv_expo_time, 1);
    EXPECT_TRUE(result.cov.isApprox(StatesGroup().cov, 0));
  }
}

TEST_F(VioSamplingTest, ProcessesFineLevelForSamePoint) {
  const StatesGroup result = Update(96, 0, 255);
  EXPECT_GT(result.inv_expo_time, 0.5);
  EXPECT_LT(result.inv_expo_time, 1);
}

TEST_F(VioSamplingTest, ProcessesSafeCoarsePatch) {
  const StatesGroup result = Update(256, 3, 255);
  EXPECT_GT(result.inv_expo_time, 0.5);
  EXPECT_LT(result.inv_expo_time, 1);
}

TEST_F(VioSamplingTest, AllInvalidPatchesPreserveCovariance) {
  manager_.patch_pyrimid_level = 1;
  manager_.G.setIdentity();  // A previous frame's information must not be reused.
  const StatesGroup result = Update(-144, 0, 255, true);
  EXPECT_TRUE(result.rot_end.isIdentity(0));
  EXPECT_TRUE(result.pos_end.isZero(0));
  EXPECT_DOUBLE_EQ(result.inv_expo_time, 1);
  EXPECT_TRUE(result.cov.isApprox(StatesGroup().cov, 0));
}

TEST(VioSamplingConfigTest, RejectsNonPositiveAndOddPatchSizes) {
  vk::PinholeCamera camera(512, 512, 1, 200, 200, 256, 256);
  for (int patch_size : {-2, 0, 1, 7, 9}) {
    SCOPED_TRACE(patch_size);
    VIOManager manager;
    manager.cam = &camera;
    manager.grid_size = 32;
    manager.raycast_en = false;
    manager.patch_size = patch_size;
    manager.patch_pyrimid_level = 4;
    manager.Rli = M3D::Identity();
    manager.Rcl = M3D::Identity();
    manager.Pli = V3D::Zero();
    manager.Pcl = V3D::Zero();
    EXPECT_THROW(manager.initializeVIO(), std::invalid_argument);
    EXPECT_EQ(manager.visual_submap, nullptr);
  }
}

}  // namespace
}  // namespace livo_atlas
