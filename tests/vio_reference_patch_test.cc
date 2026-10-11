#include "vio.h"

#include <unordered_map>

#include <gtest/gtest.h>

#include "feature.h"

namespace livo_atlas {
namespace {

class VioReferencePatchTest : public ::testing::Test {
 protected:
  void SetUp() override {
    manager_.visual_submap = new SubSparseMap;
    manager_.total_points = 1;
    manager_.patch_size_total = 4;
    manager_.update_flag = {1};
    manager_.visual_submap->voxel_points.push_back(&point_);
    point_.normal_ = V3D(0, 0, 1);
    point_.is_normal_initialized_ = true;
  }

  void AddObservations(bool cached) {
    for (int index = 0; index < 6; ++index) {
      const float mean = 10 * (index + 1);
      auto* feature = new Feature(
          &point_, new float[4]{mean, mean + 2, mean, mean - 2}, V2D::Zero(),
          V3D(0, 0, 1), SE3(), 0);
      feature->id_ = index;
      feature->mean_ = cached ? mean : 0;
      point_.obs_.push_back(feature);
    }
  }

  void ExpectCenteredScores() {
    // All patches share centered values (0, 2, 0, -2): NCC = 1.
    // Identity poses and the frontal normal add a view score of 1.
    ASSERT_TRUE(point_.has_ref_patch_);
    for (const Feature* feature : point_.obs_) {
      EXPECT_FLOAT_EQ(feature->score_, 2) << "feature " << feature->id_;
      EXPECT_FLOAT_EQ(feature->mean_, 10 * (feature->id_ + 1));
    }
  }

  VisualPoint point_{V3D(0, 0, 1)};
  VIOManager manager_;
  const std::unordered_map<VOXEL_LOCATION, VoxelOctoTree*> plane_map_;
};

TEST_F(VioReferencePatchTest, UsesCachedPatchMeans) {
  AddObservations(true);
  manager_.updateReferencePatch(plane_map_);
  ExpectCenteredScores();
}

TEST_F(VioReferencePatchTest, ScoresStayEqualAfterMeansAreCached) {
  AddObservations(false);
  manager_.updateReferencePatch(plane_map_);
  ExpectCenteredScores();
  manager_.updateReferencePatch(plane_map_);
  ExpectCenteredScores();
}

}  // namespace
}  // namespace livo_atlas
