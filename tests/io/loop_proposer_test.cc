#include "io/loop_proposer.h"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <opencv2/core/persistence.hpp>

#include "backend/loop_fixtures.h"
#include "livo_atlas/io/keyframe_store.h"
#include "livo_atlas/io/online_submap_store.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;

std::string Bytes(const fs::path& path) {
  std::ifstream file(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(file), {}};
}

std::map<std::string, std::string> DatasetBytes(const fs::path& root) {
  std::map<std::string, std::string> result;
  for (const auto& entry : fs::recursive_directory_iterator(root)) {
    if (entry.is_regular_file())
      result.emplace(fs::relative(entry.path(), root).string(),
                     Bytes(entry.path()));
  }
  return result;
}

PointCloud::ConstPtr SceneCloud(bool dense) {
  PointCloud::Ptr cloud(new PointCloud);
  if (!dense) {
    Point p;
    p.x = p.y = p.z = 0.1;
    cloud->push_back(p);
    return cloud;
  }
  const auto scene = loop_test::Scene();
  for (const auto& voxel : scene.data->voxels) {
    const auto n = *voxel.normal;
    const Eigen::Vector3d u =
        n.x() ? Eigen::Vector3d::UnitY() : Eigen::Vector3d::UnitX();
    const Eigen::Vector3d v = n.cross(u);
    // The patch must span enough of its 0.2 m voxel to qualify as a plane.
    for (double a : {-0.04, 0.0, 0.04}) {
      for (double b : {-0.04, 0.0, 0.04}) {
        const Eigen::Vector3d xyz =
            voxel.centroid + Eigen::Vector3d::Constant(0.1) + a * u + b * v;
        Point p;
        p.x = xyz.x();
        p.y = xyz.y();
        p.z = xyz.z();
        p.intensity = 8;
        cloud->push_back(p);
      }
    }
  }
  return cloud;
}

class LoopProposerTest : public testing::Test {
 protected:
  void SetUp() override {
    root_ = fs::temp_directory_path() /
            ("loop-proposer-" +
             std::to_string(
                 std::chrono::steady_clock::now().time_since_epoch().count()));
    dataset_ = root_ / "keyframes";
    fs::create_directories(root_);
  }

  void TearDown() override { fs::remove_all(root_); }

  void Dataset(bool dense = true) {
    Calibration calibration;
    calibration.camera = {"Pinhole", 12, 8, .5, 4, 5, 6, 4, {0, 0, 0, 0}};
    KeyframeStore frames(dataset_, calibration);
    OnlineSubmapOptions options;
    options.support_keyframes = 0;
    OnlineSubmapStore store(dataset_ / "online_submaps", frames, 1, options);
    OnlineSubmapBuilder builder(1, options);
    const auto cloud = SceneCloud(dense);
    for (std::uint64_t id = 0; id < 4; ++id) {
      Keyframe frame;
      frame.id = id;
      frame.timestamp = frame.cloud_timestamp = 10 + id;
      frame.odom_from_body.translation.x() = 0.02 * id;
      frame.cloud = cloud;
      frames.Append(frame);
      for (const auto& group :
           builder.AddKeyframe(id, frame.timestamp, frame.odom_from_body))
        store.AppendRevision(group);
    }
    for (const auto& group : builder.Finish()) store.AppendRevision(group);
  }

  LoopAdmissionOptions Nearby() const {
    LoopAdmissionOptions options;
    options.search.min_submap_gap = 1;
    options.search.min_keyframe_gap = 2;
    options.search.min_time_gap_s = 0.5;
    return options;
  }

  cv::FileStorage Report(const std::string& path) const {
    return cv::FileStorage((root_ / path).string(), cv::FileStorage::READ);
  }

  fs::path root_, dataset_;
};

TEST_F(LoopProposerTest, ZeroCandidatesAndGeometricRejectionsAreComplete) {
  Dataset(false);
  const auto before = DatasetBytes(dataset_);
  ProposeLoopDataset(dataset_, root_ / "zero", {});
  auto zero = Report("zero/summary.yml");
  EXPECT_EQ(static_cast<int>(zero["complete"]), 1);
  EXPECT_EQ(zero["query_count"].string(), "4");
  EXPECT_EQ(zero["proposal_count"].string(), "0");
  ProposeLoopDataset(dataset_, root_ / "rejected", Nearby());
  auto rejected = Report("rejected/queries/2.yml");
  EXPECT_EQ(rejected["status"].string(), "geometry_rejected");
  EXPECT_TRUE(rejected["trial"].empty());
  EXPECT_EQ(DatasetBytes(dataset_), before);
}

TEST_F(LoopProposerTest, AcceptsInMemoryAndUsesTheWholeClosedFrontier) {
  Dataset();
  const auto before = DatasetBytes(dataset_);
  ProposeLoopDataset(dataset_, root_ / "result", Nearby());
  auto summary = Report("result/summary.yml");
  EXPECT_EQ(static_cast<int>(summary["complete"]), 1);
  EXPECT_EQ(summary["query_count"].string(), "4");
  EXPECT_EQ(summary["proposal_count"].string(), "1");
  EXPECT_EQ(summary["pose_version"].string(), "1");
  auto accepted = Report("result/queries/2.yml");
  ASSERT_EQ(accepted["status"].string(), "proposed")
      << Bytes(root_ / "result/queries/2.yml");
  ASSERT_EQ(accepted["candidates"].size(), 1u);
  EXPECT_EQ(accepted["candidates"][0]["target_id"].string(), "0");
  auto proposal = Report("result/proposals/1.yml");
  EXPECT_EQ(proposal["update"]["base_version"].string(), "0");
  EXPECT_EQ(proposal["update"]["optimized_frame_count"].string(), "4");
  EXPECT_EQ(proposal["update"]["poses"].size(), 4u);
  auto later = Report("result/queries/3.yml");
  EXPECT_EQ(later["base_version"].string(), "1");
  EXPECT_EQ(later["status"].string(), "correlated");
  EXPECT_EQ(DatasetBytes(dataset_), before);
}

TEST_F(LoopProposerTest, DoesNotCropAnOversizeGraphOrReadClouds) {
  Dataset();
  fs::remove(dataset_ / "frames/0/cloud.pcd");
  auto options = Nearby();
  options.max_keyframes = 3;
  ProposeLoopDataset(dataset_, root_ / "budget", options);
  auto query = Report("budget/queries/0.yml");
  EXPECT_EQ(query["status"].string(), "resource_limit");
  EXPECT_EQ(query["frame_count"].string(), "4");
  auto summary = Report("budget/summary.yml");
  EXPECT_EQ(static_cast<int>(summary["complete"]), 1);
}

TEST_F(LoopProposerTest, PreservesEarlierRecordsWhenALaterAssetFails) {
  Dataset();
  std::ofstream(dataset_ / "frames/3/cloud.pcd") << "invalid pcd";
  const auto before = DatasetBytes(dataset_);
  EXPECT_THROW(ProposeLoopDataset(dataset_, root_ / "partial", Nearby()),
               std::runtime_error);
  auto summary = Report("partial/summary.yml");
  EXPECT_EQ(static_cast<int>(summary["complete"]), 0);
  EXPECT_EQ(summary["query_count"].string(), "3");
  EXPECT_EQ(summary["proposal_count"].string(), "1");
  EXPECT_TRUE(fs::exists(root_ / "partial/queries/2.yml"));
  EXPECT_TRUE(fs::exists(root_ / "partial/proposals/1.yml"));
  auto failure = Report("partial/failure.yml");
  EXPECT_FALSE(failure["error"].string().empty());
  EXPECT_EQ(DatasetBytes(dataset_), before);
}

TEST_F(LoopProposerTest, RejectsCorruptMetadataAndExclusiveOutputViolations) {
  Dataset();
  fs::create_directory(root_ / "existing");
  std::ofstream(root_ / "existing/keep") << "keep";
  EXPECT_THROW(ProposeLoopDataset(dataset_, root_ / "existing", Nearby()),
               std::runtime_error);
  EXPECT_EQ(Bytes(root_ / "existing/keep"), "keep");
  EXPECT_FALSE(fs::exists(root_ / "existing/failure.yml"));
  EXPECT_THROW(ProposeLoopDataset(dataset_, dataset_ / "result", Nearby()),
               std::invalid_argument);
  EXPECT_FALSE(fs::exists(dataset_ / "result"));
  std::ofstream(dataset_ / "frames/3/frame.yml") << "corrupt metadata";
  EXPECT_THROW(ProposeLoopDataset(dataset_, root_ / "corrupt", Nearby()),
               std::exception);
  auto summary = Report("corrupt/summary.yml");
  EXPECT_EQ(static_cast<int>(summary["complete"]), 0);
  EXPECT_TRUE(fs::exists(root_ / "corrupt/failure.yml"));
}

TEST_F(LoopProposerTest, ParsesFourStrictOptionGroups) {
  const auto config = root_ / "options.yml";
  std::ofstream(config)
      << "%YAML:1.0\n---\nsearch:\n  max_candidates: 2\n"
         "registration:\n  max_iterations: 7\ngraph:\n  loop_huber_delta: 1.5\n"
         "admission:\n  max_keyframes: 32\n  loop_translation_sigma_m: 0.3\n";
  const auto options = ReadLoopAdmissionOptions(config);
  EXPECT_EQ(options.search.max_candidates, 2u);
  EXPECT_EQ(options.registration.max_iterations, 7);
  EXPECT_DOUBLE_EQ(options.graph.loop_huber_delta, 1.5);
  EXPECT_EQ(options.max_keyframes, 32u);
  EXPECT_DOUBLE_EQ(options.loop_translation_sigma_m, 0.3);
  for (const auto* bad :
       {"unknown: 2\n", "graph:\n  typo: 1\n",
        "graph:\n  loop_huber_delta: 0\n", "admission:\n  max_keyframes: 1.5\n",
        "admission:\n  max_keyframes: 9007199254740992.0\n",
        "admission:\n  max_factor_residual: text\n",
        "admission:\n  max_factor_residual: .Nan\n"}) {
    std::ofstream(config) << "%YAML:1.0\n---\n" << bad;
    EXPECT_THROW(ReadLoopAdmissionOptions(config), std::invalid_argument)
        << bad;
  }
}

TEST_F(LoopProposerTest, RejectsDuplicateFieldsAndGroupsBeforeKeyedReads) {
  const auto config = root_ / "duplicate.yml";
  for (const auto* body :
       {"admission:\n  max_factor_residual: 3\n  max_factor_residual: 0\n",
        "graph:\n  loop_huber_delta: 1.345\n  loop_huber_delta: 0\n",
        "search:\n  max_candidates: 5\n  max_candidates: 0\n",
        "registration:\n  max_iterations: 50\n  max_iterations: 0\n",
        "admission:\n  max_factor_residual: 3\nadmission:\n  "
        "max_factor_residual: 0\n",
        "graph:\n  loop_huber_delta: 1.345\ngraph:\n  loop_huber_delta: 0\n"}) {
    std::ofstream(config) << "%YAML:1.0\n---\n" << body;
    EXPECT_THROW(ReadLoopAdmissionOptions(config), std::invalid_argument)
        << body;
  }
}

}  // namespace
}  // namespace livo_atlas
