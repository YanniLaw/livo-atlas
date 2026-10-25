#include "livo_atlas/ros1/node_options.h"

#include <XmlRpcValue.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "livo_atlas/core/livo_system.h"

namespace livo_atlas::ros1 {
namespace {

class Ros1ParametersTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const auto id = std::to_string(ros::WallTime::now().toNSec());
    node_ = std::make_unique<ros::NodeHandle>("/convergence_parameters_" + id);
    directory_ = std::filesystem::temp_directory_path() /
                 ("livo_atlas_convergence/parameters-" + id);
    node_->setParam("output/directory", directory_.string());
    for (const auto* name : {"extrinsic_R", "Rcl"}) {
      node_->setParam(std::string("extrin_calib/") + name,
                      std::vector<int>{1, 0, 0, 0, 1, 0, 0, 0, 1});
    }
    for (const auto* name : {"extrinsic_T", "Pcl"}) {
      node_->setParam(std::string("extrin_calib/") + name,
                      std::vector<double>{0, 0, 0});
    }
    for (const auto* name : {"cam_model", "cam_width", "cam_height", "cam_fx",
                             "cam_fy", "cam_cx", "cam_cy", "scale"}) {
      XmlRpc::XmlRpcValue value;
      if (camera_.getParam(name, value)) saved_[name] = value;
    }
    camera_.setParam("cam_model", "Pinhole");
    camera_.setParam("cam_width", 512);
    camera_.setParam("cam_height", 512);
    camera_.setParam("cam_fx", 200);
    camera_.setParam("cam_fy", 210.0);
    camera_.setParam("cam_cx", 256);
    camera_.setParam("cam_cy", 256);
    camera_.setParam("scale", 1.0);
  }

  void TearDown() override {
    node_->deleteParam("");
    for (const auto* name : {"cam_model", "cam_width", "cam_height", "cam_fx",
                             "cam_fy", "cam_cx", "cam_cy", "scale"}) {
      const auto it = saved_.find(name);
      if (it == saved_.end())
        camera_.deleteParam(name);
      else
        camera_.setParam(name, it->second);
    }
    std::filesystem::remove_all(directory_);
  }

  ros::NodeHandle camera_{"~"};
  std::unique_ptr<ros::NodeHandle> node_;
  std::filesystem::path directory_;
  std::map<std::string, XmlRpc::XmlRpcValue> saved_;
};

TEST_F(Ros1ParametersTest, ReadsPrivateCameraNumericArraysAndSharedDefaults) {
  const auto config = ReadNodeOptions(*node_);
  const auto& options = config.livo.system;
  EXPECT_DOUBLE_EQ(options.frontend.calibration.camera.fx, 200);
  EXPECT_DOUBLE_EQ(options.frontend.calibration.camera.fy, 210);
  EXPECT_EQ(options.frontend.calibration.camera.width, 512);
  EXPECT_EQ(options.preprocess.point_filter, 3);
  EXPECT_EQ(options.frontend.voxel_map.layer_init_num_,
            (std::vector<int>{5, 5, 5, 5, 5}));
  EXPECT_EQ(options.exporter.mode, FrontendMode::kLidarInertialVisual);
  EXPECT_EQ(options.exporter.directory, directory_);
  EXPECT_DOUBLE_EQ(options.imu_time_range.minimum, 0);
  EXPECT_DOUBLE_EQ(options.imu_time_range.maximum, 4294967296.0);
  EXPECT_FALSE(std::filesystem::exists(directory_));
}

TEST_F(Ros1ParametersTest, AppliesOverridesToOneOrdinaryConfiguration) {
  node_->setParam("common/img_en", 0);
  node_->setParam("imu/imu_en", true);
  node_->setParam("imu/acc_cov", .25);
  node_->setParam("time_offset/imu_time_offset", .03);
  node_->setParam("time_offset/img_time_offset", -.01);
  node_->setParam("lio/max_layer", 2);
  node_->setParam("lio/layer_init_num", std::vector<int>{3, 6, 9});
  node_->setParam("preprocess/point_filter_num", 1);
  node_->setParam("publish/pub_scan_num", 3);
  camera_.setParam("scale", .5);
  const auto config = ReadNodeOptions(*node_);
  const auto& options = config.livo.system;
  EXPECT_EQ(options.exporter.mode, FrontendMode::kLidarInertial);
  EXPECT_DOUBLE_EQ(options.frontend.imu.accelerometer_covariance, .25);
  EXPECT_DOUBLE_EQ(options.imu_time_offset, .03);
  EXPECT_DOUBLE_EQ(config.livo.image_time_offset, -.01);
  EXPECT_EQ(options.frontend.voxel_map.layer_init_num_,
            (std::vector<int>{3, 6, 9}));
  EXPECT_EQ(options.processor.scan_interval, 3);
  EXPECT_EQ(options.preprocess.point_filter, 1);
  EXPECT_DOUBLE_EQ(options.frontend.calibration.camera.scale, .5);
  EXPECT_DOUBLE_EQ(options.frontend.calibration.camera.fx, 200);
  EXPECT_FALSE(std::filesystem::exists(directory_));
}

TEST_F(Ros1ParametersTest, InvalidConfigurationCannotCreateOrTruncateOutput) {
  for (bool existing : {false, true}) {
    node_->setParam("output/directory",
                    (directory_ / (existing ? "old" : "new")).string());
    const auto sentinel = directory_ / "old/mat_pre.txt";
    if (existing) {
      std::filesystem::create_directories(sentinel.parent_path());
      std::ofstream(sentinel) << "keep existing output";
    }
    for (const auto& key :
         {"vio/img_point_cov", "imu/acc_cov", "preprocess/point_filter_num"}) {
      node_->setParam(key, -1);
      EXPECT_THROW(
          {
            const auto config = ReadNodeOptions(*node_);
            LivoSystem session(config.livo.system);
          },
          std::exception);
      node_->deleteParam(key);
      if (existing) {
        std::ifstream input(sentinel);
        EXPECT_EQ(std::string(std::istreambuf_iterator<char>(input), {}),
                  "keep existing output");
      } else {
        EXPECT_FALSE(std::filesystem::exists(directory_ / "new"));
      }
    }
    node_->setParam("common/imu_topic", "bad topic");
    EXPECT_THROW(ReadNodeOptions(*node_), std::exception);
    node_->deleteParam("common/imu_topic");
  }
}

TEST_F(Ros1ParametersTest, RejectsWrongScalarAndArrayTypesBeforeOpeningFiles) {
  node_->setParam("imu/imu_int_frame", "three");
  EXPECT_THROW(ReadNodeOptions(*node_), std::exception);
  node_->deleteParam("imu/imu_int_frame");
  camera_.setParam("cam_fx", true);
  EXPECT_THROW(ReadNodeOptions(*node_), std::exception);
  camera_.setParam("cam_fx", 200);
  XmlRpc::XmlRpcValue rotation;
  rotation.setSize(9);
  for (int i = 0; i < 9; ++i) rotation[i] = (i % 4 == 0 ? 1 : 0);
  rotation[1] = false;
  node_->setParam("extrin_calib/extrinsic_R", rotation);
  EXPECT_THROW(ReadNodeOptions(*node_), std::exception);
  EXPECT_FALSE(std::filesystem::exists(directory_));
}

}  // namespace
}  // namespace livo_atlas::ros1
