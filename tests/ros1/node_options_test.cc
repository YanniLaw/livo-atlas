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
  EXPECT_FALSE(options.keyframes.enabled);
  EXPECT_FALSE(options.submaps.enabled);
  EXPECT_EQ(options.submaps.max_keyframes, 20);
  EXPECT_EQ(config.livo.image_queue_size, 1);
  EXPECT_DOUBLE_EQ(options.imu_time_range.minimum, 0);
  EXPECT_DOUBLE_EQ(options.imu_time_range.maximum, 4294967296.0);
  EXPECT_FALSE(std::filesystem::exists(directory_));
}

TEST_F(Ros1ParametersTest, StartupBufferParameterCanRestoreLegacyBehavior) {
  EXPECT_TRUE(ReadNodeOptions(*node_).livo.system.buffer_before_first_lidar);
  node_->setParam("common/buffer_before_first_lidar", false);
  EXPECT_FALSE(ReadNodeOptions(*node_).livo.system.buffer_before_first_lidar);
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
  node_->setParam("publish/image_queue_size", 32);
  node_->setParam("keyframes/enabled", true);
  node_->setParam("keyframes/translation_threshold", .7);
  node_->setParam("keyframes/rotation_threshold_deg", 15.);
  node_->setParam("keyframes/time_threshold", 3.);
  node_->setParam("submaps/enabled", true);
  node_->setParam("submaps/max_keyframes", 3);
  camera_.setParam("scale", .5);
  const auto config = ReadNodeOptions(*node_);
  const auto& options = config.livo.system;
  EXPECT_EQ(options.exporter.mode, FrontendMode::kLidarInertial);
  EXPECT_DOUBLE_EQ(options.frontend.imu.accelerometer_covariance, .25);
  EXPECT_DOUBLE_EQ(options.imu_time_offset, .03);
  EXPECT_DOUBLE_EQ(config.livo.image_time_offset, -.01);
  EXPECT_EQ(options.frontend.voxel_map.layer_init_num_,
            (std::vector<int>{3, 6, 9}));
  EXPECT_EQ(options.colorizer.scan_interval, 3);
  EXPECT_EQ(config.livo.image_queue_size, 32);
  EXPECT_TRUE(options.keyframes.enabled);
  EXPECT_DOUBLE_EQ(options.keyframes.translation_threshold, .7);
  EXPECT_DOUBLE_EQ(options.keyframes.rotation_threshold_deg, 15);
  EXPECT_DOUBLE_EQ(options.keyframes.time_threshold, 3);
  EXPECT_TRUE(options.submaps.enabled);
  EXPECT_EQ(options.submaps.max_keyframes, 3);
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
         {"vio/img_point_cov", "imu/acc_cov", "preprocess/point_filter_num",
          "keyframes/translation_threshold", "keyframes/rotation_threshold_deg",
          "keyframes/time_threshold", "submaps/max_keyframes",
          "publish/image_queue_size"}) {
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
    node_->setParam("submaps/enabled", true);
    EXPECT_THROW(ReadNodeOptions(*node_), std::exception);
    node_->deleteParam("submaps/enabled");
    node_->setParam("common/imu_topic", "bad topic");
    EXPECT_THROW(ReadNodeOptions(*node_), std::exception);
    node_->deleteParam("common/imu_topic");
    node_->setParam("publish/image_queue_size", 0);
    EXPECT_THROW(ReadNodeOptions(*node_), std::exception);
    node_->deleteParam("publish/image_queue_size");
  }
}

TEST_F(Ros1ParametersTest, RejectsWrongScalarAndArrayTypesBeforeOpeningFiles) {
  node_->setParam("imu/imu_int_frame", "three");
  EXPECT_THROW(ReadNodeOptions(*node_), std::exception);
  node_->deleteParam("imu/imu_int_frame");
  node_->setParam("publish/image_queue_size", "many");
  EXPECT_THROW(ReadNodeOptions(*node_), std::exception);
  node_->deleteParam("publish/image_queue_size");
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

TEST_F(Ros1ParametersTest, ReadsOnlineSubmapModeAndValidatesBounds) {
  node_->setParam("keyframes/enabled", true);
  node_->setParam("submaps/enabled", true);
  node_->setParam("submaps/mode", "online");
  node_->setParam("submaps/online/translation_threshold", 3.0);
  const auto config = ReadNodeOptions(*node_);
  EXPECT_EQ(config.livo.system.submap_mode, SubmapMode::kOnline);
  EXPECT_DOUBLE_EQ(config.livo.system.online_submaps.translation_threshold,
                   3.0);
  node_->setParam("submaps/online/support_keyframes", -1);
  EXPECT_THROW(ReadNodeOptions(*node_), std::exception);
  node_->setParam("submaps/mode", "keyframe_groups");
  EXPECT_NO_THROW(ReadNodeOptions(*node_));
  node_->setParam("submaps/mode", "unknown");
  EXPECT_THROW(ReadNodeOptions(*node_), std::exception);
}

TEST_F(Ros1ParametersTest, ReadsGeometryOptionsAndRejectsInvalidBounds) {
  node_->setParam("keyframes/enabled", true);
  node_->setParam("submaps/enabled", true);
  node_->setParam("submaps/mode", "online");
  node_->setParam("submaps/geometry/enabled", true);
  node_->setParam("submaps/geometry/voxel_size", 0.3);
  node_->setParam("submaps/geometry/min_points_per_voxel", 8);
  node_->setParam("submaps/geometry/max_plane_rms", 0.03);
  node_->setParam("submaps/geometry/max_plane_ratio", 0.2);
  node_->setParam("submaps/geometry/max_voxels_per_submap", 50000);
  node_->setParam("submaps/geometry/max_points_per_keyframe", 200000);
  node_->setParam("submaps/geometry/max_resident_submaps", 4);
  const auto options = ReadNodeOptions(*node_).livo.system.geometry;
  EXPECT_TRUE(options.enabled);
  EXPECT_DOUBLE_EQ(options.voxel_size, 0.3);
  EXPECT_EQ(options.min_points_per_voxel, 8);
  EXPECT_DOUBLE_EQ(options.max_plane_rms, 0.03);
  EXPECT_DOUBLE_EQ(options.max_plane_ratio, 0.2);
  EXPECT_EQ(options.max_voxels_per_submap, 50000);
  EXPECT_EQ(options.max_points_per_keyframe, 200000);
  EXPECT_EQ(options.max_resident_submaps, 4);
  node_->setParam("submaps/geometry/max_resident_submaps", 1);
  EXPECT_THROW(ReadNodeOptions(*node_), std::exception);
  node_->setParam("submaps/geometry/enabled", false);
  EXPECT_NO_THROW(ReadNodeOptions(*node_));
}

TEST_F(Ros1ParametersTest, ReadsLoopOptionsAndChecksRequiredOnlineStorage) {
  EXPECT_FALSE(ReadNodeOptions(*node_).livo.system.loop_closure);
  node_->setParam("loop_closure/options_file", "/nonexistent/ignored-loop.yml");
  EXPECT_NO_THROW(ReadNodeOptions(*node_));
  node_->deleteParam("loop_closure/options_file");
  node_->setParam("loop_closure/enabled", true);
  EXPECT_THROW(ReadNodeOptions(*node_), std::invalid_argument);
  node_->setParam("keyframes/enabled", true);
  node_->setParam("submaps/enabled", true);
  node_->setParam("submaps/mode", "online");
  node_->setParam("submaps/max_keyframes", 64);
  node_->setParam("loop_closure/search/search_radius_m", 12.5);
  const auto config = ReadNodeOptions(*node_).livo;
  ASSERT_TRUE(config.system.loop_closure);
  EXPECT_TRUE(config.system.loop_closure->pose_graph_enabled);
  EXPECT_DOUBLE_EQ(config.system.loop_closure->search.search_radius_m, 12.5);
  EXPECT_FALSE(config.system.geometry.enabled);
  node_->setParam("submaps/max_keyframes", 65);
  EXPECT_THROW(ReadNodeOptions(*node_), std::invalid_argument);
  EXPECT_FALSE(std::filesystem::exists(directory_));
}

TEST_F(Ros1ParametersTest, GaussianPublicationUsesStrictSharedSchema) {
  EXPECT_FALSE(ReadNodeOptions(*node_).livo.gaussian_output.enabled);
  node_->setParam("gaussian/publish_preview", 1);
  EXPECT_THROW(ReadNodeOptions(*node_), std::exception);
  node_->setParam("gaussian/publish_preview", true);
  EXPECT_THROW(ReadNodeOptions(*node_), std::invalid_argument);
  node_->setParam("gaussian/enabled", true);
  node_->setParam("gaussian/socket_path", "/tmp/gaussian-publication.sock");
  node_->setParam("gaussian/output_directory", "/tmp/gaussian-publication");
  node_->setParam("gaussian/persist_submaps", true);
  node_->setParam("gaussian/joint_render_enabled", true);
  node_->setParam("gaussian/joint_render_interval_ms", 321);
  node_->setParam("keyframes/enabled", true);
  node_->setParam("submaps/enabled", true);
  node_->setParam("submaps/mode", "online");
  node_->setParam("gaussian/preview_frame_id", 1);
  EXPECT_THROW(ReadNodeOptions(*node_), std::exception);
  node_->setParam("gaussian/preview_frame_id", "atlas/gaussian_optical");
  const auto config = ReadNodeOptions(*node_);
  EXPECT_TRUE(config.livo.gaussian_output.enabled);
  EXPECT_EQ(config.livo.gaussian_output.optical_frame,
            "atlas/gaussian_optical");
  EXPECT_EQ(config.livo.gaussian_output.minimum_interval.count(), 321);
  EXPECT_FALSE(std::filesystem::exists(directory_));
}

TEST_F(Ros1ParametersTest, GaussianJointRenderUsesStrictSharedSchema) {
  EXPECT_FALSE(ReadNodeOptions(*node_).livo.gaussian.joint_render_enabled);
  node_->setParam("gaussian/joint_render_enabled", 1);
  EXPECT_THROW(ReadNodeOptions(*node_), std::exception);
  node_->setParam("gaussian/joint_render_enabled", true);
  EXPECT_THROW(ReadNodeOptions(*node_), std::invalid_argument);
  node_->setParam("gaussian/enabled", true);
  node_->setParam("gaussian/socket_path", "/tmp/gaussian-test.sock");
  node_->setParam("gaussian/output_directory", "/tmp/gaussian-test-output");
  node_->setParam("keyframes/enabled", true);
  node_->setParam("submaps/enabled", true);
  node_->setParam("submaps/mode", "online");
  EXPECT_THROW(ReadNodeOptions(*node_), std::invalid_argument);
  node_->setParam("gaussian/persist_submaps", true);
  EXPECT_EQ(ReadNodeOptions(*node_).livo.gaussian.joint_render_interval.count(),
            1000);
  for (const auto invalid : {-1, 0, 99, 60001}) {
    node_->setParam("gaussian/joint_render_interval_ms", invalid);
    EXPECT_THROW(ReadNodeOptions(*node_), std::invalid_argument);
  }
  node_->setParam("gaussian/joint_render_interval_ms", 1.5);
  EXPECT_THROW(ReadNodeOptions(*node_), std::exception);
  node_->setParam("gaussian/joint_render_interval_ms", "1000");
  EXPECT_THROW(ReadNodeOptions(*node_), std::exception);
  for (const auto valid : {100, 1000, 60000}) {
    node_->setParam("gaussian/joint_render_interval_ms", valid);
    const auto options = ReadNodeOptions(*node_).livo.gaussian;
    EXPECT_TRUE(options.joint_render_enabled);
    EXPECT_EQ(options.joint_render_interval.count(), valid);
  }
  EXPECT_FALSE(std::filesystem::exists(directory_));
}

TEST_F(Ros1ParametersTest, GaussianSpatialPolicyUsesStrictSharedSchema) {
  EXPECT_EQ(ReadNodeOptions(*node_).livo.gaussian.joint_render_policy,
            GaussianRenderPolicy::kTemporal);
  node_->setParam("gaussian/joint_render_policy", 1);
  EXPECT_THROW(ReadNodeOptions(*node_), std::exception);
  node_->setParam("gaussian/joint_render_policy", "unknown");
  EXPECT_THROW(ReadNodeOptions(*node_), std::invalid_argument);
  node_->setParam("gaussian/joint_render_policy", "spatial");
  EXPECT_THROW(ReadNodeOptions(*node_), std::invalid_argument);
  node_->setParam("gaussian/enabled", true);
  node_->setParam("gaussian/socket_path", "/tmp/spatial-test.sock");
  node_->setParam("gaussian/output_directory", "/tmp/spatial-output");
  node_->setParam("gaussian/persist_submaps", true);
  node_->setParam("keyframes/enabled", true);
  node_->setParam("submaps/enabled", true);
  node_->setParam("submaps/mode", "online");
  EXPECT_THROW(ReadNodeOptions(*node_), std::invalid_argument);
  node_->setParam("gaussian/joint_render_enabled", true);
  EXPECT_EQ(ReadNodeOptions(*node_).livo.gaussian.joint_render_policy,
            GaussianRenderPolicy::kSpatial);
  EXPECT_FALSE(std::filesystem::exists(directory_));
}

TEST_F(Ros1ParametersTest, GaussianFinalizationUsesStrictSharedSchema) {
  EXPECT_FALSE(ReadNodeOptions(*node_).livo.gaussian.finalize_on_shutdown);
  node_->setParam("gaussian/finalize_on_shutdown", 1);
  EXPECT_THROW(ReadNodeOptions(*node_), std::exception);
  node_->setParam("gaussian/finalize_on_shutdown", true);
  EXPECT_THROW(ReadNodeOptions(*node_), std::invalid_argument);
  node_->setParam("gaussian/enabled", true);
  node_->setParam("gaussian/socket_path", "/tmp/gaussian-test.sock");
  node_->setParam("gaussian/output_directory", "/tmp/gaussian-test-output");
  node_->setParam("keyframes/enabled", true);
  node_->setParam("submaps/enabled", true);
  node_->setParam("submaps/mode", "online");
  EXPECT_THROW(ReadNodeOptions(*node_), std::invalid_argument);
  node_->setParam("gaussian/persist_submaps", true);
  EXPECT_EQ(ReadNodeOptions(*node_).livo.gaussian.finalize_timeout.count(),
            60000);
  for (const auto invalid : {0, -1, 600001}) {
    node_->setParam("gaussian/finalize_timeout_ms", invalid);
    EXPECT_THROW(ReadNodeOptions(*node_), std::invalid_argument);
  }
  node_->setParam("gaussian/finalize_timeout_ms", 1.5);
  EXPECT_THROW(ReadNodeOptions(*node_), std::exception);
  node_->setParam("gaussian/finalize_timeout_ms", "60000");
  EXPECT_THROW(ReadNodeOptions(*node_), std::exception);
  for (const auto valid : {1, 60000, 600000}) {
    node_->setParam("gaussian/finalize_timeout_ms", valid);
    const auto options = ReadNodeOptions(*node_).livo.gaussian;
    EXPECT_TRUE(options.finalize_on_shutdown);
    EXPECT_EQ(options.finalize_timeout.count(), valid);
  }
  EXPECT_FALSE(std::filesystem::exists(directory_));
}

TEST_F(Ros1ParametersTest, GaussianPersistenceAndTimeoutUseStrictSharedSchema) {
  EXPECT_FALSE(ReadNodeOptions(*node_).livo.gaussian.persist_submaps);
  node_->setParam("gaussian/persist_submaps", 1);
  EXPECT_THROW(ReadNodeOptions(*node_), std::exception);
  node_->setParam("gaussian/persist_submaps", true);
  EXPECT_THROW(ReadNodeOptions(*node_), std::invalid_argument);
  node_->setParam("gaussian/enabled", true);
  node_->setParam("gaussian/socket_path", "/tmp/gaussian-test.sock");
  node_->setParam("gaussian/output_directory", "/tmp/gaussian-test-output");
  node_->setParam("keyframes/enabled", true);
  node_->setParam("submaps/enabled", true);
  node_->setParam("submaps/mode", "online");
  node_->setParam("gaussian/shutdown_timeout_ms", 1.5);
  EXPECT_THROW(ReadNodeOptions(*node_), std::exception);
  node_->setParam("gaussian/shutdown_timeout_ms", 0);
  EXPECT_THROW(ReadNodeOptions(*node_), std::invalid_argument);
  node_->setParam("gaussian/shutdown_timeout_ms", 30000);
  const auto config = ReadNodeOptions(*node_).livo.gaussian;
  EXPECT_TRUE(config.persist_submaps);
  EXPECT_EQ(config.shutdown_timeout.count(), 30000);
  EXPECT_FALSE(std::filesystem::exists(directory_));
}

TEST_F(Ros1ParametersTest, GaussianSchemaValidatesBeforeOpeningOutput) {
  EXPECT_FALSE(ReadNodeOptions(*node_).livo.gaussian.enabled);
  node_->setParam("gaussian/socket_path", 123);
  node_->setParam("gaussian/output_directory", 456);
  EXPECT_NO_THROW(ReadNodeOptions(*node_));
  node_->setParam("gaussian/enabled", true);
  EXPECT_THROW(ReadNodeOptions(*node_), std::exception);
  node_->setParam("gaussian/socket_path", "/tmp/gaussian-test.sock");
  node_->setParam("gaussian/output_directory", "/tmp/gaussian-test-output");
  EXPECT_THROW(ReadNodeOptions(*node_), std::invalid_argument);
  node_->setParam("keyframes/enabled", true);
  node_->setParam("submaps/enabled", true);
  node_->setParam("submaps/mode", "online");
  const auto config = ReadNodeOptions(*node_).livo;
  EXPECT_TRUE(config.gaussian.enabled);
  EXPECT_EQ(config.gaussian.socket_path, "/tmp/gaussian-test.sock");
  EXPECT_FALSE(std::filesystem::exists(directory_));
}
}  // namespace
}  // namespace livo_atlas::ros1

namespace livo_atlas::ros1 {
namespace {
TEST_F(Ros1ParametersTest, GlobalSchemaAndStorageAreValidatedBeforeOutput) {
  EXPECT_FALSE(ReadNodeOptions(*node_).livo.global_output.enabled);
  node_->setParam("global_output/map_frame", "world");
  EXPECT_NO_THROW(ReadNodeOptions(*node_));
  node_->setParam("global_output/enabled", true);
  EXPECT_THROW(ReadNodeOptions(*node_), std::invalid_argument);
  std::filesystem::create_directories(directory_);
  const auto sentinel = directory_ / "mat_pre.txt";
  std::ofstream(sentinel) << "keep global output";
  node_->setParam("keyframes/enabled", true);
  node_->setParam("submaps/enabled", true);
  node_->setParam("submaps/mode", "online");
  node_->setParam("submaps/max_keyframes", 128);
  EXPECT_THROW(ReadNodeOptions(*node_), std::invalid_argument);
  node_->setParam("global_output/map_frame", "atlas/map_2");
  node_->setParam("global_output/publish_tf", false);
  const auto config = ReadNodeOptions(*node_).livo;
  EXPECT_TRUE(config.global_output.enabled);
  EXPECT_FALSE(config.global_output.publish_tf);
  EXPECT_EQ(config.global_output.map_frame, "atlas/map_2");
  EXPECT_FALSE(config.system.loop_closure);
  EXPECT_FALSE(config.system.geometry.enabled);
  for (const auto* key :
       {"global_output/enabled", "global_output/publish_tf"}) {
    node_->setParam(key, "true");
    EXPECT_THROW(ReadNodeOptions(*node_), std::exception);
    node_->setParam(key, true);
  }
  node_->setParam("global_output/map_frame", 123);
  EXPECT_THROW(ReadNodeOptions(*node_), std::exception);
  std::ifstream input(sentinel);
  EXPECT_EQ(std::string(std::istreambuf_iterator<char>(input), {}),
            "keep global output");
}
}  // namespace
}  // namespace livo_atlas::ros1
