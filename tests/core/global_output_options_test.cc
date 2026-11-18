#include "livo_atlas/core/global_output_options.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "core/fixtures.h"
#include "livo_atlas/core/livo_config.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;

// Supplies ordinary typed values while the real shared loader validates the
// schema. No file writer, ROS parameter service or backend is substituted.
class GlobalParameters : public ParameterReader {
 public:
  explicit GlobalParameters(const fs::path& directory) {
    strings_["cam_model"] = "Pinhole";
    strings_["output.directory"] = directory.string();
    integers_["cam_width"] = integers_["cam_height"] = 512;
  }

  bool Boolean(const std::string& name, bool fallback) override {
    const auto value = booleans_.find(name);
    return value == booleans_.end() ? fallback : value->second;
  }

  int Integer(const std::string& name, int fallback) override {
    const auto value = integers_.find(name);
    return value == integers_.end() ? fallback : value->second;
  }

  double Number(const std::string& name, double fallback) override {
    const auto value = numbers_.find(name);
    if (value != numbers_.end()) return value->second;
    if (name == "cam_fx") return 200;
    if (name == "cam_fy") return 210;
    if (name == "cam_cx" || name == "cam_cy") return 256;
    return fallback;
  }

  std::string String(const std::string& name,
                     const std::string& fallback) override {
    const auto value = strings_.find(name);
    return value == strings_.end() ? fallback : value->second;
  }

  std::vector<double> NumberArray(const std::string& name) override {
    if (name == "extrin_calib.extrinsic_R" || name == "extrin_calib.Rcl")
      return {1, 0, 0, 0, 1, 0, 0, 0, 1};
    if (name == "extrin_calib.extrinsic_T" || name == "extrin_calib.Pcl")
      return {0, 0, 0};
    throw std::logic_error("Unexpected calibration array: " + name);
  }

  std::vector<int> IntegerArray(const std::string&,
                                const std::vector<int>& fallback) override {
    return fallback;
  }

  std::string Name(const std::string& name) const override { return name; }

  void EnableOnlineStorage() {
    booleans_["keyframes.enabled"] = true;
    booleans_["submaps.enabled"] = true;
    strings_["submaps.mode"] = "online";
  }

  std::map<std::string, bool> booleans_;
  std::map<std::string, int> integers_;
  std::map<std::string, std::string> strings_;
  std::map<std::string, double> numbers_;
};

class GlobalOutputOptionsTest : public testing::Test {
 protected:
  void TearDown() override { fs::remove_all(directory_); }

  fs::path directory_ =
      test::Options(FrontendMode::kLidarOnly).exporter.directory;
};

TEST_F(GlobalOutputOptionsTest, DisabledPreservesLegacyConfiguration) {
  GlobalOutputOptions options;
  options.map_frame = "invalid disabled/frame/";
  EXPECT_NO_THROW(ValidateGlobalOutputOptions(options));
  GlobalParameters parameters(directory_);
  parameters.strings_["global_output.map_frame"] = options.map_frame;
  const auto config = LoadLivoConfig(parameters);
  EXPECT_FALSE(config.global_output.enabled);
  EXPECT_FALSE(config.system.keyframes.enabled);
  EXPECT_FALSE(config.system.submaps.enabled);
  EXPECT_FALSE(config.loop_closure_enabled);
  EXPECT_FALSE(fs::exists(directory_));
}

TEST_F(GlobalOutputOptionsTest, GaussianPreviewRequiresJointAndSafeFrame) {
  GlobalParameters parameters(directory_);
  EXPECT_FALSE(LoadLivoConfig(parameters).gaussian_output.enabled);
  parameters.booleans_["gaussian.publish_preview"] = true;
  EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  parameters.EnableOnlineStorage();
  parameters.booleans_["gaussian.enabled"] = true;
  parameters.booleans_["gaussian.persist_submaps"] = true;
  parameters.strings_["gaussian.socket_path"] = "/tmp/preview-test.sock";
  parameters.strings_["gaussian.output_directory"] = "/tmp/preview-test";
  EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  parameters.booleans_["gaussian.joint_render_enabled"] = true;
  parameters.integers_["gaussian.joint_render_interval_ms"] = 234;
  const auto config = LoadLivoConfig(parameters);
  EXPECT_TRUE(config.gaussian_output.enabled);
  EXPECT_EQ(config.gaussian_output.minimum_interval.count(), 234);
  EXPECT_EQ(config.gaussian_output.optical_frame, "gaussian_camera_optical");
  for (const auto* frame : {"", "/cam", "cam/", "cam//sub", "camera_init",
                            "aft_mapped", "world", "map", "bad frame"}) {
    parameters.strings_["gaussian.preview_frame_id"] = frame;
    EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument) << frame;
  }
  parameters.strings_["gaussian.preview_frame_id"] = std::string(129, 'a');
  EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  parameters.strings_["gaussian.preview_frame_id"] = "atlas/camera_optical";
  EXPECT_NO_THROW(LoadLivoConfig(parameters));
  for (const auto* frame : {"", "/map", "map/", "bad frame", "world"}) {
    parameters.strings_["global_output.map_frame"] = frame;
    EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument) << frame;
  }
  parameters.strings_["global_output.map_frame"] = std::string(257, 'm');
  EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  parameters.strings_["global_output.map_frame"] = "atlas/map";
  EXPECT_NO_THROW(LoadLivoConfig(parameters));
  EXPECT_FALSE(fs::exists(directory_));
}

TEST_F(GlobalOutputOptionsTest, GaussianJointRenderDependenciesAndRanges) {
  GlobalParameters parameters(directory_);
  auto defaults = LoadLivoConfig(parameters).gaussian;
  EXPECT_FALSE(defaults.joint_render_enabled);
  EXPECT_EQ(defaults.joint_render_interval.count(), 1000);
  EXPECT_EQ(defaults.joint_render_timeout.count(), 5000);
  parameters.booleans_["gaussian.joint_render_enabled"] = true;
  EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  parameters.booleans_["gaussian.enabled"] = true;
  parameters.strings_["gaussian.socket_path"] = "/tmp/joint-test.sock";
  parameters.strings_["gaussian.output_directory"] = "/tmp/joint-output";
  parameters.EnableOnlineStorage();
  EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  parameters.booleans_["gaussian.persist_submaps"] = true;
  for (int interval : {-1, 0, 99, 60001}) {
    parameters.integers_["gaussian.joint_render_interval_ms"] = interval;
    EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  }
  for (int interval : {100, 1000, 60000}) {
    parameters.integers_["gaussian.joint_render_interval_ms"] = interval;
    EXPECT_EQ(LoadLivoConfig(parameters).gaussian.joint_render_interval.count(),
              interval);
  }
  for (int timeout : {-1, 0, 60001}) {
    parameters.integers_["gaussian.joint_render_timeout_ms"] = timeout;
    EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  }
  for (int timeout : {1, 5000, 60000}) {
    parameters.integers_["gaussian.joint_render_timeout_ms"] = timeout;
    EXPECT_EQ(LoadLivoConfig(parameters).gaussian.joint_render_timeout.count(),
              timeout);
  }
  EXPECT_FALSE(fs::exists(directory_));
}

TEST_F(GlobalOutputOptionsTest, GaussianSpatialPolicyIsCheckedBeforeOutputIo) {
  GlobalParameters parameters(directory_);
  EXPECT_EQ(LoadLivoConfig(parameters).gaussian.joint_render_policy,
            GaussianRenderPolicy::kTemporal);
  for (const auto* value : {"", "unknown", "Spatial"}) {
    parameters.strings_["gaussian.joint_render_policy"] = value;
    EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  }
  parameters.strings_["gaussian.joint_render_policy"] = "spatial";
  EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  parameters.EnableOnlineStorage();
  parameters.booleans_["gaussian.enabled"] = true;
  parameters.strings_["gaussian.socket_path"] = "/tmp/spatial-test.sock";
  parameters.strings_["gaussian.output_directory"] = "/tmp/spatial-output";
  parameters.booleans_["gaussian.persist_submaps"] = true;
  EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  parameters.booleans_["gaussian.joint_render_enabled"] = true;
  EXPECT_EQ(LoadLivoConfig(parameters).gaussian.joint_render_policy,
            GaussianRenderPolicy::kSpatial);
  EXPECT_FALSE(fs::exists(directory_));
}

TEST_F(GlobalOutputOptionsTest,
       GaussianFinalizationRequiresPersistenceAndBudget) {
  GlobalParameters parameters(directory_);
  EXPECT_FALSE(LoadLivoConfig(parameters).gaussian.finalize_on_shutdown);
  EXPECT_EQ(LoadLivoConfig(parameters).gaussian.finalize_timeout.count(),
            60000);
  parameters.booleans_["gaussian.finalize_on_shutdown"] = true;
  EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  parameters.booleans_["gaussian.enabled"] = true;
  parameters.strings_["gaussian.socket_path"] = "/tmp/finalize-test.sock";
  parameters.strings_["gaussian.output_directory"] = "/tmp/finalize-output";
  parameters.EnableOnlineStorage();
  EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  parameters.booleans_["gaussian.persist_submaps"] = true;
  for (const int timeout : {0, -1, 600001}) {
    parameters.integers_["gaussian.finalize_timeout_ms"] = timeout;
    EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  }
  for (const int timeout : {1, 60000, 600000}) {
    parameters.integers_["gaussian.finalize_timeout_ms"] = timeout;
    const auto options = LoadLivoConfig(parameters).gaussian;
    EXPECT_TRUE(options.finalize_on_shutdown);
    EXPECT_EQ(options.finalize_timeout.count(), timeout);
  }
  EXPECT_FALSE(fs::exists(directory_));
}

TEST_F(GlobalOutputOptionsTest, GaussianRequiresPathsAndOnlineStorage) {
  GlobalParameters parameters(directory_);
  EXPECT_FALSE(LoadLivoConfig(parameters).gaussian.enabled);
  parameters.booleans_["gaussian.enabled"] = true;
  EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  parameters.strings_["gaussian.socket_path"] = "/tmp/gaussian-test.sock";
  parameters.strings_["gaussian.output_directory"] =
      "/tmp/gaussian-test-output";
  EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  parameters.EnableOnlineStorage();
  const auto config = LoadLivoConfig(parameters);
  EXPECT_TRUE(config.gaussian.enabled);
  EXPECT_EQ(config.gaussian.socket_path, "/tmp/gaussian-test.sock");
  EXPECT_EQ(config.gaussian.output_directory, "/tmp/gaussian-test-output");
  EXPECT_FALSE(fs::exists(directory_));
}

TEST_F(GlobalOutputOptionsTest,
       GaussianPersistenceRequiresEnabledAndPositiveTimeout) {
  GlobalParameters parameters(directory_);
  EXPECT_FALSE(LoadLivoConfig(parameters).gaussian.persist_submaps);
  EXPECT_EQ(LoadLivoConfig(parameters).gaussian.shutdown_timeout.count(), 3000);
  parameters.booleans_["gaussian.persist_submaps"] = true;
  EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  parameters.booleans_["gaussian.enabled"] = true;
  parameters.strings_["gaussian.socket_path"] = "/tmp/gaussian-test.sock";
  parameters.strings_["gaussian.output_directory"] =
      "/tmp/gaussian-test-output";
  parameters.EnableOnlineStorage();
  parameters.integers_["gaussian.shutdown_timeout_ms"] = 30000;
  EXPECT_TRUE(LoadLivoConfig(parameters).gaussian.persist_submaps);
  EXPECT_EQ(LoadLivoConfig(parameters).gaussian.shutdown_timeout.count(),
            30000);
  parameters.integers_["gaussian.shutdown_timeout_ms"] = 0;
  EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  EXPECT_FALSE(fs::exists(directory_));
}

TEST_F(GlobalOutputOptionsTest,
       GaussianPathCheckDoesNotCreateOrOverwriteFiles) {
  fs::create_directories(directory_);
  const auto dataset = directory_ / "future-dataset/keyframes";
  OnlineGaussianOptions options;
  options.enabled = true;
  options.socket_path = directory_ / "worker.sock";
  options.output_directory = directory_ / "previews";
  EXPECT_NO_THROW(ValidateOnlineGaussianPaths(options, dataset));
  EXPECT_FALSE(fs::exists(dataset));
  EXPECT_FALSE(fs::exists(options.output_directory));
  fs::create_directory(options.output_directory);
  const auto sentinel = options.output_directory / "keep";
  std::ofstream(sentinel) << "unchanged";
  EXPECT_THROW(ValidateOnlineGaussianPaths(options, dataset), std::exception);
  std::ifstream input(sentinel);
  EXPECT_EQ(std::string(std::istreambuf_iterator<char>(input), {}),
            "unchanged");
  options.output_directory = directory_ / "future-dataset";
  EXPECT_THROW(ValidateOnlineGaussianPaths(options, dataset), std::exception);
  options.output_directory = directory_ / "another-output";
  options.socket_path = directory_ / std::string(108, 's');
  EXPECT_THROW(ValidateOnlineGaussianPaths(options, dataset),
               std::invalid_argument);
}

TEST_F(GlobalOutputOptionsTest, StartupBufferDefaultsOnAndCanBeDisabled) {
  GlobalParameters parameters(directory_);
  EXPECT_TRUE(LoadLivoConfig(parameters).system.buffer_before_first_lidar);
  parameters.booleans_["common.buffer_before_first_lidar"] = false;
  EXPECT_FALSE(LoadLivoConfig(parameters).system.buffer_before_first_lidar);
}

TEST_F(GlobalOutputOptionsTest, ValidFrameNamesAndIndependentControls) {
  GlobalOutputOptions options;
  options.enabled = true;
  options.publish_tf = false;
  for (const auto* name : {"map", "atlas/map_2", "123", "_atlas"}) {
    options.map_frame = name;
    EXPECT_NO_THROW(ValidateGlobalOutputOptions(options)) << name;
  }
}

TEST_F(GlobalOutputOptionsTest,
       TrajectoryRequiresGlobalButIgnoresDisabledBudgets) {
  GlobalParameters parameters(directory_);
  parameters.EnableOnlineStorage();
  parameters.booleans_["global_output.trajectory.enabled"] = true;
  EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  parameters.booleans_["global_output.enabled"] = true;
  parameters.numbers_["global_output.trajectory.max_frequency_hz"] = 2;
  parameters.integers_["global_output.trajectory.max_keyframes"] = 100;
  const auto config = LoadLivoConfig(parameters);
  EXPECT_TRUE(config.global_output.trajectory.enabled);
  EXPECT_DOUBLE_EQ(config.global_output.trajectory.max_frequency_hz, 2);
  EXPECT_EQ(config.global_output.trajectory.max_keyframes, 100);
  EXPECT_FALSE(config.loop_closure_enabled);
  EXPECT_FALSE(config.system.geometry.enabled);
  EXPECT_FALSE(fs::exists(directory_));
  for (double rate : {0.0, -1.0, 10.1, std::numeric_limits<double>::infinity(),
                      std::numeric_limits<double>::quiet_NaN()}) {
    parameters.numbers_["global_output.trajectory.max_frequency_hz"] = rate;
    EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  }
  parameters.numbers_["global_output.trajectory.max_frequency_hz"] = 10;
  for (int count : {0, -1, 100001}) {
    parameters.integers_["global_output.trajectory.max_keyframes"] = count;
    EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  }
  for (int count : {1, 100000}) {
    parameters.integers_["global_output.trajectory.max_keyframes"] = count;
    EXPECT_NO_THROW(LoadLivoConfig(parameters));
  }
  parameters.booleans_["global_output.trajectory.enabled"] = false;
  parameters.numbers_["global_output.trajectory.max_frequency_hz"] = -1;
  parameters.integers_["global_output.trajectory.max_keyframes"] = -1;
  EXPECT_NO_THROW(LoadLivoConfig(parameters));
}

TEST_F(GlobalOutputOptionsTest, RejectsUnsafeOrConflictingFrameNames) {
  GlobalOutputOptions options;
  options.enabled = true;
  for (const auto* name :
       {"", "/map", "map/", "map//sub", "map frame", "map\nsub", "map.foo",
        ":map", u8"地图", "camera_init", "aft_mapped", "world"}) {
    options.map_frame = name;
    EXPECT_THROW(ValidateGlobalOutputOptions(options), std::invalid_argument)
        << name;
  }
}

TEST_F(GlobalOutputOptionsTest, ReadsGlobalSchemaBeforeOutputIo) {
  GlobalParameters parameters(directory_);
  parameters.EnableOnlineStorage();
  parameters.booleans_["global_output.enabled"] = true;
  parameters.booleans_["global_output.publish_tf"] = false;
  parameters.strings_["global_output.map_frame"] = "atlas/map_2";
  // Global display does not inherit the registration cache's 64-member cap.
  parameters.integers_["submaps.max_keyframes"] = 128;
  const auto config = LoadLivoConfig(parameters);
  EXPECT_TRUE(config.global_output.enabled);
  EXPECT_FALSE(config.global_output.publish_tf);
  EXPECT_EQ(config.global_output.map_frame, "atlas/map_2");
  EXPECT_TRUE(config.system.keyframes.enabled);
  EXPECT_EQ(config.system.submap_mode, SubmapMode::kOnline);
  EXPECT_EQ(config.system.submaps.max_keyframes, 128);
  EXPECT_FALSE(config.loop_closure_enabled);
  EXPECT_FALSE(config.system.geometry.enabled);
  EXPECT_FALSE(fs::exists(directory_));
}

TEST_F(GlobalOutputOptionsTest,
       GeometryDisplayReadsSharedOptionsIndependently) {
  GlobalParameters parameters(directory_);
  parameters.EnableOnlineStorage();
  parameters.booleans_["global_output.enabled"] = true;
  parameters.booleans_["global_output.geometry.enabled"] = true;
  parameters.numbers_["submaps.geometry.voxel_size"] = 0.4;
  parameters.integers_["global_output.geometry.max_points"] = 1000;
  const auto config = LoadLivoConfig(parameters);
  EXPECT_TRUE(config.global_output.geometry.enabled);
  EXPECT_EQ(config.global_output.geometry.map.max_points, 1000);
  EXPECT_DOUBLE_EQ(config.system.geometry.voxel_size, 0.4);
  EXPECT_FALSE(config.system.geometry.enabled);
  EXPECT_FALSE(config.global_output.trajectory.enabled);
  EXPECT_FALSE(fs::exists(directory_));
  parameters.numbers_["submaps.geometry.voxel_size"] = -1;
  EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  parameters.booleans_["global_output.geometry.enabled"] = false;
  EXPECT_NO_THROW(LoadLivoConfig(parameters));
}

TEST_F(GlobalOutputOptionsTest,
       GeometryDisplayRejectsInvalidBudgetsAndStorage) {
  GlobalParameters parameters(directory_);
  parameters.booleans_["global_output.geometry.enabled"] = true;
  EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  parameters.booleans_["global_output.enabled"] = true;
  EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  parameters.EnableOnlineStorage();
  EXPECT_NO_THROW(LoadLivoConfig(parameters));
  parameters.integers_["submaps.max_keyframes"] = 65;
  EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  parameters.integers_["submaps.max_keyframes"] = 64;
  EXPECT_NO_THROW(LoadLivoConfig(parameters));
  parameters.strings_["global_output.map_frame"] = std::string(257, 'a');
  EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  parameters.strings_["global_output.map_frame"] = std::string(256, 'a');
  EXPECT_NO_THROW(LoadLivoConfig(parameters));
  for (double rate : {0.0, 2.1, std::numeric_limits<double>::infinity(),
                      std::numeric_limits<double>::quiet_NaN()}) {
    parameters.numbers_["global_output.geometry.max_frequency_hz"] = rate;
    EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  }
  parameters.numbers_["global_output.geometry.max_frequency_hz"] = 2;
  for (int count : {0, 100001}) {
    parameters.integers_["global_output.geometry.max_keyframes"] = count;
    EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  }
  parameters.integers_["global_output.geometry.max_keyframes"] = 100000;
  EXPECT_NO_THROW(LoadLivoConfig(parameters));
  parameters.integers_["global_output.geometry.max_points"] = 0;
  EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  parameters.booleans_["global_output.geometry.enabled"] = false;
  EXPECT_NO_THROW(LoadLivoConfig(parameters));
}

TEST_F(GlobalOutputOptionsTest, RequiresOnlineStorageOnlyWhenEnabled) {
  GlobalParameters parameters(directory_);
  EXPECT_NO_THROW(LoadLivoConfig(parameters));
  parameters.booleans_["global_output.enabled"] = true;
  EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  parameters.booleans_["keyframes.enabled"] = true;
  EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  parameters.booleans_["submaps.enabled"] = true;
  EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  parameters.strings_["submaps.mode"] = "online";
  EXPECT_NO_THROW(LoadLivoConfig(parameters));
  EXPECT_FALSE(fs::exists(directory_));
}

TEST_F(GlobalOutputOptionsTest, InvalidGlobalConfigPreservesExistingOutput) {
  fs::create_directories(directory_);
  const auto sentinel = directory_ / "mat_pre.txt";
  std::ofstream(sentinel) << "keep existing output";
  GlobalParameters parameters(directory_);
  parameters.EnableOnlineStorage();
  parameters.booleans_["global_output.enabled"] = true;
  parameters.strings_["global_output.map_frame"] = "world";
  EXPECT_THROW(LoadLivoConfig(parameters), std::invalid_argument);
  std::ifstream stream(sentinel);
  EXPECT_EQ(std::string(std::istreambuf_iterator<char>(stream), {}),
            "keep existing output");
}

}  // namespace
}  // namespace livo_atlas
