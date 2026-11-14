#include "livo_atlas/core/global_output_options.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
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

TEST_F(GlobalOutputOptionsTest, ValidFrameNamesAndIndependentControls) {
  GlobalOutputOptions options;
  options.enabled = true;
  options.publish_tf = false;
  for (const auto* name : {"map", "atlas/map_2", "123", "_atlas"}) {
    options.map_frame = name;
    EXPECT_NO_THROW(ValidateGlobalOutputOptions(options)) << name;
  }
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
