#include "livo_atlas/io/gaussian_render_writer.h"

#include <gtest/gtest.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <opencv2/imgcodecs.hpp>

#include "gaussian/render_test_helpers.h"

namespace {
bool fail_render_rename = false;
}

extern "C" int rename(const char* from, const char* to) noexcept {
  if (fail_render_rename && std::string(from).find(".tmp.json") !=
                                std::string::npos) {
    errno = EIO;
    return -1;
  }
  return static_cast<int>(syscall(SYS_rename, from, to));
}

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;

class GaussianRenderWriterTest : public testing::Test {
 protected:
  void SetUp() override {
    directory_ = fs::temp_directory_path() /
        ("joint-writer-" + std::to_string(getpid()) + "-" +
         std::to_string(std::chrono::steady_clock::now()
                            .time_since_epoch().count()));
    fs::create_directory(directory_);
    const auto request = test::RenderRequest();
    render_ = DecodeGaussianRenderReply(
        test::RenderReply(EncodeGaussianRenderRequest(request)), request);
    render_.accepted_at_unix_seconds = 1000;
    render_.accepted_at_steady_seconds = 900;
  }

  void TearDown() override {
    fail_render_rename = false;
    fs::remove_all(directory_);
  }

  fs::path Path(const std::string& extension) {
    return directory_ /
        ("joint-render-" + std::to_string(render_.request.job_id) + extension);
  }

  std::string Metadata() {
    std::ifstream file(Path(".json"));
    return {std::istreambuf_iterator<char>(file), {}};
  }

  fs::path directory_;
  GaussianRender render_;
};

TEST_F(GaussianRenderWriterTest, PublishesPixelsAndFixedAcceptedIdentity) {
  render_.rgb[0] = 12;
  render_.rgb[1] = 34;
  render_.rgb[2] = 56;
  WriteGaussianRender(directory_, render_);
  const auto image = cv::imread(Path(".png").string());
  ASSERT_FALSE(image.empty());
  EXPECT_EQ(image.cols, 8);
  EXPECT_EQ(image.rows, 6);
  EXPECT_EQ(image.at<cv::Vec3b>(0, 0), cv::Vec3b(56, 34, 12));
  const auto metadata = Metadata();
  EXPECT_NE(metadata.find("9007199254740993"), std::string::npos);
  EXPECT_NE(metadata.find("accepted_at_steady_seconds"), std::string::npos);
  EXPECT_NE(metadata.find("as_of"), std::string::npos);
  EXPECT_LE(metadata.size(), 65536u);
  EXPECT_LE(fs::file_size(Path(".png")), 4u * 1024 * 1024);
  EXPECT_FALSE(fs::exists(Path(".tmp.json")));
  EXPECT_FALSE(fs::exists(Path(".tmp.png")));
}

TEST_F(GaussianRenderWriterTest, EmptyMetricsAreJsonNullAndPerfectIsExplicit) {
  render_.single = {};
  render_.joint = {};
  render_.joint.image_pixels = 48;
  render_.joint.rgb_l1 = render_.joint.rgb_mse = 0;
  render_.joint.alpha_coverage = 0;
  render_.joint.rgb_perfect = true;
  WriteGaussianRender(directory_, render_);
  const auto text = Metadata();
  EXPECT_NE(text.find("\"rgb_psnr_db\":null"), std::string::npos);
  EXPECT_NE(text.find("\"depth_l1_m\":null"), std::string::npos);
  EXPECT_NE(text.find("\"rgb_perfect\":true"), std::string::npos);
}

TEST_F(GaussianRenderWriterTest, ProtectsExistingPathsAndRejectsBadPixels) {
  std::ofstream(Path(".json")) << "existing data";
  EXPECT_THROW(WriteGaussianRender(directory_, render_), std::exception);
  EXPECT_EQ(Metadata(), "existing data");
  EXPECT_FALSE(fs::exists(Path(".png")));
  fs::remove(Path(".json"));
  render_.rgb.pop_back();
  EXPECT_THROW(WriteGaussianRender(directory_, render_), std::exception);
  EXPECT_FALSE(fs::exists(Path(".json")));
}

TEST_F(GaussianRenderWriterTest, LastRenameFailureLeavesNoSuccessfulPair) {
  fail_render_rename = true;
  EXPECT_THROW(WriteGaussianRender(directory_, render_), std::exception);
  EXPECT_FALSE(fs::exists(Path(".json")));
  EXPECT_FALSE(fs::exists(Path(".png")));
  EXPECT_FALSE(fs::exists(Path(".tmp.json")));
  EXPECT_FALSE(fs::exists(Path(".tmp.png")));
}

TEST_F(GaussianRenderWriterTest, RejectsContradictoryOptionalPsnr) {
  const auto original = render_;
  for (int mode = 0; mode < 4; ++mode) {
    render_ = original;
    if (mode == 0) render_.joint.rgb_psnr_db.reset();
    if (mode == 1) {
      render_.joint.rgb_l1 = render_.joint.rgb_mse = 0;
      render_.joint.rgb_perfect = true;
    }
    if (mode == 2) {
      render_.joint = {};
      render_.joint.rgb_psnr_db = 1;
    }
    if (mode == 3) render_.joint.rgb_psnr_db = 1;
    EXPECT_THROW(WriteGaussianRender(directory_, render_),
                 std::invalid_argument) << mode;
    EXPECT_FALSE(fs::exists(Path(".json")));
    EXPECT_FALSE(fs::exists(Path(".png")));
    fs::remove(Path(".json"));
    fs::remove(Path(".png"));
  }
}

TEST_F(GaussianRenderWriterTest, RejectsRenderRotationBeyondWireTolerance) {
  render_.camera_from_body.rotation(0, 0) += 2e-6;
  EXPECT_THROW(WriteGaussianRender(directory_, render_),
               std::invalid_argument);
  EXPECT_FALSE(fs::exists(Path(".json")));
}

TEST_F(GaussianRenderWriterTest, RejectsCameraAndImageSemanticsInvalidOnWire) {
  const auto original = render_;
  for (int mode = 0; mode < 6; ++mode) {
    render_ = original;
    if (mode == 0) render_.K(0, 0) = -1;
    if (mode == 1) render_.camera_from_parent.translation.x() += 1;
    if (mode == 2) render_.changed_pixels = 49;
    if (mode == 3) render_.worker_seconds = -1;
    if (mode == 4) render_.calibration_id = 1;
    if (mode == 5) render_.K(0, 1) = 1;
    EXPECT_THROW(WriteGaussianRender(directory_, render_),
                 std::invalid_argument) << mode;
    EXPECT_FALSE(fs::exists(Path(".json")));
    fs::remove(Path(".json"));
    fs::remove(Path(".png"));
  }
}

}  // namespace
}  // namespace livo_atlas
