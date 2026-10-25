#include "livo_atlas/io/livo_exporter.h"

#include <Eigen/Geometry>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <opencv2/core/mat.hpp>
#include <sstream>
#include <stdexcept>
#include <vector>

#ifdef __linux__
#include <unistd.h>
#endif

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/imgcodecs.hpp>

namespace livo_atlas {
namespace {

namespace fs = std::filesystem;

std::vector<std::vector<double>> ReadRows(const fs::path& path) {
  std::ifstream input(path);
  std::vector<std::vector<double>> rows;
  std::string line;
  while (std::getline(input, line)) {
    std::istringstream stream(line);
    std::vector<double> row;
    double value;
    while (stream >> value) row.push_back(value);
    if (!row.empty()) rows.push_back(row);
  }
  return rows;
}

class ResultWriterTest : public testing::Test {
 protected:
  void SetUp() override {
    const auto id = std::chrono::steady_clock::now().time_since_epoch().count();
    root_ = fs::temp_directory_path() / ("livo_writer_" + std::to_string(id));
    ASSERT_TRUE(fs::create_directory(root_));
    options_.directory = root_ / "output";
    options_.mode = FrontendMode::kLidarOnly;
    options_.calibration.camera = {"Pinhole", 32, 24, 1.0, 10, 12, 16, 12, {}};
  }

  void TearDown() override { fs::remove_all(root_); }

  FrontendResult Result(UpdateKind kind = UpdateKind::kLidarOnly) {
    FrontendResult result;
    result.kind = kind;
    result.updated = true;
    result.timestamp = 100.25;
    result.relative_time = 0.25;
    result.prediction.pos_end << 1, 2, 3;
    result.estimate.pos_end << 2, 3, 4;
    PointCloud::Ptr cloud(new PointCloud);
    Point point;
    point.x = 1;
    point.y = 2;
    point.z = 3;
    point.intensity = 4;
    cloud->push_back(point);
    result.world_cloud = cloud;
    result.undistorted_cloud = cloud;
    return result;
  }

  PreparedOutput Prepared() {
    PreparedOutput output;
    output.has_cloud = true;
    ColoredCloud::Ptr cloud(new ColoredCloud);
    pcl::PointXYZRGB point(std::uint8_t{10}, std::uint8_t{20},
                           std::uint8_t{30});
    point.x = 1;
    point.y = 2;
    point.z = 3;
    cloud->push_back(point);
    output.colored_cloud = cloud;
    output.colmap_image = cv::Mat(2, 3, CV_8UC3, cv::Scalar(7, 8, 9));
    return output;
  }

  fs::path root_;
  ExportOptions options_;
};

TEST_F(ResultWriterTest, FailedUpdateWritesOnlyPrediction) {
  LivoExporter writer(options_);
  auto result = Result();
  result.updated = false;
  writer.Write(result, {});
  writer.Finish();
  const auto pre = ReadRows(options_.directory / "mat_pre.txt");
  ASSERT_EQ(pre.size(), 1);
  ASSERT_EQ(pre[0].size(), 19);
  EXPECT_DOUBLE_EQ(pre[0][0], 0.25);
  EXPECT_DOUBLE_EQ(pre[0][4], 1);
  EXPECT_TRUE(ReadRows(options_.directory / "mat_out.txt").empty());
}

TEST_F(ResultWriterTest, WritesSensorTrajectoryPosteriorAndImuMeans) {
  options_.trajectory = true;
  LivoExporter writer(options_);
  auto result = Result();
  result.imu_initialized = true;
  result.imu_diagnostics.push_back({0.125, {1, 2, 3}, {4, 5, 6}});
  writer.Write(result, Prepared());
  writer.Finish();
  const auto trajectory = ReadRows(options_.directory / "result/01.txt");
  ASSERT_EQ(trajectory.size(), 1);
  EXPECT_EQ(trajectory[0], (std::vector<double>{100.25, 2, 3, 4, 0, 0, 0, 1}));
  const auto posterior = ReadRows(options_.directory / "mat_out.txt");
  ASSERT_EQ(posterior.size(), 1);
  ASSERT_EQ(posterior[0].size(), 20);
  EXPECT_DOUBLE_EQ(posterior[0][4], 2);
  EXPECT_DOUBLE_EQ(posterior[0].back(), 1);
  const auto imu = ReadRows(options_.directory / "imu.txt");
  ASSERT_EQ(imu.size(), 1);
  EXPECT_EQ(imu[0], (std::vector<double>{0.125, 1, 2, 3, 4, 5, 6}));
}

TEST_F(ResultWriterTest, OpensImuLogAtInitializationCompletion) {
  LivoExporter writer(options_);
  auto result = Result();
  result.updated = false;
  writer.Write(result, {});
  EXPECT_FALSE(fs::exists(options_.directory / "imu.txt"));
  result.imu_initialized = true;
  writer.Write(result, {});
  writer.Finish();
  EXPECT_TRUE(fs::exists(options_.directory / "imu.txt"));
}

TEST_F(ResultWriterTest, NegativeIntervalWritesAllPlainPointsOnlyAtFinish) {
  options_.pcd = true;
  LivoExporter writer(options_);
  writer.Write(Result(), Prepared());
  EXPECT_FALSE(fs::exists(options_.directory / "pcd/all_raw_points.pcd"));
  writer.Finish();
  PointCloud cloud;
  ASSERT_EQ(
      pcl::io::loadPCDFile(
          (options_.directory / "pcd/all_raw_points.pcd").string(), cloud),
      0);
  ASSERT_EQ(cloud.size(), 1);
  EXPECT_FLOAT_EQ(cloud[0].x, 1);
  EXPECT_FLOAT_EQ(cloud[0].intensity, 4);
  const auto poses = ReadRows(options_.directory / "pcd/lidar_poses.txt");
  ASSERT_EQ(poses.size(), 1);
  EXPECT_EQ(poses[0], (std::vector<double>{100.25, 2, 3, 4, 0, 0, 0, 1}));
}

TEST_F(ResultWriterTest, PositiveIntervalPreservesPartialTailBehavior) {
  options_.pcd = true;
  options_.pcd_interval = 2;
  LivoExporter writer(options_);
  auto result = Result();
  writer.Write(result, Prepared());
  EXPECT_FALSE(fs::exists(options_.directory / "pcd/100.250000.pcd"));
  result.timestamp = 100.5;
  writer.Write(result, Prepared());
  result.timestamp = 100.75;
  writer.Write(result, Prepared());
  writer.Finish();
  PointCloud cloud;
  ASSERT_EQ(pcl::io::loadPCDFile(
                (options_.directory / "pcd/100.500000.pcd").string(), cloud),
            0);
  EXPECT_EQ(cloud.size(), 2);
  EXPECT_FALSE(fs::exists(options_.directory / "pcd/100.750000.pcd"));
  EXPECT_FALSE(fs::exists(options_.directory / "pcd/all_raw_points.pcd"));
}

TEST_F(ResultWriterTest, BodyPcdUsesImuFromLidarAndForcesSingleScan) {
  options_.pcd = true;
  options_.pcd_type = 1;
  options_.calibration.imu_from_lidar.rotation << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  options_.calibration.imu_from_lidar.translation << 1, 2, 3;
  LivoExporter writer(options_);
  writer.Write(Result(), Prepared());
  writer.Finish();
  PointCloud cloud;
  ASSERT_EQ(pcl::io::loadPCDFile(
                (options_.directory / "pcd/100.250000.pcd").string(), cloud),
            0);
  ASSERT_EQ(cloud.size(), 1);
  EXPECT_FLOAT_EQ(cloud[0].x, -1);
  EXPECT_FLOAT_EQ(cloud[0].y, 3);
  EXPECT_FLOAT_EQ(cloud[0].z, 6);
  EXPECT_FLOAT_EQ(cloud[0].intensity, 4);
}

TEST_F(ResultWriterTest, ImageCadenceWritesRawPixelsAndVisualPose) {
  options_.images = true;
  options_.image_interval = 2;
  LivoExporter writer(options_);
  auto result = Result(UpdateKind::kVisual);
  result.color_image = cv::Mat(2, 3, CV_8UC3, cv::Scalar(11, 22, 33));
  result.tracking_image = cv::Mat(2, 3, CV_8UC3, cv::Scalar(44, 55, 66));
  writer.Write(result, Prepared());
  EXPECT_FALSE(fs::exists(options_.directory / "image/100.250000.png"));
  result.timestamp = 100.5;
  writer.Write(result, Prepared());
  writer.Finish();
  const auto image =
      cv::imread((options_.directory / "image/100.500000.png").string());
  ASSERT_FALSE(image.empty());
  EXPECT_EQ(image.at<cv::Vec3b>(0, 0), cv::Vec3b(11, 22, 33));
  const auto poses = ReadRows(options_.directory / "image/image_poses.txt");
  ASSERT_EQ(poses.size(), 1);
  EXPECT_EQ(poses[0], (std::vector<double>{100.5, 2, 3, 4, 0, 0, 0, 1}));
}

TEST_F(ResultWriterTest, ColmapUsesCameraFromWorldAndPerInstanceImageIds) {
  options_.colmap = true;
  options_.calibration.camera.scale = 0.5;
  auto result = Result(UpdateKind::kVisual);
  result.camera_from_world_rotation = Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5);
  result.camera_from_world_translation << 7, 8, 9;
  for (const auto& name : {"first", "second"}) {
    options_.directory = root_ / name;
    LivoExporter writer(options_);
    writer.Write(result, Prepared());
    writer.Finish();
    EXPECT_TRUE(fs::exists(options_.directory / "Colmap/images/00001.png"));
    EXPECT_FALSE(fs::exists(options_.directory / "Colmap/images/00002.png"));
    const auto poses =
        ReadRows(options_.directory / "Colmap/sparse/0/images.txt");
    ASSERT_EQ(poses.size(), 2);
    EXPECT_EQ(poses[0],
              (std::vector<double>{1, 0.5, 0.5, 0.5, 0.5, 7, 8, 9, 1, 1}));
    // The final numeric 1 is the parsed prefix of the filename 00001.png.
    const auto camera =
        ReadRows(options_.directory / "Colmap/sparse/0/cameras.txt");
    ASSERT_EQ(camera.size(), 1);
    std::ifstream camera_file(options_.directory /
                              "Colmap/sparse/0/cameras.txt");
    std::string text((std::istreambuf_iterator<char>(camera_file)), {});
    EXPECT_NE(text.find("1 PINHOLE 16 12 5.000000 6.000000 8.000000 6.000000"),
              std::string::npos);
  }
}

TEST_F(ResultWriterTest, ColoredEofWritesDownsampleAndColmapPoints) {
  options_.pcd = true;
  options_.colmap = true;
  options_.mode = FrontendMode::kLidarInertialVisual;
  LivoExporter writer(options_);
  writer.Write(Result(UpdateKind::kVisual), Prepared());
  writer.Finish();
  ColoredCloud cloud;
  ASSERT_EQ(
      pcl::io::loadPCDFile(
          (options_.directory / "pcd/all_downsampled_points.pcd").string(),
          cloud),
      0);
  ASSERT_EQ(cloud.size(), 1);
  EXPECT_EQ(cloud[0].r, 10);
  EXPECT_EQ(cloud[0].g, 20);
  EXPECT_EQ(cloud[0].b, 30);
  const auto points =
      ReadRows(options_.directory / "Colmap/sparse/0/points3D.txt");
  ASSERT_EQ(points.size(), 1);
  EXPECT_EQ(points[0], (std::vector<double>{0, 1, 2, 3, 10, 20, 30, 0}));
}

TEST_F(ResultWriterTest, DebugImagesAreWrittenOutsideTheFrontend) {
  LivoExporter writer(options_);
  auto result = Result();
  result.debug_images.push_back(
      {"7_0_normal.png", cv::Mat(2, 3, CV_8UC1, cv::Scalar(42))});
  writer.Write(result, Prepared());
  writer.Finish();
  const auto image = cv::imread(
      (options_.directory / "ref_cur_combine/7_0_normal.png").string(),
      cv::IMREAD_GRAYSCALE);
  ASSERT_FALSE(image.empty());
  EXPECT_EQ(image.at<unsigned char>(0, 0), 42);
}

TEST_F(ResultWriterTest, FinishIsIdempotentAndRejectsFurtherWrites) {
  LivoExporter writer(options_);
  writer.Write(Result(), Prepared());
  writer.Finish();
  EXPECT_NO_THROW(writer.Finish());
  EXPECT_THROW(writer.Write(Result(), Prepared()), std::logic_error);
}

TEST_F(ResultWriterTest, InvalidOptionsNeverCreateOutputDirectories) {
  std::vector<ExportOptions> invalid;
  auto add = [&](const ExportOptions& options) { invalid.push_back(options); };
  auto options = options_;
  options.directory.clear();
  add(options);
  for (const auto& name : {"", ".", "..", "../bad", "a/b", "a\\b"}) {
    options = options_;
    options.sequence_name = name;
    add(options);
  }
  options = options_;
  options.images = true;
  options.image_interval = 0;
  add(options);
  options = options_;
  options.pcd = true;
  options.pcd_interval = 0;
  add(options);
  options.pcd_interval = -1;
  options.pcd_type = 2;
  add(options);
  options.pcd_type = 0;
  options.voxel_size = std::numeric_limits<double>::quiet_NaN();
  add(options);
  options = options_;
  options.colmap = true;
  options.calibration.camera.model = "Equidistant";
  add(options);
  for (const auto& invalid_options : invalid) {
    EXPECT_THROW(LivoExporter writer(invalid_options), std::invalid_argument);
    EXPECT_FALSE(fs::exists(options_.directory));
  }
}

TEST_F(ResultWriterTest, ColmapRejectsInvalidCameraCalibration) {
  options_.colmap = true;
  const auto valid = options_.calibration.camera;
  for (int variant = 0; variant < 6; ++variant) {
    options_.calibration.camera = valid;
    auto& camera = options_.calibration.camera;
    if (variant == 0) camera.width = 0;
    if (variant == 1) camera.scale = std::numeric_limits<double>::quiet_NaN();
    if (variant == 2) camera.scale = std::numeric_limits<double>::infinity();
    if (variant == 3) camera.scale = 1e100;
    if (variant == 4) camera.fx = 0;
    if (variant == 5)
      camera.distortion[0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(ValidateExportOptions(options_), std::invalid_argument);
    EXPECT_FALSE(fs::exists(options_.directory));
  }
}

TEST_F(ResultWriterTest, InvalidColmapCameraPreservesExistingOutput) {
  options_.colmap = true;
  options_.pcd = true;
  options_.calibration.camera.width = 0;
  const std::vector<fs::path> files = {"mat_pre.txt", "mat_out.txt",
                                       "Colmap/images/keep.txt",
                                       "Colmap/sparse/0/keep.txt"};
  for (const auto& file : files) {
    const auto path = options_.directory / file;
    fs::create_directories(path.parent_path());
    std::ofstream(path) << "original";
  }
  EXPECT_THROW(LivoExporter{options_}, std::invalid_argument);
  for (const auto& file : files) {
    std::ifstream stream(options_.directory / file);
    EXPECT_EQ(std::string(std::istreambuf_iterator<char>(stream), {}),
              "original")
        << file;
  }
}

TEST_F(ResultWriterTest, ReportsDirectoryCreationFailure) {
  std::ofstream(options_.directory) << "blocking file";
  EXPECT_THROW(LivoExporter writer(options_), std::runtime_error);
}

TEST_F(ResultWriterTest, ReportsImageWriteFailureAndRejectsLaterWrites) {
  options_.images = true;
  LivoExporter writer(options_);
  fs::remove_all(options_.directory / "image");
  auto result = Result(UpdateKind::kVisual);
  result.color_image = cv::Mat(2, 3, CV_8UC3, cv::Scalar(1, 2, 3));
  EXPECT_THROW(writer.Write(result, Prepared()), std::runtime_error);
  EXPECT_THROW(writer.Write(result, Prepared()), std::logic_error);
  EXPECT_THROW(writer.Finish(), std::runtime_error);
}

TEST_F(ResultWriterTest, ColmapRestartClearsOnlyGeneratedReconstruction) {
  options_.pcd = true;
  options_.colmap = true;
  options_.mode = FrontendMode::kLidarInertialVisual;
  auto result = Result(UpdateKind::kVisual);
  {
    LivoExporter writer(options_);
    writer.Write(result, Prepared());
    result.timestamp = 100.5;
    writer.Write(result, Prepared());
    writer.Finish();
  }
  ASSERT_TRUE(fs::exists(options_.directory / "Colmap/images/00002.png"));
  std::ofstream(options_.directory / "keep.txt") << "keep this file";
  {
    LivoExporter writer(options_);
    writer.Write(result, Prepared());
    writer.Finish();
  }
  EXPECT_TRUE(fs::exists(options_.directory / "Colmap/images/00001.png"));
  EXPECT_FALSE(fs::exists(options_.directory / "Colmap/images/00002.png"));
  EXPECT_TRUE(fs::exists(options_.directory / "keep.txt"));
}

void ExpectFileFailure(const std::function<void()>& operation,
                       const fs::path& path) {
  try {
    operation();
    FAIL() << "Expected a file error for " << path;
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string(error.what()).find(path.string()), std::string::npos)
        << error.what();
  }
}

TEST_F(ResultWriterTest, ReportsSpecificFileOnActualTextWriteFailure) {
#ifdef __linux__
  fs::create_directories(options_.directory);
  const auto path = options_.directory / "mat_out.txt";
  fs::create_symlink("/dev/full", path);
  LivoExporter writer(options_);
  ExpectFileFailure([&] { writer.Write(Result(), Prepared()); }, path);
  EXPECT_THROW(writer.Write(Result(), Prepared()), std::logic_error);
  EXPECT_THROW(writer.Finish(), std::runtime_error);
#else
  GTEST_SKIP() << "Requires the Linux /dev/full device";
#endif
}

TEST_F(ResultWriterTest, ReportsSpecificFileOnActualTextCloseFailure) {
#ifdef __linux__
  LivoExporter writer(options_);
  const auto path = options_.directory / "mat_out.txt";
  int descriptor = -1;
  for (const auto& entry : fs::directory_iterator("/proc/self/fd")) {
    std::error_code error;
    if (fs::read_symlink(entry.path(), error) == path && !error) {
      descriptor = std::stoi(entry.path().filename().string());
      break;
    }
  }
  ASSERT_GT(descriptor, 2);
  ASSERT_EQ(close(descriptor), 0);
  ExpectFileFailure([&] { writer.Finish(); }, path);
  EXPECT_THROW(writer.Finish(), std::runtime_error);
#else
  GTEST_SKIP() << "Requires Linux file descriptor inspection";
#endif
}

TEST_F(ResultWriterTest, InterleavedWritersKeepSeparatePcdAndImageCadences) {
  auto first_options = options_;
  first_options.directory = root_ / "first";
  first_options.mode = FrontendMode::kLidarInertialVisual;
  first_options.pcd = true;
  first_options.images = true;
  first_options.pcd_interval = 2;
  first_options.image_interval = 2;
  auto second_options = first_options;
  second_options.directory = root_ / "second";
  second_options.pcd_interval = 3;
  second_options.image_interval = 3;
  LivoExporter first(first_options);
  LivoExporter second(second_options);
  auto lidar_output = Prepared();
  lidar_output.colored_cloud.reset(new ColoredCloud);
  for (int frame = 1; frame <= 3; ++frame) {
    auto lidar = Result(UpdateKind::kLidarInertial);
    lidar.timestamp = 100 + frame * 0.25;
    auto visual = Result(UpdateKind::kVisual);
    visual.timestamp = lidar.timestamp + 0.0625;
    visual.color_image = cv::Mat(2, 3, CV_8UC3, cv::Scalar(7, 8, 9));
    first.Write(lidar, lidar_output);
    second.Write(lidar, lidar_output);
    first.Write(visual, Prepared());
    second.Write(visual, Prepared());
  }
  first.Finish();
  second.Finish();
  EXPECT_TRUE(fs::exists(first_options.directory / "pcd/100.500000.pcd"));
  EXPECT_TRUE(fs::exists(first_options.directory / "image/100.562500.png"));
  EXPECT_TRUE(fs::exists(second_options.directory / "pcd/100.750000.pcd"));
  EXPECT_TRUE(fs::exists(second_options.directory / "image/100.812500.png"));
  for (const auto& directory :
       {first_options.directory, second_options.directory}) {
    int clouds = 0;
    int images = 0;
    for (const auto& entry : fs::directory_iterator(directory / "pcd")) {
      if (entry.path().extension() == ".pcd") ++clouds;
    }
    for (const auto& entry : fs::directory_iterator(directory / "image")) {
      if (entry.path().extension() == ".png") ++images;
    }
    EXPECT_EQ(clouds, 1);
    EXPECT_EQ(images, 1);
  }
}

}  // namespace
}  // namespace livo_atlas
