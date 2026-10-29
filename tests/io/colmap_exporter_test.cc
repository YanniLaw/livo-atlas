#include "livo_atlas/io/colmap_exporter.h"

#include <gtest/gtest.h>

#if defined(__unix__)
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

#include <chrono>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <opencv2/imgcodecs.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include "livo_atlas/io/keyframe_store.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;

class ScopedDirectory {
 public:
  explicit ScopedDirectory(const fs::path& path)
      : previous_(fs::current_path()) {
    fs::current_path(path);
  }
  ~ScopedDirectory() {
    std::error_code ignored;
    fs::current_path(previous_, ignored);
  }

 private:
  fs::path previous_;
};

std::string Bytes(const fs::path& path) {
  std::ifstream file(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(file), {}};
}
std::map<std::string, std::string> Snapshot(const fs::path& root) {
  std::map<std::string, std::string> files;
  for (const auto& entry : fs::recursive_directory_iterator(root)) {
    if (entry.is_regular_file())
      files.emplace(entry.path().lexically_relative(root).string(),
                    Bytes(entry.path()));
  }
  return files;
}
std::vector<std::string> Lines(const fs::path& path) {
  std::ifstream input(path);
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line.front() != '#') lines.push_back(line);
  }
  return lines;
}
std::vector<std::string> Split(const std::string& row, char delimiter) {
  std::istringstream input(row);
  std::vector<std::string> fields;
  std::string value;
  while (std::getline(input, value, delimiter)) fields.push_back(value);
  return fields;
}
Eigen::Matrix3d Rotation() {
  Eigen::Matrix3d rotation;
  rotation << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  return rotation;
}
struct ModelImage {
  std::uint32_t id = 0, camera = 0;
  Eigen::Quaterniond rotation;
  Eigen::Vector3d translation;
  std::string name;
};
ModelImage ParseImage(const std::string& row) {
  std::istringstream input(row);
  ModelImage image;
  double w, x, y, z;
  input >> image.id >> w >> x >> y >> z >> image.translation.x() >>
      image.translation.y() >> image.translation.z() >> image.camera >>
      image.name;
  if (!input) throw std::runtime_error("Incomplete model image");
  image.rotation = Eigen::Quaterniond(w, x, y, z);
  return image;
}

class ColmapExporterTest : public testing::Test {
 protected:
  void SetUp() override {
    root_ = fs::temp_directory_path() /
            ("colmap-export-" +
             std::to_string(
                 std::chrono::steady_clock::now().time_since_epoch().count()));
    ASSERT_TRUE(fs::create_directory(root_));
    dataset_ = root_ / "dataset";
    output_ = root_ / "export";
    calibration_.camera = {"Pinhole", 12, 8, .5, 8, 10, 5, 3, {0, 0, 0, 0}};
    calibration_.imu_from_lidar.translation << 1, 0, 0;
    calibration_.camera_from_lidar = {Rotation(), Eigen::Vector3d(0, 1, 1)};
  }
  void TearDown() override { fs::remove_all(root_); }
  Keyframe Frame(std::uint64_t id, bool visual = true) {
    Keyframe frame;
    frame.id = id;
    frame.timestamp = 1000000000.1234567 + id * .25;
    frame.cloud_timestamp = frame.timestamp - .03125;
    frame.inverse_exposure_time = .8;
    frame.odom_from_body = {Rotation(), Eigen::Vector3d(1, 2, 3)};
    if (visual) {
      frame.image_timestamp = frame.timestamp + .0625;
      frame.camera_from_odom = RigidTransform{Eigen::Matrix3d::Identity(),
                                              Eigen::Vector3d(-1, -2, -2)};
      frame.image = cv::Mat(4, 6, CV_8UC3);
      for (int y = 0; y < 4; ++y) {
        for (int x = 0; x < 6; ++x)
          frame.image.at<cv::Vec3b>(y, x) = {
              static_cast<unsigned char>(10 + x),
              static_cast<unsigned char>(20 + y),
              static_cast<unsigned char>(30 + x + y)};
      }
    }
    PointCloud::Ptr cloud(new PointCloud);
    for (float x : {.5f, -.5f}) {
      Point point{};
      point.x = x;
      point.z = 3;
      point.intensity = 7;
      cloud->push_back(point);
    }
    frame.cloud = cloud;
    return frame;
  }
  fs::path root_, dataset_, output_;
  Calibration calibration_;
};

TEST_F(ColmapExporterTest,
       WritesLiteralCalibrationPosesPointsAndBidirectionalTracks) {
  KeyframeStore writer(dataset_, calibration_);
  const auto frame = Frame(0);
  writer.Append(frame);
  const auto original = Snapshot(dataset_);
  const auto summary = ExportColmap(dataset_, output_);
  EXPECT_EQ(summary.images, 1u);
  EXPECT_EQ(summary.points, 2u);
  const auto model = output_ / "sparse/0";
  ASSERT_EQ(Lines(model / "cameras.txt"),
            (std::vector<std::string>{"1 PINHOLE 6 4 4 5 3 2"}));
  const auto images = Lines(model / "images.txt");
  ASSERT_EQ(images.size(), 2u);
  const auto image = ParseImage(images[0]);
  EXPECT_EQ(image.id, 1u);
  EXPECT_EQ(image.camera, 1u);
  EXPECT_EQ(image.name, "0.png");
  EXPECT_TRUE(
      image.rotation.toRotationMatrix().isApprox(Eigen::Matrix3d::Identity()));
  EXPECT_TRUE(image.translation.isApprox(Eigen::Vector3d(-1, -2, -2)));
  const auto points = Lines(model / "points3D.txt");
  ASSERT_EQ(points.size(), 2u);
  for (std::size_t index = 0; index < 2; ++index) {
    std::istringstream row(points[index]);
    std::uint64_t id;
    double x, y, z, error;
    int red, green, blue;
    std::uint32_t image_id, point_index;
    row >> id >> x >> y >> z >> red >> green >> blue >> error >> image_id >>
        point_index;
    ASSERT_TRUE(row);
    EXPECT_EQ(id, index + 1);
    EXPECT_EQ(x, 1);
    EXPECT_EQ(y, index == 0 ? 1.5 : 2.5);
    EXPECT_EQ(z, 6);
    EXPECT_EQ(red, index == 0 ? 34 : 35);
    EXPECT_EQ(green, index == 0 ? 21 : 22);
    EXPECT_EQ(blue, 13);
    EXPECT_EQ(error, 0);
    EXPECT_EQ(image_id, 1u);
    EXPECT_EQ(point_index, index);
  }
  std::istringstream observations(images[1]);
  double u, v;
  std::uint64_t point;
  observations >> u >> v >> point;
  EXPECT_EQ(u, 3);
  EXPECT_EQ(v, 1.375);
  EXPECT_EQ(point, 1u);
  observations >> u >> v >> point;
  EXPECT_EQ(u, 3);
  EXPECT_EQ(v, 2.625);
  EXPECT_EQ(point, 2u);
  const auto decoded = cv::imread((output_ / "images/0.png").string());
  EXPECT_EQ(cv::norm(decoded, frame.image, cv::NORM_INF), 0);
  cv::FileStorage metadata((output_ / "export.yml").string(),
                           cv::FileStorage::READ);
  ASSERT_TRUE(metadata.isOpened());
  EXPECT_EQ(static_cast<int>(metadata["schema_version"]), 1);
  EXPECT_EQ(static_cast<std::string>(metadata["pose_version"]), "0");
  EXPECT_EQ(static_cast<std::string>(metadata["parent_frame"]), "odom");
  EXPECT_EQ(static_cast<std::string>(metadata["source_camera"]["fx"]), "8");
  EXPECT_EQ(static_cast<std::string>(metadata["source_camera"]["scale"]),
            "0.5");
  EXPECT_EQ(
      static_cast<std::string>(metadata["imu_from_lidar"]["translation"][0]),
      "1");
  const auto rows = Lines(output_ / "frames.csv");
  ASSERT_EQ(rows.size(), 2u);
  const auto association = Split(rows[1], ',');
  ASSERT_EQ(association.size(), 7u);
  EXPECT_EQ(association[0], "0");
  EXPECT_EQ(association[1], "1");
  EXPECT_EQ(association[2], "0.png");
  EXPECT_EQ(std::stod(association[3]), frame.timestamp);
  EXPECT_EQ(std::stod(association[4]), frame.cloud_timestamp);
  EXPECT_EQ(std::stod(association[5]), *frame.image_timestamp);
  EXPECT_EQ(std::stod(association[6]), .8);
  EXPECT_EQ(Snapshot(dataset_), original);
}

TEST_F(ColmapExporterTest,
       AppliesUnorderedNonrigidPoseUpdatesWithoutChangingImages) {
  KeyframeStore writer(dataset_, calibration_);
  writer.Append(Frame(0));
  auto second = Frame(1);
  second.odom_from_body = {Eigen::Matrix3d::Identity(),
                           Eigen::Vector3d(-2, 1, 0)};
  second.camera_from_odom =
      RigidTransform{Rotation(), Eigen::Vector3d(1, 2, 1)};
  writer.Append(second);
  RigidTransform first_pose, second_pose{Rotation(), Eigen::Vector3d(20, 0, 0)};
  first_pose.translation << 10, 0, 0;
  writer.WritePoses(1, {{1, second_pose}, {0, first_pose}});
  const auto original = Snapshot(dataset_);
  const auto raw = ExportColmap(dataset_, output_);
  const auto optimized_output = root_ / "optimized";
  const auto optimized = ExportColmap(dataset_, optimized_output, {1, 2000});
  EXPECT_EQ(raw.images, 2u);
  EXPECT_EQ(optimized.points, raw.points);
  const auto images = Lines(optimized_output / "sparse/0/images.txt");
  ASSERT_EQ(images.size(), 4u);
  const auto first = ParseImage(images[0]), last = ParseImage(images[2]);
  EXPECT_TRUE(first.rotation.toRotationMatrix().isApprox(Rotation()));
  EXPECT_TRUE(first.translation.isApprox(Eigen::Vector3d(0, -10, 1)));
  EXPECT_TRUE(
      last.rotation.toRotationMatrix().isApprox(Eigen::Matrix3d::Identity()));
  EXPECT_TRUE(last.translation.isApprox(Eigen::Vector3d(-20, 0, 1)));
  const auto points = Lines(optimized_output / "sparse/0/points3D.txt");
  ASSERT_EQ(points.size(), 4u);
  const std::vector<Eigen::Vector3d> expected = {
      {9.5, 0, 3}, {10.5, 0, 3}, {20, -.5, 3}, {20, .5, 3}};
  for (std::size_t i = 0; i < points.size(); ++i) {
    std::istringstream row(points[i]);
    std::uint64_t id;
    Eigen::Vector3d point;
    row >> id >> point.x() >> point.y() >> point.z();
    EXPECT_TRUE(point.isApprox(expected[i]));
  }
  for (const auto* name : {"0.png", "1.png"})
    EXPECT_EQ(Bytes(output_ / "images" / name),
              Bytes(optimized_output / "images" / name));
  EXPECT_EQ(Bytes(output_ / "frames.csv"),
            Bytes(optimized_output / "frames.csv"));
  EXPECT_EQ(Snapshot(dataset_), original);
}

TEST_F(ColmapExporterTest,
       ReindexesImagesWhilePreservingNonvisualGapsAndSourceIds) {
  KeyframeStore writer(dataset_, calibration_);
  writer.Append(Frame(0));
  writer.Append(Frame(1, false));
  writer.Append(Frame(2));
  const auto result = ExportColmap(dataset_, output_);
  EXPECT_EQ(result.images, 2u);
  EXPECT_EQ(result.points, 4u);
  const auto images = Lines(output_ / "sparse/0/images.txt");
  ASSERT_EQ(images.size(), 4u);
  EXPECT_EQ(ParseImage(images[0]).id, 1u);
  EXPECT_EQ(ParseImage(images[0]).name, "0.png");
  EXPECT_EQ(ParseImage(images[2]).id, 2u);
  EXPECT_EQ(ParseImage(images[2]).name, "2.png");
  EXPECT_FALSE(fs::exists(output_ / "images/1.png"));
}

TEST_F(ColmapExporterTest,
       UsesTheFrozenPosePrefixWithoutReadingLaterFrameAssets) {
  KeyframeStore writer(dataset_, calibration_);
  writer.Append(Frame(0));
  writer.Append(Frame(1, false));
  writer.WritePoses(1, {{1, {}}, {0, {}}});
  writer.Append(Frame(2));
  ASSERT_TRUE(fs::remove(dataset_ / "frames/2/cloud.pcd"));
  EXPECT_EQ(ExportColmap(dataset_, output_, {1, 2000}).images, 1u);
  EXPECT_FALSE(fs::exists(output_ / "images/2.png"));
  EXPECT_THROW(ExportColmap(dataset_, root_ / "raw"), std::runtime_error);
  EXPECT_FALSE(fs::exists(root_ / "raw"));
}

TEST_F(ColmapExporterTest, RejectsInvalidLimitsBeforeCreatingAnyOutput) {
  KeyframeStore writer(dataset_, calibration_);
  writer.Append(Frame(0));
  EXPECT_THROW(ExportColmap(dataset_, output_, {0, 0}), std::invalid_argument);
  EXPECT_THROW(ExportColmap(dataset_, output_, {0, UINT32_MAX}),
               std::invalid_argument);
  EXPECT_FALSE(fs::exists(output_));
}

TEST_F(ColmapExporterTest,
       RejectsEmptyNonvisualAndMissingVersionsBeforeOutput) {
  KeyframeStore writer(dataset_, calibration_);
  EXPECT_THROW(ExportColmap(dataset_, output_), std::runtime_error);
  EXPECT_FALSE(fs::exists(output_));
  try {
    ExportColmap(dataset_, output_, {99, 2000});
    FAIL() << "Explicit missing version must be checked on an empty dataset";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string(error.what()).find("99"), std::string::npos);
  }
  writer.Append(Frame(0, false));
  EXPECT_THROW(ExportColmap(dataset_, output_), std::runtime_error);
  EXPECT_FALSE(fs::exists(output_));
}

TEST_F(ColmapExporterTest, PreflightsAllSelectedAssetsBeforeCreatingTheOutput) {
  KeyframeStore writer(dataset_, calibration_);
  writer.Append(Frame(0));
  writer.Append(Frame(1));
  ASSERT_TRUE(fs::remove(dataset_ / "frames/1/image.png"));
  EXPECT_THROW(ExportColmap(dataset_, output_), std::runtime_error);
  EXPECT_FALSE(fs::exists(output_));
}

TEST_F(ColmapExporterTest, RefusesExistingFilesDirectoriesAndSymbolicLinks) {
  KeyframeStore writer(dataset_, calibration_);
  writer.Append(Frame(0));
  ASSERT_TRUE(fs::create_directory(output_));
  EXPECT_THROW(ExportColmap(dataset_, output_), std::runtime_error);
  std::ofstream(output_ / "sentinel") << "keep this";
  EXPECT_THROW(ExportColmap(dataset_, output_), std::runtime_error);
  EXPECT_EQ(Bytes(output_ / "sentinel"), "keep this");
  const auto file = root_ / "file";
  std::ofstream(file) << "keep file";
  EXPECT_THROW(ExportColmap(dataset_, file), std::runtime_error);
  EXPECT_EQ(Bytes(file), "keep file");
#if defined(__unix__)
  const auto link = root_ / "link";
  fs::create_directory_symlink(output_, link);
  EXPECT_THROW(ExportColmap(dataset_, link), std::runtime_error);
  const auto dangling = root_ / "dangling";
  fs::create_symlink(root_ / "not-created", dangling);
  EXPECT_THROW(ExportColmap(dataset_, dangling), std::runtime_error);
  EXPECT_TRUE(fs::is_symlink(dangling));
#endif
}

TEST_F(ColmapExporterTest,
       RefusesNormalizedAndAliasedPathsInsideTheSourceDataset) {
  KeyframeStore writer(dataset_, calibration_);
  writer.Append(Frame(0));
  const auto original = Snapshot(dataset_);
  EXPECT_THROW(ExportColmap(dataset_, dataset_ / "export"), std::exception);
  EXPECT_THROW(ExportColmap(dataset_, dataset_ / "frames/../export"),
               std::exception);
  {
    ScopedDirectory directory(dataset_);
    EXPECT_THROW(ExportColmap(".", "relative-output"), std::exception);
  }
#if defined(__unix__)
  const auto alias = root_ / "alias";
  fs::create_directory_symlink(dataset_, alias);
  EXPECT_THROW(ExportColmap(dataset_, alias / "export"), std::exception);
#endif
  EXPECT_EQ(Snapshot(dataset_), original);
}

TEST_F(ColmapExporterTest, HasNoCompletionMarkerWhenAllLidarSeedsAreInvisible) {
  KeyframeStore writer(dataset_, calibration_);
  auto frame = Frame(0);
  PointCloud::Ptr cloud(new PointCloud(*frame.cloud));
  for (auto& point : *cloud) point.z = -2;
  frame.cloud = cloud;
  writer.Append(frame);
  const auto original = Snapshot(dataset_);
  EXPECT_THROW(ExportColmap(dataset_, output_), std::runtime_error);
  EXPECT_TRUE(fs::is_directory(output_));
  EXPECT_FALSE(fs::exists(output_ / "export.yml"));
  EXPECT_THROW(ExportColmap(dataset_, output_), std::runtime_error);
  EXPECT_EQ(Snapshot(dataset_), original);
}

TEST_F(ColmapExporterTest, KeepsAnEmptyObservationLineIfAnotherImageHasSeeds) {
  KeyframeStore writer(dataset_, calibration_);
  auto frame = Frame(0);
  PointCloud::Ptr cloud(new PointCloud(*frame.cloud));
  for (auto& point : *cloud) point.z = -2;
  frame.cloud = cloud;
  writer.Append(frame);
  writer.Append(Frame(1));
  const auto result = ExportColmap(dataset_, output_);
  EXPECT_EQ(result.images, 2u);
  EXPECT_EQ(result.points, 2u);
  const auto images = Lines(output_ / "sparse/0/images.txt");
  ASSERT_EQ(images.size(), 4u);
  EXPECT_TRUE(images[1].empty());
  EXPECT_FALSE(images[3].empty());
  const auto points = Lines(output_ / "sparse/0/points3D.txt");
  ASSERT_EQ(points.size(), 2u);
  EXPECT_NE(points[0].find(" 2 0"), std::string::npos);
}

TEST_F(ColmapExporterTest,
       CapsEachImageWithoutBreakingTheObservationAssociations) {
  KeyframeStore writer(dataset_, calibration_);
  writer.Append(Frame(0));
  const auto result = ExportColmap(dataset_, output_, {0, 1});
  EXPECT_EQ(result.points, 1u);
  const auto points = Lines(output_ / "sparse/0/points3D.txt");
  ASSERT_EQ(points.size(), 1u);
  EXPECT_EQ(Lines(output_ / "sparse/0/images.txt")[1], "3 1.375 1");
}

TEST_F(ColmapExporterTest,
       RealWriteFailurePreservesTheInputAndHasNoCompletionMarker) {
#if defined(__unix__)
  KeyframeStore writer(dataset_, calibration_);
  writer.Append(Frame(0));
  const auto original = Snapshot(dataset_);
  cv::setNumThreads(1);
  const auto child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    std::signal(SIGXFSZ, SIG_IGN);
    const rlimit limit{0, 0};
    if (setrlimit(RLIMIT_FSIZE, &limit) != 0) _exit(3);
    try {
      ExportColmap(dataset_, output_);
    } catch (const std::runtime_error&) {
      _exit(0);
    }
    _exit(2);
  }
  int status = 0;
  ASSERT_EQ(waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);
  EXPECT_TRUE(fs::is_directory(output_));
  EXPECT_FALSE(fs::exists(output_ / "export.yml"));
  EXPECT_THROW(ExportColmap(dataset_, output_), std::runtime_error);
  EXPECT_EQ(Snapshot(dataset_), original);
  EXPECT_EQ(KeyframeStore(dataset_).Read(0).id, 0u);
#else
  GTEST_SKIP() << "RLIMIT_FSIZE fault injection requires Unix";
#endif
}
}  // namespace
}  // namespace livo_atlas
