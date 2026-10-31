#include "livo_atlas/io/colmap_exporter.h"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "colmap_camera.h"
#include "dataset_metadata.h"
#include "image_writer.h"
#include "livo_atlas/io/keyframe_store.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;
using dataset_metadata::Decimal;
using dataset_metadata::Fail;
using dataset_metadata::WriteTransform;
using dataset_metadata::WriteYaml;

struct FramePose {
  std::uint64_t id;
  RigidTransform parent_from_body;
};

bool Inside(const fs::path& path, const fs::path& root) {
  return std::mismatch(root.begin(), root.end(), path.begin(), path.end())
             .first == root.end();
}

std::ofstream OpenText(const fs::path& path) {
  std::ofstream output;
  output.exceptions(std::ios::badbit | std::ios::failbit);
  output.imbue(std::locale::classic());
  output.open(path, std::ios::binary);
  output << std::setprecision(std::numeric_limits<double>::max_digits10);
  return output;
}

std::vector<FramePose> SelectFrames(const KeyframeStore& keyframes,
                                    std::uint64_t version) {
  std::map<std::uint64_t, RigidTransform> poses;
  auto ids = keyframes.ids();
  if (version) {
    for (const auto& pose : keyframes.ReadPoses(version))
      poses.emplace(pose.id, pose.map_from_body);
    ids.clear();
    for (const auto& pose : poses) ids.push_back(pose.first);
  }
  std::vector<FramePose> selected;
  for (auto id : ids) {
    const auto frame = keyframes.Read(id);
    if (frame.image_timestamp)
      selected.push_back({id, version ? poses.at(id) : frame.odom_from_body});
  }
  return selected;
}

void WriteMetadata(const fs::path& path, const Calibration& calibration,
                   const ColmapExportOptions& options,
                   const ColmapExportSummary& summary) {
  WriteYaml(path, [&](auto& yaml) {
    yaml << "schema_version" << 1 << "pose_version"
         << std::to_string(options.pose_version) << "parent_frame"
         << (options.pose_version ? "map" : "odom") << "images_undistorted" << 1
         << "camera_model"
         << "PINHOLE"
         << "pixel_convention"
         << "colmap_corner"
         << "rectification"
         << "centered_pinhole"
         << "rotation_projection"
         << "nearest_so3"
         << "images" << std::to_string(summary.images) << "points"
         << std::to_string(summary.points) << "max_points_per_image"
         << std::to_string(options.max_points_per_image);
    const auto& camera = calibration.camera;
    yaml << "source_camera"
         << "{"
         << "model" << camera.model << "width" << camera.width << "height"
         << camera.height << "scale" << Decimal(camera.scale) << "fx"
         << Decimal(camera.fx) << "fy" << Decimal(camera.fy) << "cx"
         << Decimal(camera.cx) << "cy" << Decimal(camera.cy) << "distortion"
         << "[";
    for (auto value : camera.distortion) yaml << Decimal(value);
    yaml << "]"
         << "}";
    WriteTransform(yaml, "imu_from_lidar", calibration.imu_from_lidar);
    WriteTransform(yaml, "camera_from_lidar", calibration.camera_from_lidar);
  });
}
}  // namespace

ColmapExportSummary ExportColmap(const fs::path& dataset,
                                 const fs::path& output,
                                 const ColmapExportOptions& options) {
  dataset_metadata::ValidateInput(output, [&] {
    if (dataset.empty() || output.empty())
      throw std::invalid_argument("Dataset and output paths must not be empty");
    if (options.max_points_per_image == 0 ||
        options.max_points_per_image == UINT32_MAX)
      throw std::invalid_argument(
          "Export point limit must be in [1, UINT32_MAX)");
  });
  try {
    if (fs::exists(output) || fs::is_symlink(output))
      Fail(output, "Export output already exists");
    const auto source = fs::canonical(dataset);
    const auto destination = fs::weakly_canonical(fs::absolute(output));
    if (Inside(destination, source) || Inside(source, destination))
      Fail(output, "Export output must be separate from the source dataset");
    KeyframeStore keyframes(source);
    const auto frames = SelectFrames(keyframes, options.pose_version);
    if (frames.empty())
      Fail(dataset, "No visual keyframes in the selected pose version");
    if (frames.size() >= UINT32_MAX) Fail(dataset, "Too many COLMAP images");
    const ColmapCamera camera(keyframes.calibration().camera);
    if (!destination.parent_path().empty())
      fs::create_directories(destination.parent_path());
    if (!fs::create_directory(destination))
      Fail(output, "Export output already exists");
    fs::create_directory(destination / "images");
    const auto model = destination / "sparse/0";
    fs::create_directories(model);
    auto cameras = OpenText(model / "cameras.txt");
    const auto& pinhole = camera.pinhole();
    cameras
        << "# Undistorted camera, scaled once from the source calibration.\n"
        << "1 PINHOLE " << pinhole.width << ' ' << pinhole.height << ' '
        << pinhole.fx << ' ' << pinhole.fy << ' ' << pinhole.cx << ' '
        << pinhole.cy << '\n';
    cameras.close();
    auto images = OpenText(model / "images.txt");
    auto points = OpenText(model / "points3D.txt");
    auto associations = OpenText(destination / "frames.csv");
    images << "# World-to-camera Hamilton quaternion and single-view "
              "observations.\n";
    points << "# LiDAR initialization seeds; ERROR=0 is a constructed "
              "projection, not a BA statistic.\n";
    associations << "keyframe_id,image_id,image_name,timestamp,cloud_timestamp,"
                    "image_timestamp,inverse_exposure_time\n";
    ColmapExportSummary summary;
    for (const auto& selected : frames) {
      const auto frame = keyframes.Read(selected.id);
      const auto view = camera.MakeView(frame, selected.parent_from_body,
                                        options.max_points_per_image);
      if (view.samples.size() >
          std::numeric_limits<std::uint64_t>::max() - 1 - summary.points)
        Fail(output, "Too many COLMAP seed points");
      const auto image_id = ++summary.images;
      const auto name = std::to_string(frame.id) + ".png";
      const auto image_path = destination / "images" / name;
      WriteImageFile(image_path, view.image);
      Eigen::Quaterniond rotation(view.camera_from_parent.rotation);
      rotation.normalize();
      if (rotation.w() < 0) rotation.coeffs() *= -1;
      const auto& translation = view.camera_from_parent.translation;
      images << image_id << ' ' << rotation.w() << ' ' << rotation.x() << ' '
             << rotation.y() << ' ' << rotation.z() << ' ' << translation.x()
             << ' ' << translation.y() << ' ' << translation.z() << " 1 "
             << name << '\n';
      for (std::size_t index = 0; index < view.samples.size(); ++index) {
        const auto point_id = ++summary.points;
        const auto& sample = view.samples[index];
        const auto& position = sample.position;
        points << point_id << ' ' << position.x() << ' ' << position.y() << ' '
               << position.z() << ' ' << static_cast<int>(sample.rgb[0]) << ' '
               << static_cast<int>(sample.rgb[1]) << ' '
               << static_cast<int>(sample.rgb[2]) << " 0 " << image_id << ' '
               << index << '\n';
        if (index) images << ' ';
        images << sample.pixel.x() << ' ' << sample.pixel.y() << ' '
               << point_id;
      }
      images << '\n';
      associations << frame.id << ',' << image_id << ',' << name << ','
                   << frame.timestamp << ',' << frame.cloud_timestamp << ','
                   << *frame.image_timestamp << ','
                   << frame.inverse_exposure_time << '\n';
    }
    images.close();
    points.close();
    associations.close();
    if (!summary.points)
      Fail(output, "No visible LiDAR seeds in the selected visual keyframes");
    const auto pending = destination / ".pending-export.yml";
    WriteMetadata(pending, keyframes.calibration(), options, summary);
    fs::rename(pending, destination / "export.yml");
    return summary;
  } catch (const std::exception& error) {
    Fail(output, error.what());
  }
}
}  // namespace livo_atlas
