#include "livo_atlas/io/keyframe_store.h"

#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <opencv2/core/persistence.hpp>
#include <opencv2/imgcodecs.hpp>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "dataset_metadata.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;

using dataset_metadata::Decimal;
using dataset_metadata::Fail;
using dataset_metadata::Id;
using dataset_metadata::Integer;
using dataset_metadata::Number;
using dataset_metadata::ParseId;
using dataset_metadata::ReadTransform;
using dataset_metadata::ReadYaml;
using dataset_metadata::Sequence;
using dataset_metadata::String;
using dataset_metadata::ValidateInput;
using dataset_metadata::WriteTransform;
using dataset_metadata::WriteYaml;

void ValidateCalibration(const Calibration& calibration) {
  for (const auto* pose :
       {&calibration.imu_from_lidar, &calibration.camera_from_lidar}) {
    const auto error = ValidateExtrinsic(*pose);
    if (!error.empty()) throw std::invalid_argument(error);
  }
  const auto error = ValidateCameraConfig(calibration.camera);
  if (!error.empty()) throw std::invalid_argument(error);
}
void ValidateImage(const Keyframe& frame, const Calibration& calibration) {
  if (!frame.image_timestamp) return;
  const auto& camera = calibration.camera;
  if (frame.image.cols != static_cast<int>(camera.width * camera.scale) ||
      frame.image.rows != static_cast<int>(camera.height * camera.scale)) {
    throw std::invalid_argument(
        "Keyframe image must match the scaled calibration");
  }
}

void WriteCalibration(const fs::path& path, const Calibration& calibration) {
  WriteYaml(path, [&](auto& yaml) {
    yaml << "schema_version" << 1 << "calibration_id"
         << "0";
    const auto& c = calibration.camera;
    yaml << "camera"
         << "{"
         << "model" << c.model << "width" << c.width << "height" << c.height
         << "scale" << Decimal(c.scale) << "fx" << Decimal(c.fx) << "fy"
         << Decimal(c.fy) << "cx" << Decimal(c.cx) << "cy" << Decimal(c.cy)
         << "distortion"
         << "[";
    for (double value : c.distortion) yaml << Decimal(value);
    yaml << "]"
         << "}";
    WriteTransform(yaml, "imu_from_lidar", calibration.imu_from_lidar);
    WriteTransform(yaml, "camera_from_lidar", calibration.camera_from_lidar);
  });
}
Calibration ReadCalibration(const fs::path& path) {
  if (!fs::is_regular_file(path)) Fail(path, "Missing dataset metadata");
  auto yaml = ReadYaml(path);
  if (!yaml.isOpened() || Integer(yaml["schema_version"]) != 1 ||
      Id(yaml["calibration_id"]) != 0) {
    Fail(path, "Unsupported keyframe dataset schema or calibration");
  }
  Calibration calibration;
  auto& c = calibration.camera;
  const auto camera = yaml["camera"];
  c.model = String(camera["model"]);
  c.width = Integer(camera["width"]);
  c.height = Integer(camera["height"]);
  c.scale = Number(camera["scale"]);
  c.fx = Number(camera["fx"]);
  c.fy = Number(camera["fy"]);
  c.cx = Number(camera["cx"]);
  c.cy = Number(camera["cy"]);
  Sequence(camera["distortion"], 4);
  for (int i = 0; i < 4; ++i) c.distortion[i] = Number(camera["distortion"][i]);
  calibration.imu_from_lidar = ReadTransform(yaml["imu_from_lidar"]);
  calibration.camera_from_lidar = ReadTransform(yaml["camera_from_lidar"]);
  ValidateCalibration(calibration);
  return calibration;
}

void CheckPngHeader(const fs::path& path, const CameraConfig& camera) {
  std::ifstream file(path, std::ios::binary);
  std::array<unsigned char, 26> bytes{};
  file.read(reinterpret_cast<char*>(bytes.data()), bytes.size());
  constexpr std::array<unsigned char, 8> kSignature = {137, 80, 78, 71,
                                                       13,  10, 26, 10};
  const auto dimension = [&](int offset) {
    std::uint32_t value = 0;
    for (int i = 0; i < 4; ++i) value = (value << 8) | bytes[offset + i];
    return value;
  };
  if (!file ||
      !std::equal(kSignature.begin(), kSignature.end(), bytes.begin()) ||
      std::string(bytes.begin() + 12, bytes.begin() + 16) != "IHDR" ||
      dimension(16) !=
          static_cast<std::uint32_t>(camera.width * camera.scale) ||
      dimension(20) !=
          static_cast<std::uint32_t>(camera.height * camera.scale) ||
      bytes[24] != 8 || bytes[25] != 2) {
    Fail(path, "Expected an 8-bit RGB PNG matching the scaled calibration");
  }
}

// PCL 1.10 readHeader resizes PCLPointCloud2::data. Bound the binary payload
// using the actual file first, before handing a corrupt header to PCL.
void CheckPcdHeader(const fs::path& path, std::uint64_t expected_points) {
  std::ifstream file(path, std::ios::binary);
  if (!file) Fail(path, "Missing point cloud");
  std::vector<std::string> fields, sizes, types, counts;
  constexpr std::array<const char*, 10> kHeaderFields = {
      "VERSION", "FIELDS", "SIZE",      "TYPE",   "COUNT",
      "WIDTH",   "HEIGHT", "VIEWPOINT", "POINTS", "DATA"};
  std::set<std::string> seen;
  std::uint64_t width = 0, height = 0, points = 0, header_size = 0;
  bool binary = false;
  std::string line;
  char character;
  while (file.get(character)) {
    if (++header_size > 65536) Fail(path, "Point cloud header exceeds 64 KiB");
    if (character != '\n') {
      line += character;
      continue;
    }
    std::istringstream input(line);
    std::string key, value;
    input >> key;
    if (key.empty() || key.front() == '#') {
      line.clear();
      continue;
    }
    // PCL recognizes keyword prefixes. Accept only the exact schema emitted
    // by our writer so aliases cannot bypass the pre-allocation check.
    if (std::find(kHeaderFields.begin(), kHeaderFields.end(), key) ==
        kHeaderFields.end())
      Fail(path, "Unsupported PCD header keyword: " + key);
    if (!seen.insert(key).second)
      Fail(path, "Duplicate PCD header field: " + key);
    if (key == "DATA") {
      input >> value;
      binary = value == "binary";
      break;
    }
    auto* values = key == "FIELDS"  ? &fields
                   : key == "SIZE"  ? &sizes
                   : key == "TYPE"  ? &types
                   : key == "COUNT" ? &counts
                                    : nullptr;
    if (values) {
      while (input >> value) values->push_back(value);
    } else if (key == "WIDTH" || key == "HEIGHT" || key == "POINTS") {
      input >> value;
      const auto parsed = ParseId(value);
      if (key == "WIDTH") width = parsed;
      if (key == "HEIGHT") height = parsed;
      if (key == "POINTS") points = parsed;
    }
    line.clear();
  }
  constexpr auto kMax = std::numeric_limits<std::uint64_t>::max();
  if (!binary || !width || !height || width > kMax / height ||
      points != width * height || points != expected_points || fields.empty() ||
      sizes.size() != fields.size() || types.size() != fields.size() ||
      counts.size() != fields.size()) {
    Fail(path, "Invalid binary PCD shape or point count");
  }
  std::uint64_t step = 0;
  std::set<std::string> required;
  for (std::size_t i = 0; i < fields.size(); ++i) {
    const auto size = ParseId(sizes[i]), count = ParseId(counts[i]);
    if (!size || !count || size > (kMax - step) / count)
      Fail(path, "Invalid PCD field size");
    step += size * count;
    if (fields[i] == "x" || fields[i] == "y" || fields[i] == "z" ||
        fields[i] == "intensity") {
      if (size != 4 || count != 1 || types[i] != "F" ||
          !required.insert(fields[i]).second) {
        Fail(path, "Expected unique float32 XYZI fields");
      }
    }
  }
  if (required.size() != 4 || !points || step > kMax / points ||
      header_size > fs::file_size(path) ||
      step * points != fs::file_size(path) - header_size) {
    Fail(path, "PCD payload length or XYZI fields do not match metadata");
  }
}

void ValidatePoses(const std::vector<KeyframePose>& poses,
                   const std::vector<std::uint64_t>& ids) {
  if (poses.size() != ids.size() || poses.empty()) {
    throw std::invalid_argument(
        "Pose version must cover all committed keyframes");
  }
  std::set<std::uint64_t> seen;
  for (const auto& pose : poses) {
    if (!std::binary_search(ids.begin(), ids.end(), pose.id) ||
        !seen.insert(pose.id).second) {
      throw std::invalid_argument("Unknown or duplicate pose identifier");
    }
    const auto error = ValidateExtrinsic(pose.map_from_body);
    if (!error.empty()) throw std::invalid_argument(error);
  }
}
}  // namespace

KeyframeStore::KeyframeStore(const fs::path& directory,
                             const Calibration& calibration)
    : directory_(directory), calibration_(calibration), append_enabled_(true) {
  ValidateInput(directory_, [&] {
    ValidateCalibration(calibration_);
    if (directory_.empty())
      throw std::invalid_argument("Empty keyframe directory");
  });
  try {
    if (!directory_.parent_path().empty())
      fs::create_directories(directory_.parent_path());
    if (!fs::create_directory(directory_))
      Fail(directory_, "Dataset already exists");
    fs::create_directory(directory_ / "frames");
    fs::create_directory(directory_ / "poses");
    WriteCalibration(directory_ / "dataset.yml", calibration_);
  } catch (const std::exception& error) {
    Fail(directory_, error.what());
  }
}

KeyframeStore::KeyframeStore(const fs::path& directory)
    : directory_(directory) {
  try {
    calibration_ = ReadCalibration(directory_ / "dataset.yml");
    if (!fs::is_directory(directory_ / "frames") ||
        !fs::is_directory(directory_ / "poses"))
      Fail(directory_, "Missing dataset directories");
    for (const auto& entry : fs::directory_iterator(directory_ / "frames")) {
      const auto name = entry.path().filename().string();
      if (name.rfind(".pending-", 0) == 0) continue;
      const auto id = ParseId(name);
      if (!entry.is_directory() ||
          !fs::is_regular_file(entry.path() / "frame.yml")) {
        Fail(entry.path(), "Missing committed frame metadata");
      }
      ids_.push_back(id);
    }
    std::sort(ids_.begin(), ids_.end());
    for (std::size_t i = 0; i < ids_.size(); ++i) {
      if (ids_[i] != i)
        Fail(directory_, "Committed frame identifiers are not contiguous");
    }
  } catch (const std::exception& error) {
    Fail(directory_, error.what());
  }
}

void KeyframeStore::CheckWritable() const {
  if (failed_)
    throw std::logic_error(directory_.string() +
                           ": Keyframe writer previously failed");
}

void KeyframeStore::Append(const Keyframe& frame) {
  CheckWritable();
  if (!append_enabled_)
    throw std::logic_error(directory_.string() +
                           ": Reopened raw frames are read-only");
  const auto destination = directory_ / "frames" / std::to_string(frame.id);
  ValidateInput(destination, [&] {
    ValidateKeyframe(frame);
    ValidateImage(frame, calibration_);
    if (frame.id != ids_.size() || frame.calibration_id != 0) {
      throw std::invalid_argument("Unexpected frame or calibration identifier");
    }
  });
  const auto pending =
      directory_ / "frames" / (".pending-" + std::to_string(frame.id));
  if (fs::exists(destination))
    throw std::invalid_argument(destination.string() +
                                ": Frame already exists");
  try {
    if (!fs::create_directory(pending))
      Fail(pending, "Uncommitted frame already exists");
    const auto cloud_file = pending / "cloud.pcd";
    if (pcl::io::savePCDFileBinary(cloud_file.string(), *frame.cloud) != 0)
      Fail(cloud_file, "Cannot write PCD");
    if (frame.image_timestamp &&
        !cv::imwrite((pending / "image.png").string(), frame.image)) {
      Fail(pending / "image.png", "Cannot write image");
    }
    WriteYaml(pending / "frame.yml", [&](auto& yaml) {
      yaml << "id" << std::to_string(frame.id) << "calibration_id"
           << "0"
           << "timestamp" << Decimal(frame.timestamp) << "cloud_timestamp"
           << Decimal(frame.cloud_timestamp) << "cloud_points"
           << std::to_string(frame.cloud->size()) << "inverse_exposure_time"
           << Decimal(frame.inverse_exposure_time) << "has_image"
           << (frame.image_timestamp ? 1 : 0);
      WriteTransform(yaml, "odom_from_body", frame.odom_from_body);
      if (frame.image_timestamp) {
        yaml << "image_timestamp" << Decimal(*frame.image_timestamp);
        WriteTransform(yaml, "camera_from_odom", *frame.camera_from_odom);
      }
    });
    fs::rename(pending, destination);
    ids_.push_back(frame.id);
  } catch (const std::exception& error) {
    failed_ = true;
    Fail(pending, error.what());
  }
}

Keyframe KeyframeStore::Read(std::uint64_t id) const {
  if (id >= ids_.size())
    throw std::out_of_range(directory_.string() +
                            ": Unknown keyframe identifier");
  const auto path = directory_ / "frames" / std::to_string(id);
  try {
    auto yaml = ReadYaml(path / "frame.yml");
    if (!yaml.isOpened())
      Fail(path / "frame.yml", "Cannot read frame metadata");
    Keyframe frame;
    frame.id = Id(yaml["id"]);
    frame.calibration_id = Id(yaml["calibration_id"]);
    if (frame.id != id || frame.calibration_id != 0)
      Fail(path, "Frame identifier mismatch");
    frame.timestamp = Number(yaml["timestamp"]);
    frame.cloud_timestamp = Number(yaml["cloud_timestamp"]);
    frame.inverse_exposure_time = Number(yaml["inverse_exposure_time"]);
    frame.odom_from_body = ReadTransform(yaml["odom_from_body"]);
    const int has_image = Integer(yaml["has_image"]);
    if (has_image != 0 && has_image != 1) Fail(path, "Invalid image flag");
    if (has_image) {
      frame.image_timestamp = Number(yaml["image_timestamp"]);
      frame.camera_from_odom = ReadTransform(yaml["camera_from_odom"]);
      CheckPngHeader(path / "image.png", calibration_.camera);
      frame.image =
          cv::imread((path / "image.png").string(), cv::IMREAD_UNCHANGED);
    }
    const auto points = Id(yaml["cloud_points"]);
    const auto cloud_file = path / "cloud.pcd";
    CheckPcdHeader(cloud_file, points);
    pcl::PCLPointCloud2 header;
    pcl::PCDReader reader;
    if (reader.readHeader(cloud_file.string(), header) < 0 ||
        std::uint64_t{header.width} * header.height != points)
      Fail(cloud_file, "Invalid PCD header");
    PointCloud::Ptr cloud(new PointCloud);
    if (reader.read(cloud_file.string(), *cloud) < 0 || cloud->size() != points)
      Fail(cloud_file, "Cannot read PCD");
    frame.cloud = cloud;
    ValidateKeyframe(frame);
    ValidateImage(frame, calibration_);
    return frame;
  } catch (const std::exception& error) {
    Fail(path, error.what());
  }
}

void KeyframeStore::WritePoses(std::uint64_t version,
                               const std::vector<KeyframePose>& poses) {
  CheckWritable();
  const auto path = directory_ / "poses" / (std::to_string(version) + ".yml");
  ValidateInput(path, [&] {
    if (!version)
      throw std::invalid_argument("Pose version zero is the raw frontend pose");
    ValidatePoses(poses, ids_);
  });
  const auto pending =
      directory_ / "poses" / (".pending-" + std::to_string(version) + ".yml");
  if (fs::exists(path))
    throw std::invalid_argument(path.string() +
                                ": Pose version already exists");
  try {
    if (fs::exists(pending))
      Fail(pending, "Uncommitted pose version already exists");
    WriteYaml(pending, [&](auto& yaml) {
      yaml << "version" << std::to_string(version) << "frame_count"
           << std::to_string(ids_.size()) << "poses"
           << "[";
      for (const auto& pose : poses) {
        yaml << "{"
             << "id" << std::to_string(pose.id);
        WriteTransform(yaml, "map_from_body", pose.map_from_body);
        yaml << "}";
      }
      yaml << "]";
    });
    fs::rename(pending, path);
  } catch (const std::exception& error) {
    failed_ = true;
    Fail(path, error.what());
  }
}

std::vector<KeyframePose> KeyframeStore::ReadPoses(
    std::uint64_t version) const {
  const auto path = directory_ / "poses" / (std::to_string(version) + ".yml");
  if (!version)
    throw std::invalid_argument(path.string() +
                                ": Pose version zero is the raw frontend pose");
  try {
    if (!fs::is_regular_file(path)) Fail(path, "Missing pose version");
    auto yaml = ReadYaml(path);
    if (!yaml.isOpened() || Id(yaml["version"]) != version)
      Fail(path, "Pose version mismatch");
    const auto nodes = yaml["poses"];
    const auto count = Id(yaml["frame_count"]);
    if (!count || count > ids_.size())
      Fail(path, "Unknown pose version frame frontier");
    Sequence(nodes, count);
    std::vector<KeyframePose> poses;
    for (const auto& node : nodes)
      poses.push_back({Id(node["id"]), ReadTransform(node["map_from_body"])});
    ValidatePoses(
        poses, std::vector<std::uint64_t>(ids_.begin(), ids_.begin() + count));
    return poses;
  } catch (const std::exception& error) {
    Fail(path, error.what());
  }
}
}  // namespace livo_atlas
