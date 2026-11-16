#include <algorithm>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "livo_atlas/io/keyframe_store.h"
#include "livo_atlas/io/reconstruction_view.h"

namespace {
namespace fs = std::filesystem;

struct Options {
  fs::path dataset;
  fs::path output;
  std::vector<std::uint64_t> train_ids;
  std::vector<std::uint64_t> validation_ids;
  std::string source_sha256;
  livo_atlas::ReconstructionViewLimits limits;
};

std::uint64_t ParseUnsigned(const std::string& text) {
  std::uint64_t value = 0;
  const auto parsed =
      std::from_chars(text.data(), text.data() + text.size(), value);
  if (text.empty() || parsed.ec != std::errc() ||
      parsed.ptr != text.data() + text.size() ||
      text != std::to_string(value)) {
    throw std::invalid_argument("Expected a canonical unsigned integer");
  }
  return value;
}

std::vector<std::uint64_t> ParseIds(const std::string& text) {
  std::vector<std::uint64_t> ids;
  std::size_t begin = 0;
  while (true) {
    const auto end = text.find(',', begin);
    ids.push_back(ParseUnsigned(text.substr(begin, end - begin)));
    if (end == std::string::npos) return ids;
    begin = end + 1;
  }
}

std::size_t ParseLimit(const std::string& text) {
  const auto value = ParseUnsigned(text);
  if (value == 0 || value > std::numeric_limits<std::size_t>::max()) {
    throw std::invalid_argument("Input limits must be positive and fit size_t");
  }
  return static_cast<std::size_t>(value);
}

Options ParseOptions(int argc, char** argv) {
  if (argc != 15) {
    throw std::invalid_argument(
        "Expected --dataset, --output, --train-ids, --validation-ids, "
        "--source-sha256, --max-pixels and --max-cloud-points");
  }
  std::map<std::string, std::string> arguments;
  for (int i = 1; i < argc; i += 2) {
    if (!arguments.emplace(argv[i], argv[i + 1]).second) {
      throw std::invalid_argument("Duplicate command-line option");
    }
  }
  const std::set<std::string> names = {"--dataset",         "--output",
                                       "--train-ids",       "--validation-ids",
                                       "--source-sha256",   "--max-pixels",
                                       "--max-cloud-points"};
  for (const auto& argument : arguments) {
    if (!names.count(argument.first)) {
      throw std::invalid_argument("Unknown command-line option");
    }
  }
  Options result;
  result.dataset = arguments.at("--dataset");
  result.output = arguments.at("--output");
  result.train_ids = ParseIds(arguments.at("--train-ids"));
  result.validation_ids = ParseIds(arguments.at("--validation-ids"));
  result.source_sha256 = arguments.at("--source-sha256");
  if (result.source_sha256.size() != 64 ||
      !std::all_of(result.source_sha256.begin(), result.source_sha256.end(),
                   [](char c) {
                     return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
                   })) {
    throw std::invalid_argument(
        "Source SHA-256 must be 64 lowercase hex digits");
  }
  result.limits.max_pixels = ParseLimit(arguments.at("--max-pixels"));
  result.limits.max_cloud_points =
      ParseLimit(arguments.at("--max-cloud-points"));
  std::set<std::uint64_t> seen;
  for (const auto id : result.train_ids) {
    if (!seen.insert(id).second)
      throw std::invalid_argument("Duplicate frame ID");
  }
  for (const auto id : result.validation_ids) {
    if (!seen.insert(id).second)
      throw std::invalid_argument("Duplicate frame ID");
  }
  return result;
}

bool Inside(const fs::path& path, const fs::path& root) {
  return std::mismatch(root.begin(), root.end(), path.begin(), path.end())
             .first == root.end();
}

std::ofstream OpenFile(const fs::path& path) {
  std::ofstream output;
  output.exceptions(std::ios::badbit | std::ios::failbit);
  output.imbue(std::locale::classic());
  output.open(path, std::ios::binary);
  output << std::setprecision(std::numeric_limits<double>::max_digits10);
  return output;
}

void WriteImage(const fs::path& path, const cv::Mat& image) {
  if (!cv::imwrite(path.string(), image)) {
    throw std::runtime_error("Cannot write image: " + path.string());
  }
}

void WriteDepth(const fs::path& path, const cv::Mat& depth) {
  if (sizeof(float) != 4 || !std::numeric_limits<float>::is_iec559) {
    throw std::runtime_error("Depth output requires IEEE-754 float32");
  }
  const auto width = static_cast<std::size_t>(depth.cols);
  if (width > std::numeric_limits<std::size_t>::max() / 4) {
    throw std::overflow_error("Depth row is too large");
  }
  auto output = OpenFile(path);
  std::vector<char> row(width * 4);
  for (int y = 0; y < depth.rows; ++y) {
    const float* values = depth.ptr<float>(y);
    for (std::size_t x = 0; x < width; ++x) {
      std::uint32_t bits = 0;
      std::memcpy(&bits, &values[x], sizeof(bits));
      for (unsigned int byte = 0; byte < 4; ++byte) {
        row[4 * x + byte] = static_cast<char>((bits >> (8 * byte)) & 0xff);
      }
    }
    output.write(row.data(), static_cast<std::streamsize>(row.size()));
  }
  output.close();
}

void WriteTransform(std::ostream& json,
                    const livo_atlas::RigidTransform& pose) {
  json << '[';
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      if (row || col) json << ',';
      if (row == 3) {
        json << (col == 3 ? 1 : 0);
      } else if (col == 3) {
        json << pose.translation[row];
      } else {
        json << pose.rotation(row, col);
      }
    }
  }
  json << ']';
}

void WriteView(std::ostream& json, const fs::path& root,
               const livo_atlas::ReconstructionView& view,
               const livo_atlas::RigidTransform& parent_from_body,
               const std::string& split) {
  const std::string relative = "views/" + std::to_string(view.keyframe_id);
  const fs::path directory = root / relative;
  if (!fs::create_directories(directory)) {
    throw std::runtime_error("Cannot create view directory");
  }
  WriteImage(directory / "image.png", view.image_bgr);
  WriteImage(directory / "image_mask.png", view.image_mask);
  WriteImage(directory / "depth_mask.png", view.depth_mask);
  WriteDepth(directory / "depth.f32le", view.depth_m);

  json << "{\"keyframe_id\":\"" << view.keyframe_id
       << "\",\"calibration_id\":\"" << view.calibration_id
       << "\",\"pose_version\":\"" << view.pose_version
       << "\",\"frame_count\":\"" << view.frame_count
       << "\",\"optimized_frame_count\":\"" << view.optimized_frame_count
       << "\",\"parent_frame\":\"odom\",\"split\":\"" << split
       << "\",\"state_timestamp\":" << view.state_timestamp
       << ",\"cloud_timestamp\":" << view.cloud_timestamp
       << ",\"image_timestamp\":" << view.image_timestamp
       << ",\"inverse_exposure_time\":" << view.inverse_exposure_time
       << ",\"width\":" << view.camera.width
       << ",\"height\":" << view.camera.height << ",\"fx\":" << view.camera.fx
       << ",\"fy\":" << view.camera.fy << ",\"cx\":" << view.camera.cx
       << ",\"cy\":" << view.camera.cy << ",\"camera_from_parent\":";
  WriteTransform(json, view.camera_from_parent);
  json << ",\"parent_from_body\":";
  WriteTransform(json, parent_from_body);
  json << ",\"image\":\"" << relative << "/image.png\",\"image_mask\":\""
       << relative << "/image_mask.png\",\"depth_mask\":\"" << relative
       << "/depth_mask.png\",\"depth_m\":\"" << relative << "/depth.f32le\"}";
}

void Prepare(const Options& options) {
  if (options.dataset.empty() || options.output.empty() ||
      fs::exists(options.output) || fs::is_symlink(options.output)) {
    throw std::invalid_argument(
        "Dataset and new output directory are required");
  }
  const auto source = fs::canonical(options.dataset);
  const auto destination = fs::weakly_canonical(fs::absolute(options.output));
  if (Inside(destination, source) || Inside(source, destination)) {
    throw std::invalid_argument("Output must be separate from the dataset");
  }
  livo_atlas::KeyframeStore store(source);
  livo_atlas::PoseSnapshot poses;
  poses.frame_count = store.ids().size();
  for (const auto id : store.ids()) {
    poses.poses.push_back({id, store.ReadRawBodyPose(id)});
  }
  livo_atlas::ValidatePoseSnapshot(poses);
  std::vector<std::pair<std::uint64_t, std::string>> selected;
  for (const auto id : options.train_ids) selected.emplace_back(id, "train");
  for (const auto id : options.validation_ids) {
    selected.emplace_back(id, "validation");
  }
  for (const auto& entry : selected) {
    if (entry.first >= poses.frame_count) {
      throw std::out_of_range("Unknown keyframe ID");
    }
  }
  if (!fs::create_directory(destination)) {
    throw std::runtime_error("Cannot create output directory");
  }
  const auto pending = destination / "bundle.json.pending";
  auto json = OpenFile(pending);
  json << "{\"schema\":1,\"source_manifest_sha256\":\"" << options.source_sha256
       << "\",\"views\":[";
  for (std::size_t i = 0; i < selected.size(); ++i) {
    const auto& [id, split] = selected[i];
    const auto frame = store.Read(id);
    const auto view = livo_atlas::MakeReconstructionView(
        store.calibration().camera, frame, poses, options.limits);
    if (i) json << ',';
    WriteView(json, destination, view, frame.odom_from_body, split);
  }
  json << "]}\n";
  json.close();
  fs::rename(pending, destination / "bundle.json");
}
}  // namespace

int main(int argc, char** argv) {
  try {
    Prepare(ParseOptions(argc, argv));
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
