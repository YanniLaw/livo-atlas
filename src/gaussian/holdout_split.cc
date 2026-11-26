#include "livo_atlas/gaussian/holdout_split.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <opencv2/core/persistence.hpp>
#include <stdexcept>

#include "gaussian/mapper_io.h"

namespace livo_atlas {
namespace {

constexpr std::size_t kMaxSplitBytes = 1024 * 1024;

double Number(const cv::FileNode& node) {
  if ((!node.isInt() && !node.isReal()) ||
      !std::isfinite(static_cast<double>(node))) {
    throw std::invalid_argument("Invalid frozen observation timestamp");
  }
  return static_cast<double>(node);
}

}  // namespace

GaussianHoldoutSplit GaussianHoldoutSplit::Read(
    const std::filesystem::path& path) {
  const auto size = std::filesystem::file_size(path);
  if (std::filesystem::is_symlink(path) || !size || size > kMaxSplitBytes) {
    throw std::invalid_argument("Gaussian split exceeds the manifest budget");
  }
  GaussianHoldoutSplit split;
  std::ifstream input(path, std::ios::binary);
  split.payload_.resize(size);
  if (!input.read(split.payload_.data(), size) || input.peek() != EOF) {
    throw std::invalid_argument("Could not read the frozen Gaussian split");
  }
  cv::FileStorage json(split.payload_,
                       cv::FileStorage::READ | cv::FileStorage::MEMORY);
  if (static_cast<int>(json["schema"]) != 1 ||
      static_cast<std::string>(json["kind"]) != "livo_atlas.gaussian_split") {
    throw std::invalid_argument("Unsupported Gaussian split schema");
  }
  split.split_id_ = static_cast<std::string>(json["split_id"]);
  if (split.split_id_.size() != 64 ||
      !std::all_of(split.split_id_.begin(), split.split_id_.end(), [](char ch) {
        return (ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f');
      })) {
    throw std::invalid_argument("Invalid Gaussian split digest");
  }
  const auto frames = json["frames"];
  if (!frames.isSeq() || frames.empty() || frames.size() > 512) {
    throw std::invalid_argument("Split must cover 1 to 512 frames");
  }
  for (const auto& node : frames) {
    const auto id = gaussian_mapper_io::ReadId(node["keyframe_id"]);
    const auto role = static_cast<std::string>(node["role"]);
    if (id != split.observations_.size() ||
        role != (id % 8 == 1 ? "heldout" : "train")) {
      throw std::invalid_argument("Invalid frozen Gaussian prefix or role");
    }
    split.observations_.push_back(
        {gaussian_mapper_io::ReadId(node["calibration_id"]),
         gaussian_mapper_io::ReadId(node["submap_id"]),
         Number(node["state_timestamp"]), Number(node["cloud_timestamp"]),
         Number(node["image_timestamp"]), role == "train"});
  }
  return split;
}

bool GaussianHoldoutSplit::ValidateFrame(const Keyframe& frame,
                                         std::uint64_t submap_id) const {
  if (frame.id >= observations_.size()) {
    throw std::invalid_argument("Gaussian observation is not_in_scope");
  }
  const auto& expected = observations_[frame.id];
  if (frame.calibration_id != expected.calibration_id ||
      submap_id != expected.submap_id ||
      frame.timestamp != expected.state_timestamp ||
      frame.cloud_timestamp != expected.cloud_timestamp ||
      !frame.image_timestamp ||
      *frame.image_timestamp != expected.image_timestamp) {
    throw std::invalid_argument(
        "Gaussian observation differs from frozen split");
  }
  return expected.train;
}

void GaussianHoldoutSplit::ValidateTraining(const Keyframe& frame,
                                            std::uint64_t submap_id) const {
  if (!ValidateFrame(frame, submap_id)) {
    throw std::invalid_argument("Heldout observations cannot train Gaussians");
  }
}

}  // namespace livo_atlas
