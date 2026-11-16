#include "gaussian/mapper_io.h"

#include <cstring>
#include <fstream>
#include <stdexcept>

namespace livo_atlas {
namespace gaussian_mapper_io {
namespace fs = std::filesystem;

std::string Json(const std::function<void(cv::FileStorage&)>& write) {
  cv::FileStorage json(".json", cv::FileStorage::WRITE |
                                    cv::FileStorage::MEMORY |
                                    cv::FileStorage::FORMAT_JSON);
  write(json);
  return json.releaseAndGetString();
}

void Base(cv::FileStorage& json, const std::string& kind,
          const std::string& session, int schema) {
  json << "schema" << schema << "type" << kind << "session_id" << session;
}

void Id(cv::FileStorage& json, const char* key, std::uint64_t value) {
  json << key << std::to_string(value);
}

std::uint64_t ReadId(const cv::FileNode& node) {
  return std::stoull(static_cast<std::string>(node));
}

void Transform(cv::FileStorage& json, const char* key,
               const RigidTransform& t) {
  json << key << "[";
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      const double value =
          row == 3 ? (col == 3 ? 1.0 : 0.0)
                   : col == 3 ? t.translation[row] : t.rotation(row, col);
      json << value;
    }
  }
  json << "]";
}

cv::FileStorage Parse(const GaussianMessage& message, const std::string& type,
                      const std::string& session) {
  cv::FileStorage json(message.header_json, cv::FileStorage::READ |
                                                cv::FileStorage::MEMORY |
                                                cv::FileStorage::FORMAT_JSON);
  if (static_cast<std::string>(json["session_id"]) != session)
    throw std::runtime_error("Gaussian worker session mismatch");
  const auto actual = static_cast<std::string>(json["type"]);
  if (actual == "error")
    throw std::runtime_error("Gaussian worker error: " +
                             static_cast<std::string>(json["code"]));
  if (actual != type)
    throw std::runtime_error("Unexpected Gaussian worker message");
  return json;
}

void WriteFile(const fs::path& path, const std::string& data) {
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  file.exceptions(std::ios::failbit | std::ios::badbit);
  file.write(data.data(), static_cast<std::streamsize>(data.size()));
  file.close();
}

void CheckIdentity(const cv::FileStorage& json,
                   const GaussianPreview& preview) {
  if (ReadId(json["job_id"]) != preview.job_id ||
      ReadId(json["generation"]) != preview.generation ||
      ReadId(json["submap_id"]) != preview.submap_id ||
      ReadId(json["pose_version"]) != preview.pose_version)
    throw std::runtime_error("Gaussian worker job identity mismatch");
}

void PreviewIdentity(cv::FileStorage& json, const GaussianPreview& preview) {
  Id(json, "job_id", preview.job_id);
  Id(json, "generation", preview.generation);
  Id(json, "submap_id", preview.submap_id);
  Id(json, "pose_version", preview.pose_version);
}

std::vector<std::uint8_t> ViewBody(const ReconstructionView& view) {
  const auto pixels =
      static_cast<std::size_t>(view.image_bgr.cols) * view.image_bgr.rows;
  std::vector<std::uint8_t> body(pixels * 9);
  for (int y = 0; y < view.image_bgr.rows; ++y) {
    const auto row = static_cast<std::size_t>(y) * view.image_bgr.cols;
    std::memcpy(body.data() + 3 * row, view.image_bgr.ptr(y),
                3 * view.image_bgr.cols);
    std::memcpy(body.data() + 3 * pixels + row, view.image_mask.ptr(y),
                view.image_bgr.cols);
    std::memcpy(body.data() + 4 * pixels + row, view.depth_mask.ptr(y),
                view.image_bgr.cols);
    for (int x = 0; x < view.image_bgr.cols; ++x) {
      std::uint32_t bits;
      const float depth = view.depth_m.at<float>(y, x);
      static_assert(sizeof(bits) == sizeof(depth), "float must use 32 bits");
      std::memcpy(&bits, &depth, sizeof(bits));
      const auto index = 5 * pixels + 4 * (row + x);
      for (int byte = 0; byte < 4; ++byte)
        body[index + byte] = static_cast<std::uint8_t>(bits >> (8 * byte));
    }
  }
  return body;
}

}  // namespace gaussian_mapper_io
}  // namespace livo_atlas
