#include <cmath>
#include <filesystem>
#include <fstream>
#include <set>
#include <stdexcept>
#include <string>

#include "gaussian/mapper_io.h"
#include "gaussian/render_metadata_protocol.h"
#include "livo_atlas/common/logging.h"
#include "livo_atlas/gaussian/gaussian_projection.h"
#include "mapping/pose_math.h"

namespace {

void Require(bool condition, const char* message) {
  if (!condition) throw std::invalid_argument(message);
}

void Keys(const cv::FileNode& node, const std::set<std::string>& fields) {
  Require(node.isMap() && node.size() == fields.size(),
          "Invalid selector fields");
  for (const auto& field : node)
    Require(fields.count(field.name()), "Unknown selector field");
}

std::uint64_t Id(const cv::FileNode& node) {
  const auto result = livo_atlas::gaussian_mapper_io::ReadId(node);
  Require(static_cast<std::string>(node) == std::to_string(result),
          "Invalid canonical ID");
  return result;
}

livo_atlas::RigidTransform Pose(const cv::FileNode& node) {
  Require(node.isSeq() && node.size() == 16, "Invalid selector pose");
  livo_atlas::RigidTransform pose;
  for (int i = 0; i < 16; ++i) {
    Require(node[i].isInt() || node[i].isReal(), "Invalid pose number");
    const double value = node[i];
    Require(std::isfinite(value), "Nonfinite selector pose");
    if (i / 4 == 3)
      Require(std::abs(value - (i == 15 ? 1 : 0)) <= 1e-9,
              "Invalid homogeneous row");
    else if (i % 4 == 3)
      pose.translation[i / 4] = value;
    else
      pose.rotation(i / 4, i % 4) = value;
  }
  livo_atlas::pose_math::Validate(pose);
  return pose;
}

}  // namespace

int main(int argc, char** argv) {
  livo_atlas::InitializeLogging(argv[0]);
  if (argc != 3) {
    LOG(ERROR)
        << "Usage: livo_atlas_select_gaussian_neighbor INPUT_JSON OUTPUT_JSON";
    return 2;
  }
  try {
    const std::filesystem::path input(argv[1]), output(argv[2]);
    Require(std::filesystem::is_regular_file(input) &&
                std::filesystem::file_size(input) <= 512 * 1024 &&
                !std::filesystem::exists(output),
            "Invalid selector input/output");
    std::ifstream stream(input, std::ios::binary);
    std::string raw(512 * 1024 + 1, '\0');
    stream.read(raw.data(), raw.size());
    raw.resize(stream.gcount());
    Require(!stream.bad() && !raw.empty() && raw.size() <= 512 * 1024,
            "Selector input exceeds budget");
    cv::FileStorage document(raw,
                             cv::FileStorage::READ | cv::FileStorage::MEMORY);
    const auto root = document.root();
    Keys(root, {"schema", "camera", "parent_from_body", "owner", "candidates"});
    Require(root["schema"].isInt() && static_cast<int>(root["schema"]) == 1,
            "Invalid selector schema");
    const auto camera = livo_atlas::DecodeGaussianCamera(root["camera"]);
    const auto candidates = root["candidates"];
    Require(candidates.isSeq() && candidates.size() <= 32,
            "Too many candidates");
    std::vector<livo_atlas::GaussianSpatialCandidate> sources;
    for (const auto& item : candidates) {
      Keys(item, {"metadata", "parent_from_submap"});
      const auto node = item["metadata"];
      const auto source = node["content"];
      const livo_atlas::GaussianContentId content{
          static_cast<std::string>(source["session_id"]),
          Id(source["submap_id"]), Id(source["source_generation"]),
          Id(source["content_job_id"])};
      std::vector<std::uint64_t> frames;
      Require(node["extents"].isSeq() && node["extents"].size() <= 10,
              "Invalid extent budget");
      for (const auto& extent : node["extents"])
        frames.push_back(Id(extent["origin_keyframe_id"]));
      const auto metadata =
          livo_atlas::DecodeGaussianRenderMetadata(node, content, frames);
      sources.push_back({content.submap_id, metadata.extents,
                         Pose(item["parent_from_submap"])});
    }
    const auto result = livo_atlas::SelectGaussianSpatialNeighbor(
        sources, camera, Pose(root["parent_from_body"]), Id(root["owner"]));
    const auto json = livo_atlas::gaussian_mapper_io::Json([&](auto& out) {
      out << "schema" << 1 << "selected_id"
          << (result.submap_id ? std::to_string(*result.submap_id) : "")
          << "candidate_count" << static_cast<int>(result.candidate_count)
          << "candidate_pixels"
          << static_cast<int>(result.projection.candidate_pixels)
          << "projection_uncertain"
          << static_cast<int>(result.projection.uncertain);
    });
    livo_atlas::gaussian_mapper_io::WriteFile(output, json);
    return 0;
  } catch (const std::exception& error) {
    LOG(ERROR) << error.what();
    return 1;
  }
}
