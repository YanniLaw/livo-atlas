#include "gaussian/render_metadata_protocol.h"

#include <charconv>
#include <cmath>
#include <set>
#include <stdexcept>
#include <string>
#include <system_error>

#include "gaussian/mapper_io.h"

namespace livo_atlas {
namespace {

void Require(bool condition, const char* detail) {
  if (!condition) throw std::invalid_argument(detail);
}

void Keys(const cv::FileNode& node, const std::set<std::string>& fields) {
  Require(node.isMap() && node.size() == fields.size(),
          "Invalid spatial metadata fields");
  for (const auto& child : node)
    Require(fields.count(child.name()) != 0, "Unknown spatial metadata key");
}

std::uint64_t Id(const cv::FileNode& node) {
  Require(node.isString(), "Spatial metadata ID must be a string");
  const std::string text = node;
  std::uint64_t result = 0;
  const auto parsed =
      std::from_chars(text.data(), text.data() + text.size(), result);
  Require(parsed.ec == std::errc{} && parsed.ptr == text.data() + text.size() &&
              text == std::to_string(result),
          "Invalid spatial uint64");
  return result;
}

double Number(const cv::FileNode& node) {
  Require(node.isReal() || node.isInt(), "Expected a spatial number");
  const double value = node;
  Require(std::isfinite(value), "Nonfinite spatial metadata");
  return value;
}

Eigen::Vector3d Vector(const cv::FileNode& node) {
  Require(node.isSeq() && node.size() == 3, "Invalid spatial bound shape");
  return {Number(node[0]), Number(node[1]), Number(node[2])};
}

RigidTransform Pose(const cv::FileNode& node) {
  Require(node.isSeq() && node.size() == 16, "Invalid spatial camera pose");
  RigidTransform result;
  for (int i = 0; i < 16; ++i) {
    const double value = Number(node[i]);
    if (i / 4 == 3)
      Require(std::abs(value - (i == 15 ? 1 : 0)) <= 1e-9,
              "Invalid spatial pose last row");
    else if (i % 4 == 3)
      result.translation[i / 4] = value;
    else
      result.rotation(i / 4, i % 4) = value;
  }
  return result;
}

int Dimension(const cv::FileNode& node) {
  Require(node.isInt(), "Expected a spatial dimension");
  const int value = node;
  Require(value > 0 && value <= 1000000, "Spatial dimension exceeds budget");
  return value;
}

}  // namespace

GaussianCamera DecodeGaussianCamera(const cv::FileNode& camera) {
  GaussianCamera result;
  Keys(camera, {"keyframe_id", "calibration_id", "width", "height", "K",
                "camera_from_body", "capture_time"});
  result.keyframe_id = Id(camera["keyframe_id"]);
  result.calibration_id = Id(camera["calibration_id"]);
  result.width = Dimension(camera["width"]);
  result.height = Dimension(camera["height"]);
  Require(camera["K"].isSeq() && camera["K"].size() == 9,
          "Invalid spatial camera K shape");
  for (int i = 0; i < 9; ++i) result.K(i / 3, i % 3) = Number(camera["K"][i]);
  result.camera_from_body = Pose(camera["camera_from_body"]);
  result.capture_time = Number(camera["capture_time"]);
  return result;
}

void WriteGaussianRenderMetadata(cv::FileStorage& out,
                                 const GaussianRenderMetadata& metadata) {
  Require(!metadata.extents.empty() && metadata.extents.size() <= 10,
          "Spatial metadata exceeds its extent budget");
  std::vector<std::uint64_t> frames;
  for (const auto& extent : metadata.extents)
    frames.push_back(extent.origin_keyframe_id);
  ValidateGaussianRenderMetadata(metadata, metadata.content, frames);
  using gaussian_mapper_io::Id;
  out << "{" << "content" << "{" << "session_id" << metadata.content.session_id;
  Id(out, "submap_id", metadata.content.submap_id);
  Id(out, "source_generation", metadata.content.source_generation);
  Id(out, "content_job_id", metadata.content.content_job_id);
  out << "}" << "extents" << "[";
  for (const auto& extent : metadata.extents) {
    out << "{";
    Id(out, "origin_keyframe_id", extent.origin_keyframe_id);
    for (const auto* key : {"min", "max"}) {
      const auto& bound = std::string(key) == "min" ? extent.min : extent.max;
      out << key << "[";
      for (int i = 0; i < 3; ++i) out << bound[i];
      out << "]";
    }
    out << "max_scale" << extent.max_scale << "}";
  }
  const auto& camera = metadata.camera;
  out << "]" << "camera" << "{";
  Id(out, "keyframe_id", camera.keyframe_id);
  Id(out, "calibration_id", camera.calibration_id);
  out << "width" << camera.width << "height" << camera.height << "K" << "[";
  for (int i = 0; i < 9; ++i) out << camera.K(i / 3, i % 3);
  out << "]";
  gaussian_mapper_io::Transform(out, "camera_from_body",
                                camera.camera_from_body);
  out << "capture_time" << camera.capture_time << "}" << "}";
}

GaussianRenderMetadata DecodeGaussianRenderMetadata(
    const cv::FileNode& node, const GaussianContentId& content,
    const std::vector<std::uint64_t>& frames) {
  Keys(node, {"content", "extents", "camera"});
  const auto source = node["content"];
  Keys(source,
       {"session_id", "submap_id", "source_generation", "content_job_id"});
  Require(source["session_id"].isString(), "Invalid spatial session");
  GaussianRenderMetadata result;
  result.content = {static_cast<std::string>(source["session_id"]),
                    Id(source["submap_id"]), Id(source["source_generation"]),
                    Id(source["content_job_id"])};
  const auto extents = node["extents"];
  Require(!frames.empty() && frames.size() <= 10 && extents.isSeq() &&
              extents.size() == frames.size(),
          "Invalid spatial coverage");
  for (const auto& extent : extents) {
    Keys(extent, {"origin_keyframe_id", "min", "max", "max_scale"});
    result.extents.push_back({Id(extent["origin_keyframe_id"]),
                              Vector(extent["min"]), Vector(extent["max"]),
                              Number(extent["max_scale"])});
  }
  result.camera = DecodeGaussianCamera(node["camera"]);
  ValidateGaussianRenderMetadata(result, content, frames);
  return result;
}

}  // namespace livo_atlas
