#include "gaussian/render_protocol.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstring>
#include <set>
#include <stdexcept>
#include <system_error>
#include <utility>

#include "gaussian/mapper_io.h"
#include "gaussian/submap_protocol.h"
#include "mapping/pose_math.h"

namespace livo_atlas {
namespace {
using Fields = std::set<std::string>;
using gaussian_mapper_io::Base;
using gaussian_mapper_io::Id;
using gaussian_mapper_io::Json;
using gaussian_mapper_io::Transform;

const Fields kRoot = {
    "schema", "type", "session_id", "job_id", "generation", "pose_version",
    "map_sequence", "frame_count", "optimized_frame_count", "parent_frame",
    "query_keyframe_id", "query_submap_id", "neighbor_reason", "blocks"};

void Require(bool condition, const char* message) {
  if (!condition) throw std::invalid_argument(message);
}

void Keys(const cv::FileNode& node, Fields fields) {
  Require(node.isMap() && node.size() == fields.size(),
          "Missing or unknown render fields");
  for (const auto& child : node)
    Require(fields.erase(child.name()) == 1, "Unknown render field");
}

std::string Text(const cv::FileNode& node) {
  Require(node.isString(), "Expected a render string");
  return static_cast<std::string>(node);
}

std::uint64_t Uint(const cv::FileNode& node) {
  const auto text = Text(node);
  std::uint64_t value = 0;
  const auto parsed =
      std::from_chars(text.data(), text.data() + text.size(), value);
  Require(!text.empty() && parsed.ec == std::errc{} &&
              parsed.ptr == text.data() + text.size() &&
              text == std::to_string(value), "Invalid render uint64");
  return value;
}

int Count(const cv::FileNode& node, int low, int high) {
  Require(node.isInt(), "Expected a render integer");
  const int value = node;
  Require(value >= low && value <= high, "Render count exceeds its budget");
  return value;
}

double Number(const cv::FileNode& node) {
  Require(node.isReal() || node.isInt(), "Expected a finite render number");
  const double value = node;
  Require(std::isfinite(value), "Nonfinite render number");
  return value;
}

RigidTransform PoseValues(const double* values) {
  RigidTransform pose;
  for (int i = 0; i < 16; ++i) {
    const double value = values[i];
    Require(std::isfinite(value), "Nonfinite render pose");
    if (i / 4 == 3)
      Require(std::abs(value - (i == 15 ? 1 : 0)) <= 1e-9,
              "Invalid render pose last row");
    else if (i % 4 == 3)
      pose.translation[i / 4] = value;
    else
      pose.rotation(i / 4, i % 4) = value;
  }
  pose_math::Validate(pose);
  return pose;
}

RigidTransform Pose(const cv::FileNode& node) {
  Require(node.isSeq() && node.size() == 16, "Expected a render rigid pose");
  double values[16];
  for (int i = 0; i < 16; ++i) values[i] = Number(node[i]);
  return PoseValues(values);
}

RigidTransform BodyPose(const std::vector<std::uint8_t>& body,
                        std::size_t offset) {
  double values[16];
  for (int i = 0; i < 16; ++i) {
    std::uint64_t bits = 0;
    for (int byte = 0; byte < 8; ++byte)
      bits |= static_cast<std::uint64_t>(body[offset + i * 8 + byte])
              << (8 * byte);
    std::memcpy(&values[i], &bits, 8);
  }
  return PoseValues(values);
}

void AppendPose(const RigidTransform& pose, std::vector<std::uint8_t>* body) {
  for (int i = 0; i < 16; ++i) {
    const double value = i / 4 == 3 ? (i == 15 ? 1 : 0)
                         : i % 4 == 3 ? pose.translation[i / 4]
                                      : pose.rotation(i / 4, i % 4);
    std::uint64_t bits;
    std::memcpy(&bits, &value, 8);
    for (int byte = 0; byte < 8; ++byte)
      body->push_back(static_cast<std::uint8_t>(bits >> (8 * byte)));
  }
}

void Content(cv::FileStorage& out, const GaussianContentId& content) {
  Id(out, "submap_id", content.submap_id);
  Id(out, "source_generation", content.source_generation);
  Id(out, "content_job_id", content.content_job_id);
}

GaussianRenderRequest Header(const cv::FileNode& root) {
  GaussianRenderRequest result;
  result.session_id = Text(root["session_id"]);
  Require(result.session_id.size() == 32 &&
              result.session_id.find_first_not_of("0123456789abcdef") ==
                  std::string::npos, "Invalid render session");
  Require(Count(root["schema"], 2, 2) == 2, "Invalid render schema");
  result.job_id = Uint(root["job_id"]);
  result.generation = Uint(root["generation"]);
  result.map_sequence = Uint(root["map_sequence"]);
  result.pose_version = Uint(root["pose_version"]);
  result.frame_count = Uint(root["frame_count"]);
  result.optimized_frame_count = Uint(root["optimized_frame_count"]);
  result.query_keyframe_id = Uint(root["query_keyframe_id"]);
  result.query_submap_id = Uint(root["query_submap_id"]);
  result.neighbor_reason = Text(root["neighbor_reason"]);
  Require(result.job_id && result.generation && result.map_sequence &&
              result.query_keyframe_id < result.frame_count &&
              result.frame_count <= 512 &&
              result.optimized_frame_count <= result.frame_count &&
              bool(result.pose_version) == bool(result.optimized_frame_count),
          "Invalid render frontier");
  Require(Text(root["parent_frame"]) ==
              (result.pose_version ? "map" : "odom"),
          "Invalid render parent frame");
  result.parent_frame =
      result.pose_version ? PoseFrame::kMap : PoseFrame::kOdom;
  const auto blocks = root["blocks"];
  Require(blocks.isSeq() && blocks.size() >= 1 && blocks.size() <= 2,
          "Select one or two render blocks");
  int residents = 0;
  for (const auto& row : blocks) {
    const bool resident = Count(row["resident"], 0, 1) != 0;
    Fields fields = {"target", "content", "resident", "frame_ids",
                     "gaussian_count"};
    if (!resident) fields.insert("asset");
    Keys(row, fields);
    const auto target = row["target"];
    Keys(target, {"generation", "submap_id", "pose_version",
                  "membership_revision", "frame_count",
                  "optimized_frame_count", "anchor_keyframe_id",
                  "primary_count", "submap_state", "parent_frame",
                  "parent_from_submap"});
    GaussianRenderBlock block;
    auto& t = block.target;
    t.session_id = result.session_id;
    t.generation = Uint(target["generation"]);
    t.group.primary.id = Uint(target["submap_id"]);
    t.group.primary.anchor_keyframe_id = Uint(target["anchor_keyframe_id"]);
    t.group.membership_revision = Uint(target["membership_revision"]);
    const int count = Count(target["primary_count"], 1, 512);
    const auto anchor = t.group.primary.anchor_keyframe_id;
    const auto state = Text(target["submap_state"]);
    Require(t.generation && anchor < result.frame_count &&
                static_cast<std::uint64_t>(count) <=
                    result.frame_count - anchor &&
                (state == "active" || state == "sealed") &&
                t.group.membership_revision == count + (state == "sealed"),
            "Invalid render membership");
    t.group.state = state == "sealed" ? OnlineSubmapState::kSealed
                                       : OnlineSubmapState::kActive;
    t.pose_version = Uint(target["pose_version"]);
    t.frame_count = Uint(target["frame_count"]);
    t.optimized_frame_count = Uint(target["optimized_frame_count"]);
    t.parent_frame = result.parent_frame;
    t.parent_from_submap = Pose(target["parent_from_submap"]);
    Require(t.pose_version == result.pose_version &&
                t.frame_count == result.frame_count &&
                t.optimized_frame_count == result.optimized_frame_count &&
                Text(target["parent_frame"]) == Text(root["parent_frame"]),
            "Render target differs from root frontier");
    for (int i = 0; i < count; ++i)
      t.group.primary.keyframe_ids.push_back(anchor + i);
    const auto content = row["content"];
    Keys(content, {"submap_id", "source_generation", "content_job_id"});
    block.content = {result.session_id, Uint(content["submap_id"]),
                     Uint(content["source_generation"]),
                     Uint(content["content_job_id"])};
    Require(block.content.submap_id == t.group.primary.id &&
                block.content.source_generation &&
                block.content.content_job_id,
            "Invalid render content");
    block.resident = resident;
    residents += resident;
    if (!resident) {
      const auto asset = row["asset"];
      Keys(asset, {"asset_id", "submap_id", "source_generation",
                   "content_job_id", "manifest_sha256", "manifest_bytes",
                   "gaussian_bytes", "observation_bytes"});
      Require(Uint(asset["submap_id"]) == block.content.submap_id &&
                  Uint(asset["source_generation"]) ==
                      block.content.source_generation &&
                  Uint(asset["content_job_id"]) == block.content.content_job_id,
              "Render asset content mismatch");
      block.asset = GaussianAssetReference{
          Uint(asset["asset_id"]), block.content,
          Text(asset["manifest_sha256"]), Uint(asset["manifest_bytes"]),
          Uint(asset["gaussian_bytes"]), Uint(asset["observation_bytes"])};
      ValidateGaussianAssetReference(*block.asset);
    }
    const auto frames = row["frame_ids"];
    Require(frames.isSeq() && frames.size() >= 1 && frames.size() <= 10,
            "Invalid render coverage");
    for (const auto& frame : frames) {
      const auto id = Uint(frame);
      Require(id >= anchor && id < anchor + count &&
                  (block.frame_ids.empty() || id > block.frame_ids.back()),
              "Invalid retained render frame order");
      block.frame_ids.push_back(id);
    }
    block.gaussian_count = Count(
        row["gaussian_count"], static_cast<int>(frames.size()),
        1000 * static_cast<int>(frames.size()));
    Require(result.blocks.empty() ||
                t.group.primary.id >
                    result.blocks.back().target.group.primary.id,
            "Duplicate or unordered render blocks");
    result.blocks.push_back(std::move(block));
  }
  Require(residents <= 1 && result.blocks.back().target.group.primary.id ==
                                result.query_submap_id &&
              std::binary_search(result.blocks.back().frame_ids.begin(),
                                 result.blocks.back().frame_ids.end(),
                                 result.query_keyframe_id),
          "Query owner is missing from render selection");
  if (result.blocks.size() == 2) {
    Require(result.query_submap_id > 0 && result.neighbor_reason.empty() &&
                result.blocks[0].target.group.primary.id ==
                    result.query_submap_id - 1, "Invalid render neighbor");
  } else {
    Require(result.neighbor_reason ==
                (result.query_submap_id ? "neighbor_unavailable"
                                        : "no_previous_group"),
            "Missing single-block render reason");
  }
  return result;
}

GaussianRenderRequest Request(const cv::FileNode& root,
                              const std::vector<std::uint8_t>& body) {
  Keys(root, kRoot);
  Require(Text(root["type"]) == "joint_render", "Expected render request");
  auto request = Header(root);
  std::size_t bytes = 128;
  for (const auto& block : request.blocks)
    bytes += 128 * block.target.group.primary.keyframe_ids.size();
  Require(bytes <= 131200 && body.size() == bytes,
          "Incorrect render request body size");
  request.parent_from_body = BodyPose(body, 0);
  std::size_t offset = 128;
  for (auto& block : request.blocks) {
    for (std::size_t i = 0; i < block.target.group.primary.keyframe_ids.size();
         ++i, offset += 128)
      block.target.submap_from_body.push_back(BodyPose(body, offset));
  }
  ValidateGaussianRenderRequest(request);
  return request;
}

GaussianRenderMetrics Metrics(const cv::FileNode& node, int pixels) {
  Keys(node, {"image_pixels", "covered_pixels", "depth_pixels_available",
              "depth_pixels_used", "image_valid", "depth_valid", "psnr_valid",
              "rgb_perfect", "rgb_l1", "rgb_mse", "rgb_psnr_db",
              "alpha_coverage", "depth_l1_m", "depth_rmse_m"});
  GaussianRenderMetrics result;
  result.image_pixels = Count(node["image_pixels"], 0, pixels);
  result.covered_pixels = Count(node["covered_pixels"], 0, result.image_pixels);
  result.depth_pixels_available =
      Count(node["depth_pixels_available"], 0, result.image_pixels);
  result.depth_pixels_used =
      Count(node["depth_pixels_used"], 0,
            std::min(result.depth_pixels_available, result.covered_pixels));
  const auto image = result.image_pixels;
  const auto depth = result.depth_pixels_used;
  const double mse = Number(node["rgb_mse"]);
  Require(mse >= 0 && mse <= 1 &&
              Count(node["image_valid"], 0, 1) == (image > 0) &&
              Count(node["depth_valid"], 0, 1) == (depth > 0) &&
              Count(node["psnr_valid"], 0, 1) == (image > 0 && mse > 0) &&
              Count(node["rgb_perfect"], 0, 1) == (image > 0 && mse == 0),
          "Inconsistent render metric validity");
  result.rgb_perfect = image > 0 && mse == 0;
  const auto optional = [&](const char* key, bool valid)
      -> std::optional<double> {
    const double value = Number(node[key]);
    Require(valid || value == 0, "Invalid render metric needs finite zero");
    if (valid) return value;
    return {};
  };
  result.rgb_l1 = optional("rgb_l1", image > 0);
  result.rgb_mse = optional("rgb_mse", image > 0);
  result.rgb_psnr_db = optional("rgb_psnr_db", image > 0 && mse > 0);
  result.alpha_coverage = optional("alpha_coverage", image > 0);
  result.depth_l1_m = optional("depth_l1_m", depth > 0);
  result.depth_rmse_m = optional("depth_rmse_m", depth > 0);
  const double coverage =
      image ? static_cast<double>(result.covered_pixels) / image : 0;
  Require(Number(node["rgb_l1"]) >= 0 && Number(node["rgb_l1"]) <= 1 &&
              Number(node["depth_l1_m"]) >= 0 &&
              Number(node["depth_rmse_m"]) >= 0 &&
              std::abs(Number(node["alpha_coverage"]) -
                       coverage) <
                  1e-9, "Invalid render error or alpha coverage");
  if (result.rgb_psnr_db)
    Require(std::abs(*result.rgb_psnr_db + 10 * std::log10(mse)) < 1e-6,
            "Incorrect render PSNR");
  return result;
}

bool EqualJson(const cv::FileNode& a, const cv::FileNode& b) {
  if (a.isMap() && b.isMap()) {
    if (a.size() != b.size()) return false;
    for (const auto& child : a)
      if (!EqualJson(child, b[child.name()])) return false;
    return true;
  }
  if (a.isSeq() && b.isSeq()) {
    if (a.size() != b.size()) return false;
    for (int i = 0; i < static_cast<int>(a.size()); ++i)
      if (!EqualJson(a[i], b[i])) return false;
    return true;
  }
  if (a.isString()) return b.isString() && Text(a) == Text(b);
  return (a.isInt() || a.isReal()) && (b.isInt() || b.isReal()) &&
         std::abs(Number(a) - Number(b)) <= 1e-9;
}

void Echo(const cv::FileNode& root, const GaussianRenderRequest& request) {
  const auto encoded = EncodeGaussianRenderRequest(request);
  cv::FileStorage expected(encoded.header_json,
                           cv::FileStorage::READ | cv::FileStorage::MEMORY);
  for (const auto& key : kRoot)
    if (key != "type")
      Require(EqualJson(root[key], expected[key]),
              "Render reply echo mismatch");
}

}  // namespace

GaussianMessage EncodeGaussianRenderRequest(
    const GaussianRenderRequest& request) {
  ValidateGaussianRenderRequest(request);
  Require(request.job_id > 0, "Render mailbox job ID is required");
  GaussianMessage message;
  message.header_json = Json([&](auto& out) {
    Base(out, "joint_render", request.session_id, 2);
    Id(out, "job_id", request.job_id);
    Id(out, "generation", request.generation);
    Id(out, "map_sequence", request.map_sequence);
    Id(out, "pose_version", request.pose_version);
    Id(out, "frame_count", request.frame_count);
    Id(out, "optimized_frame_count", request.optimized_frame_count);
    Id(out, "query_keyframe_id", request.query_keyframe_id);
    Id(out, "query_submap_id", request.query_submap_id);
    out << "parent_frame"
        << (request.parent_frame == PoseFrame::kMap ? "map" : "odom")
        << "neighbor_reason" << request.neighbor_reason << "blocks" << "[";
    for (const auto& block : request.blocks) {
      out << "{" << "target" << "{";
      WriteGaussianTargetHeader(out, block.target);
      out << "}" << "content" << "{";
      Content(out, block.content);
      out << "}" << "resident" << static_cast<int>(block.resident)
          << "frame_ids" << "[";
      for (const auto id : block.frame_ids) out << std::to_string(id);
      out << "]" << "gaussian_count" << block.gaussian_count;
      if (block.asset) {
        const auto& asset = *block.asset;
        out << "asset" << "{";
        Content(out, asset.content);
        Id(out, "asset_id", asset.asset_id);
        out << "manifest_sha256" << asset.manifest_sha256;
        Id(out, "manifest_bytes", asset.manifest_bytes);
        Id(out, "gaussian_bytes", asset.gaussian_bytes);
        Id(out, "observation_bytes", asset.observation_bytes);
        out << "}";
      }
      out << "}";
    }
    out << "]";
  });
  AppendPose(request.parent_from_body, &message.body);
  for (const auto& block : request.blocks) {
    const auto primary = EncodeGaussianPrimaryPoses(block.target);
    message.body.insert(message.body.end(), primary.begin(), primary.end());
  }
  return message;
}

GaussianRenderRequest DecodeGaussianRenderRequest(
    const GaussianMessage& message) {
  Require(message.header_json.size() <= 65536, "Render header exceeds budget");
  cv::FileStorage json(message.header_json,
                       cv::FileStorage::READ | cv::FileStorage::MEMORY);
  return Request(json.root(), message.body);
}

void ValidateGaussianRenderMessage(const cv::FileNode& root,
                                   const std::vector<std::uint8_t>& body) {
  const auto kind = Text(root["type"]);
  if (kind == "joint_render") {
    (void)Request(root, body);
    return;
  }
  auto fields = kRoot;
  if (kind == "joint_render_error") {
    fields.insert({"code", "error"});
    Keys(root, fields);
    (void)Header(root);
    Require(Text(root["code"]) == "oom" &&
                !Text(root["error"]).empty() &&
                Text(root["error"]).size() <= 4096 && body.empty(),
            "Invalid render error");
    return;
  }
  Require(kind == "joint_render_ok", "Unexpected render message");
  fields.insert({"training_view", "width", "height", "calibration_id", "fx",
                 "fy", "cx", "cy", "parent_from_body", "camera_from_body",
                 "camera_from_parent", "image_timestamp", "state_timestamp",
                 "cloud_timestamp", "inverse_exposure_time", "single",
                 "joint", "changed_pixels", "single_gaussian_count",
                 "gaussian_count", "worker_seconds", "gpu_allocated_bytes",
                 "gpu_reserved_bytes", "gpu_peak_allocated_bytes",
                 "gpu_peak_reserved_bytes", "worker_peak_rss_bytes"});
  Keys(root, fields);
  const auto request = Header(root);
  const int width = Count(root["width"], 1, 1000000);
  const int height = Count(root["height"], 1, 1000000);
  Require(static_cast<std::int64_t>(width) * height <= 1000000 &&
              body.size() == 3u * width * height, "Incorrect render RGB size");
  (void)Count(root["training_view"], 1, 1);
  Require(Uint(root["calibration_id"]) == 0 && Number(root["fx"]) > 0 &&
              Number(root["fy"]) > 0 && Number(root["worker_seconds"]) >= 0,
          "Invalid render camera or timing");
  for (const auto* key : {"cx", "cy", "image_timestamp", "state_timestamp",
                          "cloud_timestamp", "inverse_exposure_time"})
    (void)Number(root[key]);
  Require(pose_math::Equal(Pose(root["camera_from_parent"]),
              pose_math::Compose(Pose(root["camera_from_body"]),
                  pose_math::Inverse(Pose(root["parent_from_body"])))),
          "Incorrect render camera composition");
  (void)Metrics(root["single"], width * height);
  (void)Metrics(root["joint"], width * height);
  (void)Count(root["changed_pixels"], 0, width * height);
  int total = 0;
  for (const auto& block : request.blocks) total += block.gaussian_count;
  (void)Count(root["gaussian_count"], total, total);
  const int single = request.blocks.back().gaussian_count;
  (void)Count(root["single_gaussian_count"], single, single);
  for (const auto* key : {"gpu_allocated_bytes", "gpu_reserved_bytes",
                          "gpu_peak_allocated_bytes", "gpu_peak_reserved_bytes",
                          "worker_peak_rss_bytes"})
    (void)Uint(root[key]);
}

GaussianRender DecodeGaussianRenderReply(
    const GaussianMessage& message, const GaussianRenderRequest& request) {
  (void)EncodeGaussianMessage(message);
  const auto json = gaussian_mapper_io::Parse(
      message, "joint_render_ok", request.session_id, 2);
  Echo(json.root(), request);
  Require(pose_math::Equal(Pose(json["parent_from_body"]),
                           request.parent_from_body),
          "Query pose echo mismatch");
  GaussianRender result;
  result.request = request;
  result.rgb = message.body;
  result.width = json["width"];
  result.height = json["height"];
  result.K << Number(json["fx"]), 0, Number(json["cx"]),
              0, Number(json["fy"]), Number(json["cy"]), 0, 0, 1;
  result.camera_from_body = Pose(json["camera_from_body"]);
  result.camera_from_parent = Pose(json["camera_from_parent"]);
  result.capture_time = Number(json["image_timestamp"]);
  result.state_timestamp = Number(json["state_timestamp"]);
  result.cloud_timestamp = Number(json["cloud_timestamp"]);
  result.inverse_exposure_time = Number(json["inverse_exposure_time"]);
  result.worker_seconds = Number(json["worker_seconds"]);
  result.single = Metrics(json["single"], result.width * result.height);
  result.joint = Metrics(json["joint"], result.width * result.height);
  result.changed_pixels = json["changed_pixels"];
  result.gpu_allocated_bytes = Uint(json["gpu_allocated_bytes"]);
  result.gpu_reserved_bytes = Uint(json["gpu_reserved_bytes"]);
  result.gpu_peak_allocated_bytes = Uint(json["gpu_peak_allocated_bytes"]);
  result.gpu_peak_reserved_bytes = Uint(json["gpu_peak_reserved_bytes"]);
  result.worker_peak_rss_bytes = Uint(json["worker_peak_rss_bytes"]);
  return result;
}

std::string DecodeGaussianRenderError(
    const GaussianMessage& message, const GaussianRenderRequest& request) {
  (void)EncodeGaussianMessage(message);
  const auto json = gaussian_mapper_io::Parse(
      message, "joint_render_error", request.session_id, 2);
  Echo(json.root(), request);
  return Text(json["error"]);
}

}  // namespace livo_atlas
