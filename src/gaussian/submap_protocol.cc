#include "gaussian/submap_protocol.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

#include "mapping/pose_math.h"

namespace livo_atlas {
namespace {

using Fields = std::set<std::string>;

void Require(bool value, const char* message) {
  if (!value) throw std::invalid_argument(message);
}

std::string String(const cv::FileNode& root, const char* name) {
  Require(root[name].isString(), "Expected a schema 2 string");
  return static_cast<std::string>(root[name]);
}

std::uint64_t UnsignedNode(const cv::FileNode& node) {
  Require(node.isString(), "Expected a uint64 string");
  const auto text = static_cast<std::string>(node);
  Require(!text.empty() && text.size() <= 20 &&
              (text.size() == 1 || text[0] != '0') &&
              std::all_of(text.begin(), text.end(),
                          [](char c) { return c >= '0' && c <= '9'; }),
          "Expected a canonical uint64 string");
  std::uint64_t value = 0;
  const auto result =
      std::from_chars(text.data(), text.data() + text.size(), value);
  Require(result.ec == std::errc{} && result.ptr == text.data() + text.size(),
          "Uint64 value overflow");
  return value;
}

std::uint64_t Id(const cv::FileNode& root, const char* name) {
  return UnsignedNode(root[name]);
}

int Integer(const cv::FileNode& root, const char* name, int low, int high) {
  Require(root[name].isInt(), "Expected an integer count");
  const int value = root[name];
  Require(value >= low && value <= high, "Integer is outside schema 2 bounds");
  return value;
}

double Number(const cv::FileNode& node) {
  Require(node.isInt() || node.isReal(), "Expected a finite number");
  const double value = node;
  Require(std::isfinite(value), "Expected a finite number");
  return value;
}

bool Hex(const std::string& value, std::size_t count) {
  return value.size() == count &&
         std::all_of(value.begin(), value.end(), [](char c) {
           return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
         });
}

Fields Base(bool job = true) {
  Fields result = {"schema", "type", "session_id"};
  if (job) result.insert({"job_id", "generation", "pose_version"});
  return result;
}

void Keys(const cv::FileNode& root, Fields required, Fields optional = {}) {
  for (const auto& key : required)
    Require(root[key].type() != cv::FileNode::NONE, "Missing schema 2 field");
  required.insert(optional.begin(), optional.end());
  for (auto it = root.begin(); it != root.end(); ++it)
    Require(required.count((*it).name()), "Unknown schema 2 field");
}

void Pose(const double* values, RigidTransform* pose) {
  for (int i = 0; i < 16; ++i)
    Require(std::isfinite(values[i]), "Nonfinite pose");
  Require(std::abs(values[12]) <= 1e-9 && std::abs(values[13]) <= 1e-9 &&
              std::abs(values[14]) <= 1e-9 && std::abs(values[15] - 1) <= 1e-9,
          "Invalid rigid pose last row");
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column)
      pose->rotation(row, column) = values[4 * row + column];
    pose->translation[row] = values[4 * row + 3];
  }
  pose_math::Validate(*pose);
}

RigidTransform Pose(const cv::FileNode& node) {
  Require(node.isSeq() && node.size() == 16, "Expected a 4x4 rigid pose");
  double values[16];
  for (int i = 0; i < 16; ++i) values[i] = Number(node[i]);
  RigidTransform pose;
  Pose(values, &pose);
  return pose;
}

void WritePose(cv::FileStorage& json, const char* name,
               const RigidTransform& pose) {
  json << name << "[";
  for (int row = 0; row < 4; ++row)
    for (int column = 0; column < 4; ++column)
      json << (row == 3 ? (column == 3 ? 1.0 : 0.0)
                        : column == 3 ? pose.translation[row]
                                      : pose.rotation(row, column));
  json << "]";
}

std::size_t Pixels(const cv::FileNode& root) {
  const auto width = Integer(root, "width", 1, 1000000);
  const auto height = Integer(root, "height", 1, 1000000);
  const auto pixels = static_cast<std::size_t>(width) * height;
  Require(pixels <= 1000000, "Image pixel budget exceeded");
  return pixels;
}

std::size_t Frames(const cv::FileNode& root) {
  const auto frames = root["frame_ids"];
  Require(frames.isSeq() && frames.size() >= 1 && frames.size() <= 10,
          "Invalid retained frame IDs");
  std::uint64_t previous = 0;
  bool first = true;
  for (const auto& value : frames) {
    const auto id = UnsignedNode(value);
    Require(id < 512 && (first || id > previous),
            "Invalid retained frame order");
    previous = id;
    first = false;
  }
  (void)Integer(root, "gaussian_count", static_cast<int>(frames.size()),
                static_cast<int>(frames.size()) * 1000);
  return frames.size();
}

void Content(const cv::FileNode& root) {
  (void)Id(root, "source_generation");
  Require(Id(root, "content_job_id") > 0, "Content job must be nonzero");
}

void View(const cv::FileNode& root, const std::vector<std::uint8_t>& body,
          const GaussianTarget& target) {
  const auto pixels = Pixels(root);
  const auto id = Id(root, "keyframe_id");
  const auto anchor = target.group.primary.anchor_keyframe_id;
  Require(id >= anchor && id - anchor < target.submap_from_body.size() &&
              Id(root, "calibration_id") == 0,
          "View does not belong to primary membership");
  for (const auto* name :
       {"state_timestamp", "cloud_timestamp", "image_timestamp",
        "inverse_exposure_time", "cx", "cy"})
    (void)Number(root[name]);
  Require(Number(root["fx"]) > 0 && Number(root["fy"]) > 0,
          "Invalid focal lengths");
  const auto camera = Pose(root["camera_from_submap"]);
  const auto expected = pose_math::Compose(
      Pose(root["camera_from_body"]),
      pose_math::Inverse(target.submap_from_body[id - anchor]));
  Require(
      (camera.rotation - expected.rotation).cwiseAbs().maxCoeff() <= 1e-8 &&
          (camera.translation - expected.translation).cwiseAbs().maxCoeff() <=
              1e-8,
      "Independent camera/body relation disagrees");
  for (std::size_t i = 0; i < pixels; ++i) {
    const auto image = body[3 * pixels + i];
    const auto depth_mask = body[4 * pixels + i];
    std::uint32_t bits = 0;
    for (int byte = 0; byte < 4; ++byte)
      bits |= static_cast<std::uint32_t>(body[5 * pixels + 4 * i + byte])
              << (8 * byte);
    float depth = 0;
    std::memcpy(&depth, &bits, sizeof(depth));
    Require(image <= 1 && depth_mask <= image && std::isfinite(depth) &&
                (depth_mask ? depth > 0 : depth == 0),
            "Invalid view masks or depth");
  }
}

}  // namespace

std::vector<std::uint8_t> EncodeGaussianPrimaryPoses(
    const GaussianTarget& target) {
  ValidateGaussianTarget(target);
  static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559,
                "Primary poses require IEEE-754 float64");
  std::vector<std::uint8_t> result;
  result.reserve(128 * target.submap_from_body.size());
  for (const auto& pose : target.submap_from_body) {
    for (int row = 0; row < 4; ++row) {
      for (int column = 0; column < 4; ++column) {
        const double value = row == 3
                                 ? (column == 3 ? 1.0 : 0.0)
                                 : column == 3 ? pose.translation[row]
                                               : pose.rotation(row, column);
        std::uint64_t bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));
        for (int byte = 0; byte < 8; ++byte)
          result.push_back(static_cast<std::uint8_t>(bits >> (8 * byte)));
      }
    }
  }
  return result;
}

void WriteGaussianTargetHeader(cv::FileStorage& json,
                               const GaussianTarget& target) {
  ValidateGaussianTarget(target);
  json << "generation" << std::to_string(target.generation) << "submap_id"
       << std::to_string(target.group.primary.id) << "pose_version"
       << std::to_string(target.pose_version) << "membership_revision"
       << std::to_string(target.group.membership_revision) << "frame_count"
       << std::to_string(target.frame_count) << "optimized_frame_count"
       << std::to_string(target.optimized_frame_count) << "anchor_keyframe_id"
       << std::to_string(target.group.primary.anchor_keyframe_id)
       << "primary_count" << static_cast<int>(target.submap_from_body.size())
       << "submap_state"
       << (target.group.state == OnlineSubmapState::kSealed ? "sealed"
                                                            : "active")
       << "parent_frame"
       << (target.parent_frame == PoseFrame::kMap ? "map" : "odom");
  WritePose(json, "parent_from_submap", target.parent_from_submap);
}

GaussianTarget DecodeGaussianTarget(const cv::FileNode& header,
                                    const std::vector<std::uint8_t>& body,
                                    std::size_t offset) {
  const auto count = Integer(header, "primary_count", 1, 512);
  const auto anchor = Id(header, "anchor_keyframe_id");
  const auto frontier = Id(header, "frame_count");
  Require(frontier <= 512 && anchor < frontier &&
              static_cast<std::uint64_t>(count) <= frontier - anchor &&
              offset <= body.size() && body.size() - offset == 128u * count,
          "Primary count, frontier or exact body size is invalid");
  GaussianTarget target;
  target.session_id = String(header, "session_id");
  target.generation = Id(header, "generation");
  target.pose_version = Id(header, "pose_version");
  target.frame_count = frontier;
  target.optimized_frame_count = Id(header, "optimized_frame_count");
  const auto parent = String(header, "parent_frame");
  Require(parent == "odom" || parent == "map", "Invalid parent frame");
  target.parent_frame = parent == "odom" ? PoseFrame::kOdom : PoseFrame::kMap;
  target.parent_from_submap = Pose(header["parent_from_submap"]);
  target.group.primary.id = Id(header, "submap_id");
  target.group.primary.anchor_keyframe_id = anchor;
  target.group.membership_revision = Id(header, "membership_revision");
  const auto state = String(header, "submap_state");
  Require(state == "active" || state == "sealed", "Invalid submap state");
  target.group.state = state == "active" ? OnlineSubmapState::kActive
                                         : OnlineSubmapState::kSealed;
  Require(target.group.membership_revision ==
              static_cast<std::uint64_t>(count) + (state == "sealed"),
          "Membership revision differs from primary count");
  for (int i = 0; i < count; ++i) {
    double values[16];
    for (int element = 0; element < 16; ++element) {
      std::uint64_t bits = 0;
      for (int byte = 0; byte < 8; ++byte)
        bits |= static_cast<std::uint64_t>(
                    body[offset + 128 * i + 8 * element + byte])
                << (8 * byte);
      std::memcpy(&values[element], &bits, sizeof(bits));
    }
    RigidTransform pose;
    Pose(values, &pose);
    target.group.primary.keyframe_ids.push_back(anchor + i);
    target.submap_from_body.push_back(pose);
  }
  ValidateGaussianTarget(target);
  return target;
}

void ValidateGaussianSubmapMessage(const cv::FileNode& root,
                                   const std::vector<std::uint8_t>& body) {
  const auto kind = String(root, "type");
  Require(Hex(String(root, "session_id"), 32), "Invalid session ID");
  auto required = Base();
  if (kind != "hello" && kind != "hello_ok" && kind != "error")
    for (const auto* name : {"job_id", "generation", "pose_version"})
      (void)Id(root, name);
  if (kind == "hello") {
    required = Base(false);
    required.insert("state_directory");
    Keys(root, required);
    const auto name = String(root, "state_directory");
    const std::filesystem::path path(name);
    Require(name.find('\0') == std::string::npos && path.is_absolute() &&
                path.filename() == "states" &&
                path.lexically_normal().string() == name &&
                std::none_of(path.begin(), path.end(),
                             [](const auto& part) { return part == ".."; }),
            "Expected a normalized absolute states directory");
  } else if (kind == "hello_ok") {
    required = Base(false);
    required.insert({"mode", "state_schema", "python", "torch", "gsplat",
                     "cuda", "device", "gpu_free_bytes", "gpu_total_bytes"});
    Keys(root, required);
    Require(String(root, "mode") == "persistent_submaps" &&
                Integer(root, "state_schema", 1, 1) == 1,
            "Worker lacks persistent submap support");
    for (const auto* name : {"python", "torch", "gsplat", "cuda", "device"}) {
      const auto value = String(root, name);
      Require(!value.empty() && value.size() <= 128, "Invalid worker metadata");
    }
    const auto total = Id(root, "gpu_total_bytes");
    Require(total && Id(root, "gpu_free_bytes") <= total,
            "Invalid GPU memory counters");
  } else if (kind == "activate" || kind == "view") {
    required.insert({"submap_id", "membership_revision", "frame_count",
                     "optimized_frame_count", "anchor_keyframe_id",
                     "primary_count", "submap_state", "parent_frame",
                     "parent_from_submap"});
    Fields optional;
    if (kind == "activate") {
      required.insert("action");
      optional = {"source_generation", "content_job_id", "asset_id",
                  "manifest_sha256"};
    } else {
      required.insert({"keyframe_id", "calibration_id", "state_timestamp",
                       "cloud_timestamp", "image_timestamp",
                       "inverse_exposure_time", "fx", "fy", "cx", "cy", "width",
                       "height", "camera_from_submap", "camera_from_body"});
    }
    Keys(root, required, optional);
    const auto target =
        DecodeGaussianTarget(root, body, kind == "view" ? 9 * Pixels(root) : 0);
    if (kind == "view") {
      View(root, body, target);
    } else {
      const auto action = String(root, "action");
      Require(action == "empty" || action == "reuse" || action == "rebuild",
              "Invalid activation action");
      if (action == "empty") {
        for (const auto& name : optional)
          Require(root[name].type() == cv::FileNode::NONE,
                  "Empty activation references old content");
      } else {
        Content(root);
        const bool asset = root["asset_id"].type() != cv::FileNode::NONE;
        Require(asset == (root["manifest_sha256"].type() != cv::FileNode::NONE),
                "Asset and digest must be supplied together");
        if (asset)
          Require(
              Id(root, "asset_id") && Hex(String(root, "manifest_sha256"), 64),
              "Invalid asset identity");
      }
    }
    return;
  } else if (kind == "activated") {
    required.insert("submap_id");
    Keys(root, required);
    (void)Id(root, "submap_id");
  } else if (kind == "candidate") {
    required.insert({"submap_id", "operation", "membership_revision",
                     "frame_count", "optimized_frame_count", "frame_ids",
                     "width", "height", "steps", "gaussian_count",
                     "supervised_depth_pixels", "rgb_l1", "depth_l1_m",
                     "nonzero_gradient", "parameter_updated", "capture_time"});
    Keys(root, required,
         {"gpu_peak_allocated_bytes", "gpu_peak_reserved_bytes",
          "worker_peak_rss_bytes"});
    (void)Id(root, "submap_id");
    const auto count = Frames(root);
    const auto frontier = Id(root, "frame_count");
    const auto revision = Id(root, "membership_revision");
    Require(frontier <= 512 &&
                UnsignedNode(root["frame_ids"][static_cast<int>(count - 1)]) <
                    frontier &&
                Id(root, "optimized_frame_count") <= frontier &&
                revision >= 1 && revision <= 513,
            "Invalid candidate frontier");
    const auto operation = String(root, "operation");
    Require(
        operation == "view" || operation == "reuse" || operation == "rebuild",
        "Invalid operation");
    const int steps =
        operation == "reuse"
            ? 0
            : operation == "view" ? 20 : static_cast<int>(20 * count);
    (void)Integer(root, "steps", steps, steps);
    (void)Integer(root, "supervised_depth_pixels", 0, steps * 1000000);
    Require(Number(root["rgb_l1"]) >= 0 && Number(root["depth_l1_m"]) >= 0,
            "Negative training loss");
    (void)Number(root["capture_time"]);
    for (const auto* name : {"nonzero_gradient", "parameter_updated"})
      (void)Integer(root, name, 0, 1);
    for (const auto* name :
         {"gpu_peak_allocated_bytes", "gpu_peak_reserved_bytes",
          "worker_peak_rss_bytes"})
      if (root[name].type() != cv::FileNode::NONE) (void)Id(root, name);
    Require(body.size() == 3 * Pixels(root), "Incorrect candidate image size");
    return;
  } else if (kind == "decision" || kind == "commit_ok" ||
             kind == "discard_ok") {
    required.insert({"submap_id", "operation"});
    if (kind == "decision") required.insert("action");
    if (kind == "commit_ok")
      required.insert({"source_generation", "content_job_id", "frame_ids",
                       "gaussian_count"});
    Keys(root, required);
    (void)Id(root, "submap_id");
    const auto operation = String(root, "operation");
    Require(
        operation == "view" || operation == "reuse" || operation == "rebuild",
        "Invalid operation");
    if (kind == "decision") {
      const auto action = String(root, "action");
      Require(action == "accept" || action == "discard", "Invalid decision");
    } else if (kind == "commit_ok") {
      Content(root);
      (void)Frames(root);
    }
  } else if (kind == "checkpoint" || kind == "checkpoint_ok") {
    required.insert(
        {"submap_id", "asset_id", "source_generation", "content_job_id"});
    if (kind == "checkpoint_ok")
      required.insert({"manifest_sha256", "manifest_bytes", "gaussian_bytes",
                       "observation_bytes", "frame_ids", "gaussian_count"});
    Keys(root, required);
    (void)Id(root, "submap_id");
    Content(root);
    Require(Id(root, "asset_id") > 0, "Asset ID must be nonzero");
    if (kind == "checkpoint_ok") {
      (void)Frames(root);
      Require(Hex(String(root, "manifest_sha256"), 64),
              "Invalid manifest digest");
      for (const auto& value :
           {std::pair<const char*, std::uint64_t>{"manifest_bytes", 512 * 1024},
            {"gaussian_bytes", 1024 * 1024},
            {"observation_bytes", 192 * 1024 * 1024}})
        Require(
            Id(root, value.first) > 0 && Id(root, value.first) <= value.second,
            "Asset exceeds file budget");
    }
  } else if (kind == "map_committed" || kind == "map_committed_ok") {
    required.insert({"map_sequence", "asset_ids"});
    Keys(root, required);
    Require(Id(root, "map_sequence") > 0, "Map sequence must be nonzero");
    const auto ids = root["asset_ids"];
    Require(ids.isSeq() && ids.size() <= 32, "Too many map references");
    std::uint64_t previous = 0;
    for (const auto& item : ids) {
      const auto value = UnsignedNode(item);
      Require(value > previous,
              "Map references must be nonzero, ordered and unique");
      previous = value;
    }
  } else if (kind == "prune" || kind == "prune_ok") {
    required.insert({"map_sequence", "asset_id"});
    Keys(root, required);
    Require(Id(root, "map_sequence") > 0 && Id(root, "asset_id") > 0,
            "Invalid map/asset identity");
  } else if (kind == "stop" || kind == "stopped") {
    required.insert("map_sequence");
    Keys(root, required, {"incomplete"});
    (void)Id(root, "map_sequence");
    if (root["incomplete"].type() != cv::FileNode::NONE)
      (void)Integer(root, "incomplete", 0, 1);
  } else if (kind == "error") {
    required = Base(false);
    required.insert("code");
    Keys(root, required,
         {"job_id", "generation", "pose_version", "submap_id", "message"});
    const Fields codes = {"gpu_unavailable", "extension_unavailable",
                          "protocol_error",  "worker_error",
                          "storage_error",   "commit_uncertain",
                          "resource_limit"};
    Require(codes.count(String(root, "code")), "Invalid worker error");
    for (const auto* name :
         {"job_id", "generation", "pose_version", "submap_id"})
      if (root[name].type() != cv::FileNode::NONE) (void)Id(root, name);
    if (root["message"].type() != cv::FileNode::NONE)
      Require(String(root, "message").size() <= 2048,
              "Worker error detail exceeds budget");
  } else {
    throw std::invalid_argument("Unsupported schema 2 message");
  }
  Require(body.empty(), "Unexpected message body");
}

}  // namespace livo_atlas
