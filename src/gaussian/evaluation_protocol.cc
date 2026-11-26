#include "gaussian/evaluation_protocol.h"

#include <charconv>
#include <cmath>
#include <cstring>
#include <set>
#include <stdexcept>

#include "gaussian/mapper_io.h"
#include "gaussian/render_protocol.h"
#include "gaussian/render_validation.h"

namespace livo_atlas {
namespace {
using gaussian_mapper_io::Base;
using gaussian_mapper_io::Id;
using gaussian_mapper_io::Json;

using gaussian_mapper_io::Transform;

void Require(bool value, const char* message) {
  if (!value) throw std::invalid_argument(message);
}

std::uint64_t ReadId(const cv::FileNode& node) {
  Require(node.isString(), "Expected canonical evaluation id");
  const auto value = static_cast<std::string>(node);
  std::uint64_t result = 0;
  const auto parsed =
      std::from_chars(value.data(), value.data() + value.size(), result);
  Require(!value.empty() && (value == "0" || value.front() != '0') &&
              parsed.ec == std::errc{} &&
              parsed.ptr == value.data() + value.size(),
          "Invalid canonical evaluation id");
  return result;
}

void Keys(const cv::FileNode& node, const std::set<std::string>& expected) {
  Require(node.isMap(), "Expected evaluation object");
  std::set<std::string> actual;
  for (const auto& field : node) actual.insert(field.name());
  Require(actual == expected, "Unknown or missing evaluation fields");
}

bool Hex(const cv::FileNode& node, std::size_t size) {
  if (!node.isString()) return false;
  const auto text = static_cast<std::string>(node);
  return text.size() == size &&
         text.find_first_not_of("0123456789abcdef") == std::string::npos;
}

double Number(const cv::FileNode& node) {
  Require(node.isInt() || node.isReal(), "Expected evaluation number");
  const auto value = static_cast<double>(node);
  Require(std::isfinite(value), "Nonfinite evaluation number");
  return value;
}

RigidTransform Pose(const cv::FileNode& node) {
  Require(node.isSeq() && node.size() == 16, "Invalid evaluation camera pose");
  RigidTransform result;
  for (int row = 0; row < 4; ++row)
    for (int col = 0; col < 4; ++col) {
      const auto value = Number(node[4 * row + col]);
      if (row == 3)
        Require(std::abs(value - (col == 3 ? 1 : 0)) <= 1e-9,
                "Invalid homogeneous row");
      else if (col == 3)
        result.translation[row] = value;
      else
        result.rotation(row, col) = value;
    }
  ValidateRenderPose(result);
  return result;
}

}  // namespace

GaussianMessage EncodeGaussianEvaluation(
    const GaussianEvaluationRequest& request, const ReconstructionView& view,
    const RigidTransform& camera_from_body) {
  ValidateGaussianSourceRequest(request.sources);
  Require(view.keyframe_id == request.sources.query_keyframe_id &&
              request.camera.keyframe_id == view.keyframe_id,
          "Evaluation observation differs from captured query");
  auto body = EncodeGaussianSourceBody(request.sources);
  const auto pixels = gaussian_mapper_io::ViewBody(view);
  body.insert(body.end(), pixels.begin(), pixels.end());
  return {Json([&](auto& out) {
            Base(out, "heldout_evaluate", request.sources.session_id, 2);
            Id(out, "job_id", request.sources.job_id);
            out << "split_id" << request.split_id << "sealed_at_unix_seconds"
                << request.sealed_at_unix_seconds
                << "dispatched_at_unix_seconds"
                << request.dispatched_at_unix_seconds << "sources" << "{";
            WriteGaussianSourceHeader(out, request.sources, "gaussian_sources");
            out << "}" << "query" << "{";
            Id(out, "keyframe_id", view.keyframe_id);
            Id(out, "calibration_id", view.calibration_id);
            out << "width" << view.image_bgr.cols << "height"
                << view.image_bgr.rows << "fx" << view.camera.fx << "fy"
                << view.camera.fy << "cx" << view.camera.cx << "cy"
                << view.camera.cy << "state_timestamp" << view.state_timestamp
                << "cloud_timestamp" << view.cloud_timestamp
                << "image_timestamp" << view.image_timestamp
                << "inverse_exposure_time" << view.inverse_exposure_time;
            Transform(out, "camera_from_body", camera_from_body);
            out << "}";
          }),
          std::move(body)};
}

void ValidateGaussianEvaluationMessage(const cv::FileNode& root,
                                       const std::vector<std::uint8_t>& body) {
  Require(static_cast<int>(root["schema"]) == 2 &&
              Hex(root["session_id"], 32) && Hex(root["split_id"], 64) &&
              ReadId(root["job_id"]) > 0,
          "Invalid evaluation identity");
  const auto kind = static_cast<std::string>(root["type"]);
  if (kind == "heldout_evaluate") {
    Keys(root,
         {"schema", "type", "session_id", "job_id", "split_id", "sources",
          "query", "sealed_at_unix_seconds", "dispatched_at_unix_seconds"});
    const auto source = root["sources"], query = root["query"];
    Require(static_cast<std::string>(source["type"]) == "gaussian_sources" &&
                static_cast<std::string>(source["session_id"]) ==
                    static_cast<std::string>(root["session_id"]) &&
                ReadId(source["job_id"]) == ReadId(root["job_id"]),
            "Evaluation source envelope differs");
    Keys(query,
         {"width", "height", "keyframe_id", "calibration_id", "fx", "fy", "cx",
          "cy", "state_timestamp", "cloud_timestamp", "image_timestamp",
          "inverse_exposure_time", "camera_from_body"});
    Require(query["width"].isInt() && query["height"].isInt(),
            "Invalid evaluation image size");
    const int width = query["width"], height = query["height"];
    Require(width > 0 && height > 0 &&
                static_cast<std::int64_t>(width) * height <= 1000000,
            "Evaluation exceeds pixel budget");
    const auto pixels = static_cast<std::size_t>(width) * height;
    std::size_t bytes = 128;
    for (const auto& block : source["blocks"]) {
      Require(block["target"]["primary_count"].isInt(),
              "Invalid evaluation primary count");
      const int count = block["target"]["primary_count"];
      Require(count > 0 && count <= 512, "Evaluation primary budget exceeded");
      bytes += 128 * count;
    }
    Require(bytes <= 131200 && body.size() == bytes + 9 * pixels,
            "Invalid evaluation body size");
    const auto request =
        DecodeGaussianSources(source, {body.begin(), body.begin() + bytes});
    GaussianCamera camera;
    camera.keyframe_id = ReadId(query["keyframe_id"]);
    camera.calibration_id = ReadId(query["calibration_id"]);
    camera.width = width;
    camera.height = height;
    camera.K << Number(query["fx"]), 0, Number(query["cx"]), 0,
        Number(query["fy"]), Number(query["cy"]), 0, 0, 1;
    camera.camera_from_body = Pose(query["camera_from_body"]);
    camera.capture_time = Number(query["image_timestamp"]);
    ValidateRenderCamera(camera);
    if (request.policy == GaussianRenderPolicy::kSpatial) {
      const auto& retained = request.blocks.back().render_metadata->camera;
      Require(camera.width == retained.width &&
                  camera.height == retained.height,
              "Evaluation camera dimensions differ from spatial owner");
    }
    Require(camera.keyframe_id == request.query_keyframe_id &&
                camera.calibration_id == 0,
            "Evaluation query identity differs");
    for (const auto* name :
         {"state_timestamp", "cloud_timestamp", "inverse_exposure_time"})
      (void)Number(query[name]);
    Require(Number(root["sealed_at_unix_seconds"]) > 0 &&
                Number(root["dispatched_at_unix_seconds"]) >=
                    Number(root["sealed_at_unix_seconds"]),
            "Invalid evaluation event times");
    for (std::size_t i = 0; i < pixels; ++i) {
      const auto image = body[bytes + 3 * pixels + i],
                 depth_mask = body[bytes + 4 * pixels + i];
      std::uint32_t bits = 0;
      for (int j = 0; j < 4; ++j)
        bits |= std::uint32_t(body[bytes + 5 * pixels + 4 * i + j]) << (8 * j);
      float depth;
      std::memcpy(&depth, &bits, 4);
      Require(image <= 1 && depth_mask <= image && std::isfinite(depth) &&
                  (depth_mask ? depth > 0 : depth == 0),
              "Invalid evaluation depth or mask");
    }
    return;
  }
  std::set<std::string> fields = {"schema",      "type",     "session_id",
                                  "job_id",      "split_id", "keyframe_id",
                                  "map_sequence"};
  Require(body.empty() && ReadId(root["map_sequence"]) > 0,
          "Invalid evaluation reply body/frontier");
  (void)ReadId(root["keyframe_id"]);
  if (kind == "heldout_evaluation_ok") {
    fields.insert({"report_sha256", "report_bytes"});
    Keys(root, fields);
    Require(Hex(root["report_sha256"], 64) &&
                ReadId(root["report_bytes"]) > 0 &&
                ReadId(root["report_bytes"]) <= 2 * 1024 * 1024,
            "Invalid evaluation report identity");
  } else {
    fields.insert({"code", "message"});
    Keys(root, fields);
    const auto code = static_cast<std::string>(root["code"]);
    Require(kind == "heldout_evaluation_error" &&
                (code == "oom" || code == "source" || code == "storage" ||
                 code == "budget") &&
                root["message"].isString() &&
                static_cast<std::string>(root["message"]).size() <= 2048,
            "Invalid evaluation error");
  }
}

std::string CheckGaussianEvaluationReply(
    const GaussianMessage& message, const GaussianEvaluationRequest& request) {
  cv::FileStorage json(message.header_json,
                       cv::FileStorage::READ | cv::FileStorage::MEMORY);
  const auto& source = request.sources;
  Require(static_cast<std::string>(json["session_id"]) == source.session_id &&
              ReadId(json["job_id"]) == source.job_id &&
              ReadId(json["keyframe_id"]) == source.query_keyframe_id &&
              ReadId(json["map_sequence"]) == source.map_sequence &&
              static_cast<std::string>(json["split_id"]) == request.split_id,
          "Evaluation receipt identity mismatch");
  return static_cast<std::string>(json["type"]) == "heldout_evaluation_ok"
             ? ""
             : static_cast<std::string>(json["message"]);
}

}  // namespace livo_atlas
