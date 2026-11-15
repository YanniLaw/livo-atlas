#include "gaussian/wire_protocol.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include <opencv2/core.hpp>

#include "gaussian/unix_socket.h"

namespace livo_atlas {
namespace {

constexpr std::size_t kPrefixBytes = 12;
constexpr std::size_t kMaxHeaderBytes = 64 * 1024;
constexpr std::size_t kMaxBodyBytes = 16 * 1024 * 1024;
constexpr int kMaxPixels = 1000000;

void Require(bool condition, const char* message) {
  if (!condition) throw std::invalid_argument(message);
}

bool ValidUtf8(const std::string& text) {
  for (std::size_t i = 0; i < text.size();) {
    const auto first = static_cast<std::uint8_t>(text[i++]);
    if (first < 0x80) continue;
    int continuation = 0;
    std::uint32_t value = 0;
    std::uint32_t minimum = 0;
    if (first >= 0xc2 && first <= 0xdf) {
      continuation = 1;
      value = first & 0x1f;
      minimum = 0x80;
    } else if (first >= 0xe0 && first <= 0xef) {
      continuation = 2;
      value = first & 0x0f;
      minimum = 0x800;
    } else if (first >= 0xf0 && first <= 0xf4) {
      continuation = 3;
      value = first & 0x07;
      minimum = 0x10000;
    } else {
      return false;
    }
    for (int j = 0; j < continuation; ++j) {
      if (i >= text.size()) return false;
      const auto byte = static_cast<std::uint8_t>(text[i++]);
      if ((byte & 0xc0) != 0x80) return false;
      value = (value << 6) | (byte & 0x3f);
    }
    if (value < minimum || value > 0x10ffff ||
        (value >= 0xd800 && value <= 0xdfff)) {
      return false;
    }
  }
  return true;
}

bool JsonSpace(char value) {
  return value == ' ' || value == '\t' || value == '\n' || value == '\r';
}

bool SingleJsonObject(const std::string& text) {
  std::size_t start = 0;
  while (start < text.size() && JsonSpace(text[start])) ++start;
  if (start == text.size() || text[start] != '{') return false;
  int depth = 0;
  bool quoted = false;
  bool escaped = false;
  for (std::size_t i = start; i < text.size(); ++i) {
    const char value = text[i];
    if (quoted) {
      if (static_cast<std::uint8_t>(value) < 0x20) return false;
      if (escaped) {
        escaped = false;
      } else if (value == '\\') {
        escaped = true;
      } else if (value == '"') {
        quoted = false;
      }
      continue;
    }
    if (value == '"') {
      quoted = true;
    } else if (value == '{' || value == '[') {
      ++depth;
    } else if (value == '}' || value == ']') {
      if (--depth < 0) return false;
      if (depth == 0) {
        return std::all_of(text.begin() + i + 1, text.end(), JsonSpace);
      }
    }
  }
  return false;
}

std::string String(const cv::FileNode& node, const char* name) {
  Require(node.isString(), name);
  return static_cast<std::string>(node);
}

std::uint64_t Unsigned(const cv::FileNode& node, const char* name) {
  const auto value = String(node, name);
  std::uint64_t result = 0;
  const auto parsed =
      std::from_chars(value.data(), value.data() + value.size(), result);
  Require(!value.empty() && parsed.ec == std::errc() &&
              parsed.ptr == value.data() + value.size() &&
              value == std::to_string(result),
          name);
  return result;
}

int Integer(const cv::FileNode& node, const char* name, int minimum,
            int maximum) {
  Require(node.isInt(), name);
  const int value = static_cast<int>(node);
  Require(value >= minimum && value <= maximum, name);
  return value;
}

double Number(const cv::FileNode& node, const char* name) {
  Require(node.isInt() || node.isReal(), name);
  const double value = static_cast<double>(node);
  Require(std::isfinite(value), name);
  return value;
}

void CheckUniqueKeys(const cv::FileNode& node) {
  if (node.isMap()) {
    std::set<std::string> names;
    for (const auto& child : node) {
      Require(names.insert(child.name()).second, "Duplicate JSON key");
      CheckUniqueKeys(child);
    }
  } else if (node.isSeq()) {
    for (const auto& child : node) CheckUniqueKeys(child);
  }
}

void Pose(const cv::FileNode& node, const char* name) {
  Require(node.isSeq() && node.size() == 16, name);
  std::array<double, 16> matrix{};
  std::size_t index = 0;
  for (const auto& value : node) matrix[index++] = Number(value, name);
  for (int col = 0; col < 4; ++col) {
    Require(std::abs(matrix[12 + col] - (col == 3 ? 1.0 : 0.0)) <= 1e-9, name);
  }
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      double dot = 0;
      for (int k = 0; k < 3; ++k)
        dot += matrix[4 * k + row] * matrix[4 * k + col];
      Require(std::abs(dot - (row == col ? 1.0 : 0.0)) <= 1e-6, name);
    }
  }
  const double determinant =
      matrix[0] * (matrix[5] * matrix[10] - matrix[6] * matrix[9]) -
      matrix[1] * (matrix[4] * matrix[10] - matrix[6] * matrix[8]) +
      matrix[2] * (matrix[4] * matrix[9] - matrix[5] * matrix[8]);
  Require(std::abs(determinant - 1.0) <= 1e-6, name);
}

std::size_t Pixels(const cv::FileNode& root) {
  const int width = Integer(root["width"], "Invalid width", 1, kMaxPixels);
  const int height = Integer(root["height"], "Invalid height", 1, kMaxPixels);
  Require(width <= kMaxPixels / height, "Image exceeds the pixel limit");
  const auto pixels = static_cast<std::size_t>(width) * height;
  Require(pixels <= kMaxPixels, "Image exceeds the pixel limit");
  return pixels;
}

void View(const cv::FileNode& root, const std::vector<std::uint8_t>& body) {
  const auto pixels = Pixels(root);
  Require(body.size() == 9 * pixels, "Incorrect view body size");
  const auto frame = Unsigned(root["keyframe_id"], "Invalid keyframe ID");
  const auto count = Unsigned(root["frame_count"], "Invalid frame count");
  const auto optimized =
      Unsigned(root["optimized_frame_count"], "Invalid optimized count");
  const auto calibration =
      Unsigned(root["calibration_id"], "Invalid calibration ID");
  const auto version = Unsigned(root["pose_version"], "Invalid pose version");
  const auto parent = String(root["parent_frame"], "Invalid parent frame");
  Require(frame < count && optimized <= count && calibration == 0 &&
              parent == (version == 0 ? "odom" : "map"),
          "Invalid view identity or pose frontier");
  for (const char* name :
       {"state_timestamp", "cloud_timestamp", "image_timestamp",
        "inverse_exposure_time", "cx", "cy"}) {
    (void)Number(root[name], name);
  }
  Require(Number(root["fx"], "Invalid fx") > 0 &&
              Number(root["fy"], "Invalid fy") > 0,
          "Focal lengths must be positive");
  Pose(root["camera_from_submap"], "Invalid camera pose");
  Pose(root["parent_from_submap"], "Invalid submap pose");
  Require(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
          "Depth requires IEEE-754 float32");
  for (std::size_t i = 0; i < pixels; ++i) {
    const auto image_mask = body[3 * pixels + i];
    const auto depth_mask = body[4 * pixels + i];
    const auto offset = 5 * pixels + 4 * i;
    std::uint32_t bits = 0;
    for (unsigned int byte = 0; byte < 4; ++byte) {
      bits |= static_cast<std::uint32_t>(body[offset + byte]) << (8 * byte);
    }
    float depth = 0;
    std::memcpy(&depth, &bits, sizeof(bits));
    Require(image_mask <= 1 && depth_mask <= image_mask &&
                std::isfinite(depth) && (depth_mask ? depth > 0 : depth == 0),
            "Invalid view masks or depth");
  }
}

void Candidate(const cv::FileNode& root,
               const std::vector<std::uint8_t>& body) {
  const auto pixels = Pixels(root);
  Require(body.size() == 3 * pixels, "Incorrect preview body size");
  const auto frames = root["frame_ids"];
  Require(frames.isSeq() && frames.size() >= 1 && frames.size() <= 10,
          "Invalid candidate frame IDs");
  const auto count = Unsigned(root["frame_count"], "Invalid frame count");
  Require(Unsigned(root["optimized_frame_count"], "Invalid optimized count") <=
              count,
          "Invalid candidate pose frontier");
  std::uint64_t previous = 0;
  bool first = true;
  for (const auto& node : frames) {
    const auto id = Unsigned(node, "Invalid frame ID");
    Require(id < count && (first || id > previous),
            "Candidate frame IDs must be sorted and unique");
    previous = id;
    first = false;
  }
  (void)Integer(root["steps"], "Invalid steps", 20, 20);
  (void)Integer(root["gaussian_count"], "Invalid Gaussian count", 1, 10000);
  (void)Integer(root["supervised_depth_pixels"], "Invalid depth count", 0,
                20 * kMaxPixels);
  Require(Number(root["rgb_l1"], "Invalid RGB loss") >= 0 &&
              Number(root["depth_l1_m"], "Invalid depth loss") >= 0,
          "Losses must be nonnegative");
}

void Validate(const GaussianMessage& message) {
  // OpenCV 4.2 accepts trailing bytes and invalid UTF-8, so check framing
  // first.
  Require(!message.header_json.empty() &&
              message.header_json.size() <= kMaxHeaderBytes &&
              message.body.size() <= kMaxBodyBytes &&
              message.header_json.find('\0') == std::string::npos &&
              ValidUtf8(message.header_json) &&
              SingleJsonObject(message.header_json),
          "Message size exceeds the limit");
  try {
    cv::FileStorage json(message.header_json, cv::FileStorage::READ |
                                                  cv::FileStorage::MEMORY |
                                                  cv::FileStorage::FORMAT_JSON);
    Require(json.isOpened(), "Invalid JSON header");
    const auto root = json.root();
    Require(root.isMap(), "Invalid message header");
    CheckUniqueKeys(root);
    (void)Integer(root["schema"], "Invalid schema", 1, 1);
    const auto kind = String(root["type"], "Invalid message type");
    const std::set<std::string> types = {
        "hello",    "hello_ok",  "reset",      "reset_ok", "view", "candidate",
        "decision", "commit_ok", "discard_ok", "error",    "stop"};
    Require(types.count(kind), "Unsupported message type");
    const auto session = String(root["session_id"], "Invalid session ID");
    Require(session.size() == 32 &&
                std::all_of(session.begin(), session.end(),
                            [](char value) {
                              return (value >= '0' && value <= '9') ||
                                     (value >= 'a' && value <= 'f');
                            }),
            "Invalid session ID");
    if (kind == "reset" || kind == "reset_ok") {
      (void)Unsigned(root["generation"], "Invalid generation");
      (void)Unsigned(root["pose_version"], "Invalid pose version");
      if (!root["submap_id"].empty()) {
        (void)Unsigned(root["submap_id"], "Invalid submap ID");
      }
    }
    if (kind == "view" || kind == "candidate" || kind == "decision" ||
        kind == "commit_ok" || kind == "discard_ok") {
      for (const char* name :
           {"job_id", "generation", "submap_id", "pose_version"}) {
        (void)Unsigned(root[name], name);
      }
    }
    if (kind == "view" || kind == "candidate") {
      (void)Unsigned(root["membership_revision"],
                     "Invalid membership revision");
    }
    if (kind == "view") {
      View(root, message.body);
    } else if (kind == "candidate") {
      Candidate(root, message.body);
    } else {
      Require(message.body.empty(), "Unexpected body");
    }
    if (kind == "decision") {
      const auto action = String(root["action"], "Invalid decision");
      Require(action == "accept" || action == "discard", "Invalid decision");
    }
    if (kind == "error") {
      const auto code = String(root["code"], "Invalid worker error");
      Require(code == "gpu_unavailable" || code == "extension_unavailable" ||
                  code == "protocol_error" || code == "worker_error",
              "Invalid worker error");
    }
    if (kind == "hello_ok") {
      for (const char* name : {"python", "torch", "gsplat", "cuda", "device"}) {
        Require(!String(root[name], name).empty(), "Invalid worker metadata");
      }
      const auto free =
          Unsigned(root["gpu_free_bytes"], "Invalid free GPU bytes");
      const auto total =
          Unsigned(root["gpu_total_bytes"], "Invalid total GPU bytes");
      Require(total && free <= total, "Invalid GPU memory metadata");
    }
  } catch (const cv::Exception& error) {
    throw std::invalid_argument(std::string("Invalid JSON header: ") +
                                error.what());
  }
}

gaussian_detail::Deadline MakeDeadline(std::chrono::milliseconds timeout) {
  Require(timeout.count() > 0, "Timeout must be positive");
  return std::chrono::steady_clock::now() + timeout;
}

std::uint64_t ReadLittleEndian(const std::uint8_t* bytes, int count) {
  std::uint64_t value = 0;
  for (int i = 0; i < count; ++i) {
    value |= static_cast<std::uint64_t>(bytes[i]) << (8 * i);
  }
  return value;
}

void AppendLittleEndian(std::uint64_t value, int count,
                        std::vector<std::uint8_t>* result) {
  for (int i = 0; i < count; ++i) {
    result->push_back(static_cast<std::uint8_t>((value >> (8 * i)) & 0xff));
  }
}

}  // namespace

std::vector<std::uint8_t> EncodeGaussianMessage(
    const GaussianMessage& message) {
  Validate(message);
  std::vector<std::uint8_t> result;
  result.reserve(kPrefixBytes + message.header_json.size() +
                 message.body.size());
  AppendLittleEndian(message.header_json.size(), 4, &result);
  AppendLittleEndian(message.body.size(), 8, &result);
  result.insert(result.end(), message.header_json.begin(),
                message.header_json.end());
  result.insert(result.end(), message.body.begin(), message.body.end());
  return result;
}

GaussianMessage ReadGaussianMessage(int fd, std::chrono::milliseconds timeout) {
  const auto deadline = MakeDeadline(timeout);
  std::array<std::uint8_t, kPrefixBytes> prefix{};
  gaussian_detail::ReadExact(fd, prefix.data(), prefix.size(), deadline);
  const auto header_size = ReadLittleEndian(prefix.data(), 4);
  const auto body_size = ReadLittleEndian(prefix.data() + 4, 8);
  Require(header_size > 0 && header_size <= kMaxHeaderBytes &&
              body_size <= kMaxBodyBytes,
          "Message size exceeds the limit");
  GaussianMessage result;
  result.header_json.resize(static_cast<std::size_t>(header_size));
  result.body.resize(static_cast<std::size_t>(body_size));
  gaussian_detail::ReadExact(
      fd, reinterpret_cast<std::uint8_t*>(result.header_json.data()),
      result.header_json.size(), deadline);
  gaussian_detail::ReadExact(fd, result.body.data(), result.body.size(),
                             deadline);
  Validate(result);
  return result;
}

void WriteGaussianMessage(int fd, const GaussianMessage& message,
                          std::chrono::milliseconds timeout) {
  const auto deadline = MakeDeadline(timeout);
  const auto bytes = EncodeGaussianMessage(message);
  gaussian_detail::WriteExact(fd, bytes.data(), bytes.size(), deadline);
}

}  // namespace livo_atlas
