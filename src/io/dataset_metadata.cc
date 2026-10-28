#include "dataset_metadata.h"

#include <charconv>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>

namespace livo_atlas {
namespace dataset_metadata {
namespace fs = std::filesystem;

[[noreturn]] void Fail(const fs::path& path, const std::string& message) {
  throw std::runtime_error(path.string() + ": " + message);
}

std::string Decimal(double value) {
  std::ostringstream text;
  text.imbue(std::locale::classic());
  text << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
  return text.str();
}

std::uint64_t ParseId(const std::string& text) {
  std::uint64_t value = 0;
  const auto result =
      std::from_chars(text.data(), text.data() + text.size(), value);
  if (text.empty() || result.ec != std::errc() ||
      result.ptr != text.data() + text.size() ||
      std::to_string(value) != text) {
    throw std::invalid_argument("Invalid unsigned identifier: " + text);
  }
  return value;
}

std::string String(const cv::FileNode& node) {
  if (!node.isString()) throw std::invalid_argument("Expected a string field");
  return static_cast<std::string>(node);
}
std::uint64_t Id(const cv::FileNode& node) { return ParseId(String(node)); }
int Integer(const cv::FileNode& node) {
  if (!node.isInt()) throw std::invalid_argument("Expected an integer field");
  return static_cast<int>(node);
}
double Number(const cv::FileNode& node) {
  std::istringstream text(String(node));
  text.imbue(std::locale::classic());
  double value = 0;
  if (!(text >> value) || !std::isfinite(value)) {
    throw std::invalid_argument("Expected a finite decimal string");
  }
  text >> std::ws;
  if (!text.eof()) throw std::invalid_argument("Trailing decimal data");
  return value;
}
void Sequence(const cv::FileNode& node, std::size_t size) {
  if (!node.isSeq() || node.size() != size) {
    throw std::invalid_argument("Incorrect sequence length");
  }
}

void WriteTransform(cv::FileStorage& output, const std::string& name,
                    const RigidTransform& pose) {
  output << name << "{"
         << "rotation"
         << "[";
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      output << Decimal(pose.rotation(row, col));
  output << "]"
         << "translation"
         << "[";
  for (int i = 0; i < 3; ++i) output << Decimal(pose.translation[i]);
  output << "]"
         << "}";
}
RigidTransform ReadTransform(const cv::FileNode& node) {
  RigidTransform pose;
  const auto rotation = node["rotation"], translation = node["translation"];
  Sequence(rotation, 9);
  Sequence(translation, 3);
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      pose.rotation(row, col) = Number(rotation[row * 3 + col]);
  for (int i = 0; i < 3; ++i) pose.translation[i] = Number(translation[i]);
  const auto error = ValidateExtrinsic(pose);
  if (!error.empty()) throw std::invalid_argument(error);
  return pose;
}

}  // namespace dataset_metadata
}  // namespace livo_atlas
