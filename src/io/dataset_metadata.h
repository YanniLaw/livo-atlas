#ifndef LIVO_ATLAS_SRC_IO_DATASET_METADATA_H_
#define LIVO_ATLAS_SRC_IO_DATASET_METADATA_H_

#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <opencv2/core/persistence.hpp>
#include <stdexcept>
#include <string>

#include "livo_atlas/sensor/extrinsics.h"

namespace livo_atlas {
namespace dataset_metadata {
[[noreturn]] void Fail(const std::filesystem::path& path,
                       const std::string& message);
std::string Decimal(double value);
std::uint64_t ParseId(const std::string& text);
std::string String(const cv::FileNode& node);
std::uint64_t Id(const cv::FileNode& node);
int Integer(const cv::FileNode& node);
double Number(const cv::FileNode& node);
void Sequence(const cv::FileNode& node, std::size_t size);
void WriteTransform(cv::FileStorage& output, const std::string& name,
                    const RigidTransform& pose);
RigidTransform ReadTransform(const cv::FileNode& node);

template <typename Validate>
void ValidateInput(const std::filesystem::path& path, Validate validate) {
  try {
    validate();
  } catch (const std::invalid_argument& error) {
    throw std::invalid_argument(path.string() + ": " + error.what());
  }
}

template <typename Write>
void WriteYaml(const std::filesystem::path& path, Write write) {
  cv::FileStorage yaml("metadata.yml", cv::FileStorage::WRITE |
                                           cv::FileStorage::MEMORY |
                                           cv::FileStorage::FORMAT_YAML);
  write(yaml);
  const auto text = yaml.releaseAndGetString();
  std::ofstream output;
  output.exceptions(std::ios::badbit | std::ios::failbit);
  try {
    output.open(path, std::ios::binary);
    output.write(text.data(), text.size());
    output.close();
  } catch (const std::exception& error) {
    Fail(path, error.what());
  }
}
}  // namespace dataset_metadata
}  // namespace livo_atlas

#endif  // LIVO_ATLAS_SRC_IO_DATASET_METADATA_H_
