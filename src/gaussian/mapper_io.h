#ifndef LIVO_ATLAS_SRC_GAUSSIAN_MAPPER_IO_H_
#define LIVO_ATLAS_SRC_GAUSSIAN_MAPPER_IO_H_

#include <functional>
#include <filesystem>
#include <opencv2/core.hpp>
#include "gaussian/wire_protocol.h"
#include "livo_atlas/gaussian/online_gaussian_mapper.h"
#include "livo_atlas/io/reconstruction_view.h"

namespace livo_atlas {
namespace gaussian_mapper_io {
std::string Json(const std::function<void(cv::FileStorage&)>& write);
void Base(cv::FileStorage& json, const std::string& kind,
          const std::string& session, int schema = 1);
void Id(cv::FileStorage& json, const char* key, std::uint64_t value);
std::uint64_t ReadId(const cv::FileNode& node);
void Transform(cv::FileStorage& json, const char* key,
               const RigidTransform& value);
cv::FileStorage Parse(const GaussianMessage& message, const std::string& type,
                      const std::string& session, int schema = 1);
void WriteFile(const std::filesystem::path& path, const std::string& data);
void CheckIdentity(const cv::FileStorage& json, const GaussianPreview& preview);
void PreviewIdentity(cv::FileStorage& json, const GaussianPreview& preview);
std::vector<std::uint8_t> ViewBody(const ReconstructionView& view);
}  // namespace gaussian_mapper_io
}  // namespace livo_atlas
#endif  // LIVO_ATLAS_SRC_GAUSSIAN_MAPPER_IO_H_
