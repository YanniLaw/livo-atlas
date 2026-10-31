#ifndef LIVO_ATLAS_IO_IMAGE_WRITER_H_
#define LIVO_ATLAS_IO_IMAGE_WRITER_H_

#include <filesystem>
#include <opencv2/core/mat.hpp>

namespace livo_atlas {

// Uses the filename extension and reports encoding, write and close failures.
// The caller owns parent directories and any transaction/completion policy.
void WriteImageFile(const std::filesystem::path& path, const cv::Mat& image);

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_IO_IMAGE_WRITER_H_
