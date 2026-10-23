#ifndef LIVO_ATLAS_IMAGE_IMAGE_OPS_H_
#define LIVO_ATLAS_IMAGE_IMAGE_OPS_H_

#include <opencv2/core.hpp>

namespace livo_atlas {

// Both operations require CV_8UC1. Interpolation additionally requires the full
// 2x2 neighborhood; invalid sampling coordinates throw std::out_of_range.
float InterpolateGray8(const cv::Mat& image, float u, float v);
// Returns zero when the 8x8 score window and its gradient border do not fit.
float ShiTomasiScore(const cv::Mat& image, int u, int v);

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_IMAGE_IMAGE_OPS_H_
