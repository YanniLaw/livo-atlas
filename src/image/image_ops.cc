#include "livo_atlas/image/image_ops.h"

#include <cmath>
#include <cstddef>
#include <opencv2/core/mat.hpp>
#include <stdexcept>

namespace livo_atlas {

// Preserve the float arithmetic of Christian Forster's vikit vision helpers.
float InterpolateGray8(const cv::Mat& image, float u, float v) {
  if (image.empty() || image.type() != CV_8UC1) {
    throw std::invalid_argument("InterpolateGray8 requires a grayscale image");
  }
  if (!std::isfinite(u) || !std::isfinite(v) || u < 0 || v < 0 ||
      static_cast<double>(u) >= image.cols - 1 ||
      static_cast<double>(v) >= image.rows - 1) {
    throw std::out_of_range(
        "InterpolateGray8 requires a full 2x2 neighborhood");
  }
  const int x = static_cast<int>(std::floor(u));
  const int y = static_cast<int>(std::floor(v));
  const float sub_x = u - x;
  const float sub_y = v - y;
  const float w00 = (1.f - sub_x) * (1.f - sub_y);
  const float w01 = (1.f - sub_x) * sub_y;
  const float w10 = sub_x * (1.f - sub_y);
  const float w11 = 1.f - w00 - w01 - w10;
  const std::size_t stride = image.step[0];
  const auto* pixel = image.ptr<unsigned char>(y) + x;
  return w00 * pixel[0] + w01 * pixel[stride] + w10 * pixel[1] +
         w11 * pixel[stride + 1];
}

float ShiTomasiScore(const cv::Mat& image, int u, int v) {
  if (image.empty() || image.type() != CV_8UC1) {
    throw std::invalid_argument("ShiTomasiScore requires a grayscale image");
  }
  constexpr int kHalfBox = 4;
  constexpr int kBoxSize = 8;
  constexpr int kBoxArea = kBoxSize * kBoxSize;
  // Check before adding offsets, including for extreme integer coordinates.
  if (u < kHalfBox + 1 || v < kHalfBox + 1 || u >= image.cols - kHalfBox - 1 ||
      v >= image.rows - kHalfBox - 1) {
    return 0;
  }
  float dxx = 0;
  float dyy = 0;
  float dxy = 0;
  const std::ptrdiff_t stride = image.step[0];
  for (int y = v - kHalfBox; y < v + kHalfBox; ++y) {
    const auto* pixel = image.ptr<unsigned char>(y) + u - kHalfBox;
    for (int x = 0; x < kBoxSize; ++x, ++pixel) {
      const float dx = pixel[1] - pixel[-1];
      const float dy = pixel[stride] - pixel[-stride];
      dxx += dx * dx;
      dyy += dy * dy;
      dxy += dx * dy;
    }
  }
  dxx = dxx / (2.0 * kBoxArea);
  dyy = dyy / (2.0 * kBoxArea);
  dxy = dxy / (2.0 * kBoxArea);
  return 0.5 *
         (dxx + dyy -
          std::sqrt((dxx + dyy) * (dxx + dyy) - 4 * (dxx * dyy - dxy * dxy)));
}

}  // namespace livo_atlas
