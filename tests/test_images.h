#ifndef LIVO_ATLAS_TESTS_TEST_IMAGES_H_
#define LIVO_ATLAS_TESTS_TEST_IMAGES_H_

#include <cstdint>

#include <opencv2/core.hpp>

namespace livo_atlas {
namespace test_data {

inline cv::Mat MakeImage(int width, int height, int channels, bool roi = false) {
  cv::Mat storage(height, width + (roi ? 11 : 0),
                  channels == 1 ? CV_8UC1 : CV_8UC3, cv::Scalar::all(233));
  cv::Mat image = storage(cv::Rect(0, 0, width, height));
  for (int y = 0; y < height; ++y) {
    auto* row = image.ptr<unsigned char>(y);
    for (int x = 0; x < width; ++x) {
      for (int channel = 0; channel < channels; ++channel) {
        row[x * channels + channel] =
            (x * 31 + y * 17 + x * y * 13 + channel * 47) % 256;
      }
    }
  }
  return image;
}

inline std::uint64_t ImageHash(const cv::Mat& image) {
  std::uint64_t hash = 14695981039346656037ULL;
  const int row_bytes = image.cols * image.elemSize();
  for (int y = 0; y < image.rows; ++y) {
    const auto* row = image.ptr<unsigned char>(y);
    for (int x = 0; x < row_bytes; ++x) {
      hash ^= row[x];
      hash *= 1099511628211ULL;
    }
  }
  return hash;
}

}  // namespace test_data
}  // namespace livo_atlas

#endif  // LIVO_ATLAS_TESTS_TEST_IMAGES_H_
