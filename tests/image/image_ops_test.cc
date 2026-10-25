#include "livo_atlas/image/image_ops.h"

#include <gtest/gtest.h>

#include <fstream>
#include <limits>
#include <opencv2/core/mat.hpp>
#include <sstream>
#include <stdexcept>
#include <string>

#include "test_images.h"

namespace livo_atlas {
namespace {

TEST(ImageOpsTest, MatchesFrozenContiguousAndRoiSamples) {
  std::ifstream input(std::string(TEST_DATA_DIR) +
                      "/image/data/legacy_image.csv");
  ASSERT_TRUE(input.is_open());
  int rows = 0;
  for (std::string line; std::getline(input, line);) {
    SCOPED_TRACE(line);
    std::istringstream row(line);
    std::string operation, roi, x, y, value;
    std::getline(row, operation, ',');
    std::getline(row, roi, ',');
    std::getline(row, x, ',');
    std::getline(row, y, ',');
    std::getline(row, value, ',');
    const auto image = test_data::MakeImage(128, 64, 1, std::stoi(roi));
    const float actual =
        operation == "I" ? InterpolateGray8(image, std::stof(x), std::stof(y))
                         : ShiTomasiScore(image, std::stoi(x), std::stoi(y));
    // Native FMA changes the frozen score by at most 2 ULP. Interpolation
    // and the old source compiled for the baseline ISA are bit exact.
    EXPECT_FLOAT_EQ(actual, std::stof(value));
    ++rows;
  }
  EXPECT_EQ(rows, 194);
}

TEST(ImageOpsTest, InterpolatesHandCalculatedStridedGradient) {
  cv::Mat storage(8, 20, CV_8UC1);
  for (int y = 0; y < storage.rows; ++y) {
    for (int x = 0; x < storage.cols; ++x) {
      storage.at<uchar>(y, x) = 3 * x + 5 * y;
    }
  }
  const cv::Mat roi = storage(cv::Rect(2, 1, 8, 6));
  ASSERT_FALSE(roi.isContinuous());
  EXPECT_FLOAT_EQ(InterpolateGray8(roi, 2.25F, 3.5F), 35.25F);
}

TEST(ImageOpsTest, RejectsInvalidTypeAndMissingInterpolationNeighborhood) {
  const auto image = test_data::MakeImage(16, 16, 1);
  EXPECT_THROW(InterpolateGray8(image, -0.1F, 3), std::out_of_range);
  EXPECT_THROW(InterpolateGray8(image, 15, 3), std::out_of_range);
  EXPECT_THROW(InterpolateGray8(image, 3, 15), std::out_of_range);
  EXPECT_THROW(
      InterpolateGray8(image, std::numeric_limits<float>::infinity(), 3),
      std::out_of_range);
  EXPECT_THROW(InterpolateGray8(cv::Mat(), 0, 0), std::invalid_argument);
  EXPECT_THROW(ShiTomasiScore(test_data::MakeImage(16, 16, 3), 8, 8),
               std::invalid_argument);
  EXPECT_EQ(ShiTomasiScore(image, std::numeric_limits<int>::max(), 8), 0);
}

}  // namespace
}  // namespace livo_atlas
