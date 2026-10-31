#include "image_writer.h"

#include <fstream>
#include <opencv2/imgcodecs.hpp>
#include <stdexcept>
#include <vector>

namespace livo_atlas {

void WriteImageFile(const std::filesystem::path& path, const cv::Mat& image) {
  try {
    std::vector<unsigned char> encoded;
    if (!cv::imencode(path.extension().string(), image, encoded))
      throw std::runtime_error("Cannot encode image");
    std::ofstream output;
    output.exceptions(std::ios::badbit | std::ios::failbit);
    output.open(path, std::ios::binary);
    output.write(reinterpret_cast<const char*>(encoded.data()),
                 static_cast<std::streamsize>(encoded.size()));
    output.close();
  } catch (const std::exception& error) {
    throw std::runtime_error(path.string() + ": " + error.what());
  }
}

}  // namespace livo_atlas
