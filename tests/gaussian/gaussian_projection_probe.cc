#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

#include <opencv2/core.hpp>

#include "gaussian/projection_envelope.h"
#include "gaussian/render_metadata_protocol.h"
#include "livo_atlas/gaussian/gaussian_projection.h"

// Test-only process boundary for the CUDA comparison. It runs the production
// envelope and exposes its rectangles without enlarging the public SDK DTO.
int main() {
  try {
    std::string line;
    while (std::getline(std::cin, line)) {
      if (line.empty() || line.size() > 65536)
        throw std::invalid_argument("Invalid projection fixture size");
      cv::FileStorage input(line,
                            cv::FileStorage::READ | cv::FileStorage::MEMORY);
      const livo_atlas::GaussianContentId content{
          std::string(32, 'a'), 0, 1, 1};
      const auto metadata = livo_atlas::DecodeGaussianRenderMetadata(
          input["metadata"], content, {0});
      const double bound = input["coordinate_abs_upper"];
      const auto& camera = metadata.camera;
      const auto result = livo_atlas::ProjectGaussianExtents(
          metadata.extents, camera, camera.camera_from_body, bound);
      std::cout << "{\"candidate_pixels\":" << result.candidate_pixels
                << ",\"uncertain\":" << static_cast<int>(result.uncertain)
                << ",\"rectangles\":[";
      bool first = true;
      for (const auto& extent : metadata.extents) {
        const auto rectangle = livo_atlas::ProjectGaussianEnvelope(
            extent, camera, camera.camera_from_body, bound);
        if (!first) std::cout << ',';
        first = false;
        std::cout << '[' << rectangle.left << ',' << rectangle.top << ','
                  << rectangle.right << ',' << rectangle.bottom << ']';
      }
      std::cout << "]}\n";
    }
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
