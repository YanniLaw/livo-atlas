#ifndef LIVO_ATLAS_CORE_GAUSSIAN_RENDER_OUTPUT_OPTIONS_H_
#define LIVO_ATLAS_CORE_GAUSSIAN_RENDER_OUTPUT_OPTIONS_H_

#include <chrono>
#include <string>

namespace livo_atlas {

struct GaussianRenderOutputOptions {
  bool enabled = false;
  std::string optical_frame = "gaussian_camera_optical";
  std::chrono::milliseconds minimum_interval{1000};
};

// Pure validation; does not touch files or middleware.
void ValidateGaussianRenderOutputOptions(
    const GaussianRenderOutputOptions& options,
    const std::string& map_frame = "map");

}  // namespace livo_atlas
#endif  // LIVO_ATLAS_CORE_GAUSSIAN_RENDER_OUTPUT_OPTIONS_H_
