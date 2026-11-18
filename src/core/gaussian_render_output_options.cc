#include "livo_atlas/core/gaussian_render_output_options.h"

#include <algorithm>
#include <stdexcept>

#include "livo_atlas/core/global_output_options.h"

namespace livo_atlas {

void ValidateGaussianRenderOutputOptions(
    const GaussianRenderOutputOptions& options, const std::string& map_frame) {
  if (!options.enabled) return;
  GlobalOutputOptions parent;
  parent.enabled = true;
  parent.map_frame = map_frame;
  ValidateGlobalOutputOptions(parent);
  if (map_frame.size() > 256)
    throw std::invalid_argument("Gaussian parent frame exceeds 256 bytes");
  const auto& frame = options.optical_frame;
  if (options.minimum_interval.count() < 100 ||
      options.minimum_interval.count() > 60000 || frame.empty() ||
      frame.size() > 128 || frame.front() == '/' || frame.back() == '/' ||
      frame.find("//") != std::string::npos || frame == "camera_init" ||
      frame == "aft_mapped" || frame == "world" || frame == map_frame ||
      !std::all_of(frame.begin(), frame.end(), [](unsigned char c) {
        return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
               (c >= '0' && c <= '9') || c == '_' || c == '/';
      }))
    throw std::invalid_argument("Invalid Gaussian preview frame or interval");
}

}  // namespace livo_atlas
