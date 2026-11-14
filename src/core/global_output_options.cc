#include "livo_atlas/core/global_output_options.h"

#include <algorithm>
#include <stdexcept>

namespace livo_atlas {

void ValidateGlobalOutputOptions(const GlobalOutputOptions& options) {
  if (!options.enabled) return;
  const auto& frame = options.map_frame;
  if (frame.empty() || frame.front() == '/' || frame.back() == '/' ||
      frame.find("//") != std::string::npos || frame == "camera_init" ||
      frame == "aft_mapped" || frame == "world" ||
      !std::all_of(frame.begin(), frame.end(), [](unsigned char c) {
        return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
               (c >= '0' && c <= '9') || c == '_' || c == '/';
      }))
    throw std::invalid_argument(
        "global_output.map_frame must be a distinct ASCII frame name with "
        "nonempty slash-separated segments");
}

}  // namespace livo_atlas
