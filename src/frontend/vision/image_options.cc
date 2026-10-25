#include "livo_atlas/frontend/vision/image_options.h"

#include <cstdint>
#include <limits>

namespace livo_atlas {
std::string ValidateImageOptions(const CameraConfig& camera,
                                 const VisualOptions& visual_options) {
  const std::string error = ValidateCameraConfig(camera);
  if (!error.empty()) return error;
  const int width = static_cast<int>(camera.width * camera.scale);
  const int height = static_cast<int>(camera.height * camera.scale);
  int cell_size = visual_options.grid_size;
  if (cell_size <= 0) {
    return "vio/grid_size must be positive";
  }
  if (cell_size <= 10) {
    if (visual_options.grid_rows <= 0) {
      return "vio/grid_n_height must be positive";
    }
    cell_size = height / visual_options.grid_rows;
  }
  if (cell_size == 0 || width / cell_size == 0 || height / cell_size == 0) {
    return "VIO grid is incompatible with camera dimensions";
  }
  if (visual_options.patch_size <= 0 || visual_options.patch_size % 2 != 0) {
    return "vio/patch_size must be a positive even number";
  }
  // Search can add two to the largest pyramid index (levels - 1).
  if (visual_options.patch_pyramid_levels <= 0 ||
      visual_options.patch_pyramid_levels > 28) {
    return "vio/patch_pyrimid_level is outside the index range";
  }
  constexpr int kMaxIndex = std::numeric_limits<int>::max();
  const std::int64_t patch_area =
      std::int64_t{visual_options.patch_size} * visual_options.patch_size;
  if (patch_area > kMaxIndex / visual_options.patch_pyramid_levels) {
    return "vio/patch_size exceeds the patch index range";
  }
  const std::int64_t scale = std::int64_t{1}
                             << (visual_options.patch_pyramid_levels + 1);
  if ((visual_options.patch_size / 2 + 1) > kMaxIndex / scale ||
      width > kMaxIndex / scale) {
    return "vio/patch_pyrimid_level exceeds image index bounds";
  }
  // Preserve VIO's floor grid counts. Truncated tails must be excluded by its
  // image border, so every accepted pixel has an allocated row and column.
  const std::int64_t border = (visual_options.patch_size / 2 + 1) * (scale / 2);
  if (width % cell_size > border || height % cell_size > border) {
    return "VIO grid does not cover the accepted image area";
  }
  return {};
}
}  // namespace livo_atlas
