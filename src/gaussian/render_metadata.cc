#include "livo_atlas/gaussian/gaussian_map.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "gaussian/render_validation.h"

namespace livo_atlas {
namespace {

void Require(bool condition, const char* detail) {
  if (!condition) throw std::invalid_argument(detail);
}

}  // namespace

bool operator==(const GaussianRenderMetadata& a,
                const GaussianRenderMetadata& b) {
  if (a.content != b.content || a.extents.size() != b.extents.size())
    return false;
  if (!SameRenderCamera(a.camera, b.camera)) return false;
  for (std::size_t i = 0; i < a.extents.size(); ++i) {
    const auto& left = a.extents[i];
    const auto& right = b.extents[i];
    if (left.origin_keyframe_id != right.origin_keyframe_id ||
        left.max_scale != right.max_scale ||
        !(left.min.array() == right.min.array()).all() ||
        !(left.max.array() == right.max.array()).all())
      return false;
  }
  return true;
}

bool operator!=(const GaussianRenderMetadata& a,
                const GaussianRenderMetadata& b) {
  return !(a == b);
}

void ValidateGaussianRenderMetadata(
    const GaussianRenderMetadata& metadata, const GaussianContentId& content,
    const std::vector<std::uint64_t>& frames) {
  Require(metadata.content == content && content.session_id.size() == 32 &&
              content.session_id.find_first_not_of("0123456789abcdef") ==
                  std::string::npos &&
              content.source_generation > 0 && content.content_job_id > 0,
          "Spatial metadata content identity differs");
  Require(!frames.empty() && frames.size() <= 10 && frames.back() < 512 &&
              std::is_sorted(frames.begin(), frames.end()) &&
              std::adjacent_find(frames.begin(), frames.end()) ==
                  frames.end() && metadata.extents.size() == frames.size(),
          "Spatial origins must cover every retained frame");
  for (std::size_t i = 0; i < frames.size(); ++i) {
    const auto& extent = metadata.extents[i];
    Require(extent.origin_keyframe_id == frames[i] &&
                extent.min.allFinite() && extent.max.allFinite() &&
                (extent.min.array() <= extent.max.array()).all() &&
                std::isfinite(extent.max_scale) && extent.max_scale > 0,
            "Invalid Gaussian spatial extent");
  }
  const auto& camera = metadata.camera;
  Require(camera.keyframe_id == frames.back(),
          "Spatial query is not the latest retained camera");
  ValidateRenderCamera(camera);
}

}  // namespace livo_atlas
