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

bool SamePose(const RigidTransform& a, const RigidTransform& b) {
  return (a.rotation.array() == b.rotation.array()).all() &&
         (a.translation.array() == b.translation.array()).all();
}

}  // namespace

bool operator==(const GaussianRenderMetadata& a,
                const GaussianRenderMetadata& b) {
  if (a.content != b.content || a.extents.size() != b.extents.size())
    return false;
  const auto& x = a.camera;
  const auto& y = b.camera;
  if (x.keyframe_id != y.keyframe_id ||
      x.calibration_id != y.calibration_id || x.width != y.width ||
      x.height != y.height || !(x.K.array() == y.K.array()).all() ||
      !SamePose(x.camera_from_body, y.camera_from_body) ||
      x.capture_time != y.capture_time)
    return false;
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
  Require(camera.keyframe_id == frames.back() && camera.calibration_id == 0 &&
              camera.width > 0 && camera.height > 0 &&
              static_cast<std::int64_t>(camera.width) * camera.height <=
                  1000000 && std::isfinite(camera.capture_time) &&
              camera.K.allFinite() && camera.K(0, 0) > 0 &&
              camera.K(1, 1) > 0 && camera.K(0, 1) == 0 &&
              camera.K(1, 0) == 0 && camera.K(2, 0) == 0 &&
              camera.K(2, 1) == 0 && camera.K(2, 2) == 1,
          "Invalid retained spatial query camera");
  ValidateRenderPose(camera.camera_from_body);
}

}  // namespace livo_atlas
