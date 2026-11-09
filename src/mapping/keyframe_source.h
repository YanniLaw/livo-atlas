#ifndef LIVO_ATLAS_SRC_MAPPING_KEYFRAME_SOURCE_H_
#define LIVO_ATLAS_SRC_MAPPING_KEYFRAME_SOURCE_H_

#include <cmath>
#include <stdexcept>

#include "livo_atlas/frontend/frontend_result.h"
#include "livo_atlas/mapping/keyframe.h"

namespace livo_atlas {
// Private, short-lived view shared by selection and capture. Selection reads
// and validates it without copying image pixels or cloud points.
inline Keyframe KeyframeView(const FrontendResult& result) {
  if (!result.updated || (result.kind != UpdateKind::kVisual &&
                          result.kind != UpdateKind::kLidarOnly &&
                          result.kind != UpdateKind::kLidarInertial)) {
    throw std::invalid_argument("Keyframe requires a successful update");
  }
  Keyframe view;
  view.timestamp = result.timestamp;
  view.cloud_timestamp = result.cloud_timestamp;
  view.odom_from_body = {result.estimate.rot_end, result.estimate.pos_end};
  view.inverse_exposure_time = result.estimate.inv_expo_time;
  view.cloud = result.world_cloud;
  if (result.kind == UpdateKind::kVisual) {
    const auto& rotation = result.camera_from_world_rotation;
    if (!rotation.coeffs().allFinite() ||
        std::abs(rotation.norm() - 1) > 1e-6) {
      throw std::invalid_argument("Invalid keyframe camera rotation");
    }
    view.image_timestamp = result.image_timestamp;
    view.image = result.color_image;
    view.camera_from_odom = {rotation.toRotationMatrix(),
                             result.camera_from_world_translation};
    if (!view.image_timestamp) {
      throw std::invalid_argument("Visual keyframe has no image timestamp");
    }
  }
  ValidateKeyframe(view);
  return view;
}
}  // namespace livo_atlas

#endif  // LIVO_ATLAS_SRC_MAPPING_KEYFRAME_SOURCE_H_
