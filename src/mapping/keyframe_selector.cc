#include "livo_atlas/mapping/keyframe_selector.h"

#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>

#include "keyframe_source.h"

namespace livo_atlas {
void ValidateKeyframeOptions(const KeyframeOptions& options) {
  for (double value :
       {options.translation_threshold, options.rotation_threshold_deg,
        options.time_threshold}) {
    if (!std::isfinite(value) || value < 0) {
      throw std::invalid_argument(
          "Keyframe thresholds must be finite and nonnegative");
    }
  }
}

KeyframeSelector::KeyframeSelector(FrontendMode mode,
                                   const KeyframeOptions& options)
    : mode_(mode), options_(options) {
  ValidateKeyframeOptions(options_);
}

bool KeyframeSelector::Select(const FrontendResult& result) {
  const auto stage =
      mode_ == FrontendMode::kLidarInertialVisual ? UpdateKind::kVisual
      : mode_ == FrontendMode::kLidarInertial     ? UpdateKind::kLidarInertial
                                                  : UpdateKind::kLidarOnly;
  if (!options_.enabled || !result.updated || result.kind != stage ||
      !result.world_cloud || result.world_cloud->empty()) {
    return false;
  }
  const auto candidate = KeyframeView(result);
  if (last_pose_ && result.timestamp == last_time_) return false;
  bool selected = !last_pose_ || result.timestamp < last_time_;
  if (!selected) {
    const double translation =
        (candidate.odom_from_body.translation - last_pose_->translation).norm();
    const double angle = Eigen::AngleAxisd(last_pose_->rotation.transpose() *
                                           candidate.odom_from_body.rotation)
                             .angle();
    selected = translation >= options_.translation_threshold ||
               angle >= options_.rotation_threshold_deg * std::acos(-1) / 180 ||
               result.timestamp - last_time_ >= options_.time_threshold;
  }
  if (selected) {
    last_pose_ = candidate.odom_from_body;
    last_time_ = result.timestamp;
  }
  return selected;
}
}  // namespace livo_atlas
