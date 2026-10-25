#ifndef LIVO_ATLAS_MAPPING_KEYFRAME_SELECTOR_H_
#define LIVO_ATLAS_MAPPING_KEYFRAME_SELECTOR_H_

#include <optional>

#include "livo_atlas/frontend/frontend_result.h"
#include "livo_atlas/mapping/keyframe.h"

namespace livo_atlas {
struct KeyframeOptions {
  bool enabled = false;
  double translation_threshold = .5;
  double rotation_threshold_deg = 10;
  double time_threshold = 2;
};

void ValidateKeyframeOptions(const KeyframeOptions& options);

// Serial, per-system selection. A rejected candidate never moves the anchor.
class KeyframeSelector {
 public:
  KeyframeSelector(FrontendMode mode, const KeyframeOptions& options);
  bool Select(const FrontendResult& result);

 private:
  FrontendMode mode_;
  KeyframeOptions options_;
  std::optional<RigidTransform> last_pose_;
  double last_time_ = 0;
};
}  // namespace livo_atlas

#endif  // LIVO_ATLAS_MAPPING_KEYFRAME_SELECTOR_H_
