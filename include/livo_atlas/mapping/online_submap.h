#ifndef LIVO_ATLAS_MAPPING_ONLINE_SUBMAP_H_
#define LIVO_ATLAS_MAPPING_ONLINE_SUBMAP_H_

#include <cstdint>
#include <optional>
#include <vector>

#include "livo_atlas/mapping/submap.h"
#include "livo_atlas/sensor/extrinsics.h"

namespace livo_atlas {
enum class SubmapMode { kKeyframeGroups, kOnline };
enum class OnlineSubmapState { kActive, kSealed };
struct OnlineSubmapOptions {
  double translation_threshold = 5.0;
  double rotation_threshold_deg = 45.0;
  double time_threshold = 20.0;
  int support_keyframes = 2;
};
struct OnlineSubmap {
  Submap primary;
  std::vector<std::uint64_t> support_keyframe_ids;
  std::uint64_t membership_revision = 1;
  OnlineSubmapState state = OnlineSubmapState::kActive;
};
void ValidateOnlineSubmapOptions(int max_keyframes,
                                 const OnlineSubmapOptions& options);

// Serial grouping of committed frames. Returned revisions own their metadata.
class OnlineSubmapBuilder {
 public:
  OnlineSubmapBuilder(int max_keyframes, OnlineSubmapOptions options);
  std::vector<OnlineSubmap> AddKeyframe(std::uint64_t id, double timestamp,
                                        const RigidTransform& odom_from_body);
  std::vector<OnlineSubmap> Finish();

 private:
  void Seal(std::vector<OnlineSubmap>* revisions);
  int max_keyframes_;
  OnlineSubmapOptions options_;
  std::optional<OnlineSubmap> active_;
  std::vector<std::uint64_t> previous_members_;
  RigidTransform anchor_pose_;
  double anchor_time_ = 0;
  double last_time_ = 0;
  std::uint64_t next_keyframe_id_ = 0;
  std::uint64_t next_submap_id_ = 0;
  bool finished_ = false;
};
}  // namespace livo_atlas
#endif  // LIVO_ATLAS_MAPPING_ONLINE_SUBMAP_H_
