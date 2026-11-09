#ifndef LIVO_ATLAS_MAPPING_SUBMAP_H_
#define LIVO_ATLAS_MAPPING_SUBMAP_H_

#include <cstdint>
#include <optional>
#include <vector>

namespace livo_atlas {
struct SubmapOptions {
  bool enabled = false;
  int max_keyframes = 20;
};
void ValidateSubmapOptions(const SubmapOptions& options);

// Stable membership; the coordinate frame is the first member's IMU/body.
struct Submap {
  std::uint64_t id = 0;
  std::uint64_t anchor_keyframe_id = 0;
  std::vector<std::uint64_t> keyframe_ids;
};
void ValidateSubmap(const Submap& submap);

// Serial grouping of committed keyframe IDs. It never retains sensor data.
class SubmapBuilder {
 public:
  explicit SubmapBuilder(int max_keyframes);
  std::optional<Submap> AddKeyframe(std::uint64_t id);
  std::optional<Submap> Finish();

 private:
  Submap Emit();
  int max_keyframes_;
  std::uint64_t next_keyframe_id_ = 0;
  std::uint64_t next_submap_id_ = 0;
  std::vector<std::uint64_t> members_;
  bool finished_ = false;
};
}  // namespace livo_atlas

#endif  // LIVO_ATLAS_MAPPING_SUBMAP_H_
