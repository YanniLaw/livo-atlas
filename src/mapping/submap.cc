#include "livo_atlas/mapping/submap.h"

#include <cstddef>
#include <stdexcept>
#include <utility>

namespace livo_atlas {
void ValidateSubmapOptions(const SubmapOptions& options) {
  if (options.max_keyframes <= 0)
    throw std::invalid_argument("Submap max_keyframes must be positive");
}

void ValidateSubmap(const Submap& submap) {
  const auto& ids = submap.keyframe_ids;
  if (ids.empty() || submap.anchor_keyframe_id != ids.front())
    throw std::invalid_argument("Submap requires its first member as anchor");
  for (std::size_t i = 1; i < ids.size(); ++i) {
    if (ids[i] <= ids[i - 1] || ids[i] - ids[i - 1] != 1)
      throw std::invalid_argument("Submap members must be consecutive IDs");
  }
}

SubmapBuilder::SubmapBuilder(int max_keyframes)
    : max_keyframes_(max_keyframes) {
  ValidateSubmapOptions({false, max_keyframes});
}

std::optional<Submap> SubmapBuilder::AddKeyframe(std::uint64_t id) {
  if (finished_) throw std::logic_error("Submap builder has finished");
  if (id != next_keyframe_id_)
    throw std::invalid_argument("Expected the next committed keyframe ID");
  members_.push_back(id);
  ++next_keyframe_id_;
  if (members_.size() == static_cast<std::size_t>(max_keyframes_))
    return Emit();
  return {};
}

std::optional<Submap> SubmapBuilder::Finish() {
  finished_ = true;
  if (members_.empty()) return {};
  return Emit();
}

Submap SubmapBuilder::Emit() {
  Submap result{next_submap_id_, members_.front(), std::move(members_)};
  members_.clear();
  ++next_submap_id_;
  return result;
}
}  // namespace livo_atlas
