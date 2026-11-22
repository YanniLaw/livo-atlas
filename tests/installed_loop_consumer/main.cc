#include "livo_atlas/backend/loop_closure.h"

#include <memory>

int main() {
  livo_atlas::GeometrySubmap target, source;
  target.membership.primary = {0, 0, {0}};
  source.membership.primary = {5, 100, {100}};
  for (auto* group : {&target, &source}) {
    group->membership.state = livo_atlas::OnlineSubmapState::kSealed;
    group->membership.membership_revision = 2;
    group->frame_count = 200;
    group->representation_revision = 1;
    auto data = std::make_shared<livo_atlas::GeometryData>();
    data->sources.push_back({group->membership.primary.anchor_keyframe_id, {}});
    group->data = data;
  }
  const auto result = livo_atlas::VerifyLoop(target, source);
  if (result.status != livo_atlas::LoopStatus::kInsufficientPoints ||
      result.target_from_source)
    return 1;
  return 0;
}
