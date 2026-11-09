#include "livo_atlas/mapping/online_submap.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace livo_atlas {
void ValidateOnlineSubmapOptions(int max_keyframes,
                                 const OnlineSubmapOptions& options) {
  ValidateSubmapOptions({false, max_keyframes});
  for (double threshold :
       {options.translation_threshold, options.rotation_threshold_deg,
        options.time_threshold}) {
    if (!std::isfinite(threshold) || threshold <= 0)
      throw std::invalid_argument(
          "Online submap thresholds must be finite and positive");
  }
  if (options.support_keyframes < 0 ||
      options.support_keyframes > max_keyframes)
    throw std::invalid_argument("Online submap support count is out of range");
}

OnlineSubmapBuilder::OnlineSubmapBuilder(int max_keyframes,
                                         OnlineSubmapOptions options)
    : max_keyframes_(max_keyframes), options_(options) {
  ValidateOnlineSubmapOptions(max_keyframes, options);
}

std::vector<OnlineSubmap> OnlineSubmapBuilder::AddKeyframe(
    std::uint64_t id, double timestamp, const RigidTransform& odom_from_body) {
  if (finished_) throw std::logic_error("Online submap builder has finished");
  if (id != next_keyframe_id_ || !std::isfinite(timestamp) ||
      (id && timestamp < last_time_))
    throw std::invalid_argument("Invalid online keyframe ID or timestamp");
  const auto error = ValidateExtrinsic(odom_from_body);
  if (!error.empty()) throw std::invalid_argument(error);
  const auto maximum = std::numeric_limits<std::uint64_t>::max();
  if (id == maximum || next_submap_id_ == maximum ||
      (active_ && active_->membership_revision > maximum - 2))
    throw std::overflow_error("Online submap counter exhausted");
  std::vector<OnlineSubmap> revisions;
  if (active_) {
    const double distance =
        (odom_from_body.translation - anchor_pose_.translation).norm();
    const double angle = Eigen::AngleAxisd(anchor_pose_.rotation.transpose() *
                                           odom_from_body.rotation)
                             .angle();
    if (distance >= options_.translation_threshold ||
        angle >= options_.rotation_threshold_deg * (std::acos(-1.0) / 180.0) ||
        timestamp - anchor_time_ >= options_.time_threshold)
      Seal(&revisions);
  }
  if (!active_) {
    OnlineSubmap submap;
    submap.primary.id = next_submap_id_++;
    submap.primary.anchor_keyframe_id = id;
    const auto count =
        std::min(previous_members_.size(),
                 static_cast<std::size_t>(options_.support_keyframes));
    submap.support_keyframe_ids.assign(previous_members_.end() - count,
                                       previous_members_.end());
    active_ = std::move(submap);
    anchor_pose_ = odom_from_body;
    anchor_time_ = timestamp;
  } else {
    ++active_->membership_revision;
  }
  active_->primary.keyframe_ids.push_back(id);
  revisions.push_back(*active_);
  if (active_->primary.keyframe_ids.size() ==
      static_cast<std::size_t>(max_keyframes_))
    Seal(&revisions);
  ++next_keyframe_id_;
  last_time_ = timestamp;
  return revisions;
}

void OnlineSubmapBuilder::Seal(std::vector<OnlineSubmap>* revisions) {
  ++active_->membership_revision;
  active_->state = OnlineSubmapState::kSealed;
  revisions->push_back(*active_);
  previous_members_ = active_->primary.keyframe_ids;
  active_.reset();
}

std::vector<OnlineSubmap> OnlineSubmapBuilder::Finish() {
  std::vector<OnlineSubmap> revisions;
  if (active_) Seal(&revisions);
  finished_ = true;
  return revisions;
}
}  // namespace livo_atlas
