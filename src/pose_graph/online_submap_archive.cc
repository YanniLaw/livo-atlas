#include "pose_graph/online_submap_archive.h"

#include <utility>

#include "mapping/pose_math.h"
#include "mapping/submap_pose_comparison.h"

namespace livo_atlas::pose_graph {

OnlineSubmapArchive::OnlineSubmapArchive(
    const std::filesystem::path& directory, const KeyframeStore& keyframes,
    int max_keyframes, OnlineSubmapOptions options)
    : builder_(max_keyframes, options),
      store_(directory, keyframes, max_keyframes, options),
      frozen_(std::make_shared<const PoseSnapshot>()) {}

PoseFrontier OnlineSubmapArchive::GetPoseFrontier() const {
  auto frontier = frozen_->frontier();
  frontier.frame_count = raw_poses_.size();
  return frontier;
}

PoseSnapshot OnlineSubmapArchive::MaterializePoses() const {
  return MaterializePoseSnapshot(*frozen_, raw_poses_, raw_poses_.size());
}

OnlineSubmapDelta OnlineSubmapArchive::Append(
    std::shared_ptr<const Keyframe> keyframe) {
  auto revisions = builder_.AddKeyframe(keyframe->id, keyframe->timestamp,
                                       keyframe->odom_from_body);
  raw_poses_.push_back(keyframe->odom_from_body);
  for (const auto& revision : revisions) store_.AppendRevision(revision);
  const auto appended =
      pose_math::MaterializeAppendedPose(*frozen_, raw_poses_);
  if (correction_) correction_->frame_count = raw_poses_.size();
  return {std::move(keyframe), std::move(revisions), GetPoseFrontier(), {}, {},
          appended};
}

std::optional<OnlineSubmapDelta> OnlineSubmapArchive::SealInput() {
  auto revisions = builder_.Finish();
  if (revisions.empty()) return std::nullopt;
  for (const auto& revision : revisions) store_.AppendRevision(revision);
  return OnlineSubmapDelta{{}, std::move(revisions), GetPoseFrontier(), {}, {}};
}

PoseSnapshot OnlineSubmapArchive::PreparePoseUpdate(
    const PoseUpdate& update) const {
  return MakePoseSnapshot(update, *frozen_, raw_poses_);
}

OnlineSubmapDelta OnlineSubmapArchive::WritePoses(
    std::shared_ptr<const PoseSnapshot> candidate) {
  const auto previous = MaterializePoses();
  const SubmapPoseComparison comparison(previous, *candidate);
  std::vector<SubmapPoseChange> changes;
  changes.reserve(store_.size());
  for (std::size_t i = 0; i < store_.size(); ++i) {
    changes.push_back(comparison.Compare(store_.Read(i)));
  }
  std::optional<MapCorrection> correction;
  if (candidate->tail_policy == PoseTailPolicy::kProjectedLastOptimizedAnchor) {
    const auto anchor = candidate->optimized_frame_count - 1;
    correction = MapCorrection{
        candidate->version, candidate->frame_count,
        candidate->optimized_frame_count,
        pose_math::ProjectedCorrection(
            candidate->poses[anchor].parent_from_body(), raw_poses_[anchor])};
  }
  store_.WritePoseSnapshot(*candidate);
  frozen_ = std::move(candidate);
  correction_ = correction;
  return {{}, {}, frozen_->frontier(), frozen_, std::move(changes)};
}

}  // namespace livo_atlas::pose_graph
