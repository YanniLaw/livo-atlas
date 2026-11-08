#include "online_submap_manager.h"

#include <stdexcept>
#include <utility>
namespace livo_atlas {
OnlineSubmapManager::OnlineSubmapManager(const std::filesystem::path& directory,
                                         KeyframeStore& keyframes,
                                         int max_keyframes,
                                         OnlineSubmapOptions options)
    : builder_(max_keyframes, options),
      store_(directory, keyframes, max_keyframes, options) {}
void OnlineSubmapManager::RethrowFailure() const {
  if (failure_) std::rethrow_exception(failure_);
}
void OnlineSubmapManager::CheckRunning() const {
  RethrowFailure();
  if (finished_) throw std::logic_error("Online mapping has finished");
}
void OnlineSubmapManager::SetCallback(
    LivoSystem::OnlineSubmapCallback callback) {
  if (callback) {
    CheckRunning();
    if (!raw_poses_.empty())
      throw std::logic_error(
          "Register online callback before processing frames");
  }
  callback_ = std::move(callback);
}
std::shared_ptr<const PoseSnapshot> OnlineSubmapManager::GetPoseSnapshot()
    const {
  return std::make_shared<const PoseSnapshot>(
      MaterializePoseSnapshot(frozen_, raw_poses_, raw_poses_.size()));
}
void OnlineSubmapManager::Notify(OnlineSubmapUpdate update) {
  if (!callback_) return;
  update.poses = GetPoseSnapshot();
  callback_(update);
}
void OnlineSubmapManager::AppendCommittedKeyframe(
    std::shared_ptr<const Keyframe> keyframe) {
  CheckRunning();
  const auto previous_count = raw_poses_.size();
  bool committed = false;
  try {
    if (!keyframe) throw std::invalid_argument("Missing committed keyframe");
    auto revisions = builder_.AddKeyframe(keyframe->id, keyframe->timestamp,
                                          keyframe->odom_from_body);
    // Allocate metadata capacity before committing any membership revision.
    raw_poses_.push_back(keyframe->odom_from_body);
    for (const auto& revision : revisions) store_.AppendRevision(revision);
    committed = true;
    Notify({std::move(keyframe), std::move(revisions), {}, {}});
  } catch (...) {
    if (!committed) raw_poses_.resize(previous_count);
    failure_ = std::current_exception();
    throw;
  }
}
void OnlineSubmapManager::ApplyPoseUpdate(const PoseUpdate& update) {
  CheckRunning();
  // Invalid/stale backend inputs do not poison the running writer.
  auto candidate = MakePoseSnapshot(update, frozen_, raw_poses_);
  const auto previous = GetPoseSnapshot();
  std::vector<SubmapPoseChange> changes;
  try {
    for (std::size_t i = 0; i < store_.size(); ++i)
      changes.push_back(
          CompareSubmapPoses(store_.Read(i), *previous, candidate));
    store_.WritePoseSnapshot(candidate);
    frozen_ = std::move(candidate);
    Notify({{}, {}, {}, std::move(changes)});
  } catch (...) {
    failure_ = std::current_exception();
    throw;
  }
}
void OnlineSubmapManager::Finish() {
  if (failure_) std::rethrow_exception(failure_);
  if (finished_) return;
  try {
    auto revisions = builder_.Finish();
    for (const auto& revision : revisions) store_.AppendRevision(revision);
    finished_ = true;
    if (!revisions.empty()) Notify({{}, std::move(revisions), {}, {}});
  } catch (...) {
    failure_ = std::current_exception();
    throw;
  }
}
}  // namespace livo_atlas
