#include "online_submap_manager.h"

#include <chrono>
#include <exception>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "livo_atlas/common/logging.h"
#include "mapping/pose_math.h"

namespace livo_atlas {
OnlineSubmapManager::OnlineSubmapManager(const std::filesystem::path& directory,
                                         KeyframeStore& keyframes,
                                         int max_keyframes,
                                         OnlineSubmapOptions options,
                                         GeometryOptions geometry)
    : builder_(max_keyframes, options),
      store_(directory, keyframes, max_keyframes, options),
      geometry_(
          geometry.enabled
              ? std::make_unique<GeometryMapper>(
                    geometry,
                    [&keyframes, limit = geometry.max_points_per_keyframe](
                        std::uint64_t id) {
                      return keyframes.ReadBodyCloud(id, limit);
                    })
              : nullptr) {}

void OnlineSubmapManager::RethrowFailure() const {
  if (failure_) std::rethrow_exception(failure_);
}

void OnlineSubmapManager::CheckRunning() const {
  RethrowFailure();
  if (finished_) throw std::logic_error("Online mapping has finished");
}

void OnlineSubmapManager::SetLoopBackend(LoopBackend& backend) {
  loop_backend_ = &backend;
}

void OnlineSubmapManager::SetCallback(
    LivoSystem::OnlineSubmapCallback callback) {
  if (callback) {
    CheckRunning();
    if (input_sealed_ || !raw_poses_.empty())
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

MapCorrection OnlineSubmapManager::GetMapCorrection() const {
  MapCorrection correction{
      frozen_.version, raw_poses_.size(), frozen_.optimized_frame_count, {}};
  if (frozen_.version == 0) return correction;
  if (frozen_.tail_policy != PoseTailPolicy::kProjectedLastOptimizedAnchor ||
      frozen_.optimized_frame_count == 0 ||
      frozen_.optimized_frame_count > frozen_.poses.size() ||
      frozen_.optimized_frame_count > raw_poses_.size())
    throw std::invalid_argument(
        "Map correction requires a projected optimized anchor");
  // Commit already validates the full prefix. Reading its last optimized
  // anchor stays constant-size even as additional raw frames accumulate.
  const auto anchor = frozen_.optimized_frame_count - 1;
  correction.map_from_odom = pose_math::ProjectedCorrection(
      frozen_.poses[anchor].map_from_body, raw_poses_[anchor]);
  if (!correction.map_from_odom.rotation.allFinite() ||
      !correction.map_from_odom.translation.allFinite())
    throw std::invalid_argument("Derived map correction must be finite");
  return correction;
}

void OnlineSubmapManager::Notify(OnlineSubmapUpdate update) {
  if (!loop_backend_ && !callback_ && !geometry_) return;
  const auto start = std::chrono::steady_clock::now();
  update.poses = GetPoseSnapshot();
  if (geometry_) {
    geometry_->Update(update.revisions, update.poses, update.keyframe.get());
    const auto total =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
            .count();
    const auto stats = geometry_->GetStats();
    std::ostringstream message;
    message.imbue(std::locale::classic());
    message << std::setprecision(17) << "geometry_total_seconds=" << total
            << " geometry_update_seconds=" << stats.last_update_seconds
            << " geometry_cloud_read_seconds=" << stats.last_cloud_read_seconds
            << " geometry_full_build_seconds=" << stats.last_full_build_seconds
            << " geometry_snapshot_seconds=" << stats.last_snapshot_seconds;
    LOG(INFO) << message.str();
  }
  if (loop_backend_) loop_backend_->Observe(update);
  if (callback_) callback_(update);
}

GeometrySubmap OnlineSubmapManager::GetGeometrySubmap(std::uint64_t id) {
  if (!geometry_) throw std::logic_error("Geometry submaps are not enabled");
  RethrowFailure();
  if (id >= store_.size())
    throw std::out_of_range("Unknown geometry submap ID");
  try {
    return geometry_->GetSubmap(id);
  } catch (...) {
    failure_ = std::current_exception();
    throw;
  }
}

GeometryCacheStats OnlineSubmapManager::GetGeometryStats() const {
  if (!geometry_) throw std::logic_error("Geometry submaps are not enabled");
  return geometry_->GetStats();
}

void OnlineSubmapManager::AppendCommittedKeyframe(
    std::shared_ptr<const Keyframe> keyframe) {
  CheckRunning();
  if (input_sealed_) throw std::logic_error("Online input has been sealed");
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

void OnlineSubmapManager::SealInput() {
  RethrowFailure();
  if (input_sealed_) return;
  try {
    auto revisions = builder_.Finish();
    for (const auto& revision : revisions) store_.AppendRevision(revision);
    input_sealed_ = true;
    if (!revisions.empty()) Notify({{}, std::move(revisions), {}, {}});
  } catch (...) {
    failure_ = std::current_exception();
    throw;
  }
}

void OnlineSubmapManager::Finish() {
  RethrowFailure();
  if (finished_) return;
  SealInput();
  finished_ = true;
}
}  // namespace livo_atlas
