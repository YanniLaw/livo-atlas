#include "livo_atlas/gaussian/gaussian_render_snapshot.h"

#include <algorithm>
#include <stdexcept>

#include "gaussian/render_validation.h"
#include "livo_atlas/gaussian/online_gaussian_mapper.h"

namespace livo_atlas {
namespace {

bool Identity(const std::string& value) {
  return value.size() == 32 &&
         std::all_of(value.begin(), value.end(), [](char c) {
           return (c >= 'a' && c <= 'f') || (c >= '0' && c <= '9');
         });
}

bool Reason(const GaussianRenderSnapshot& source) {
  switch (source.status) {
    case GaussianRenderStatus::kReady:
      return source.reason.empty();
    case GaussianRenderStatus::kWaiting:
      return source.reason == "awaiting_render";
    case GaussianRenderStatus::kInvalidated:
      return source.reason == "source_changed";
    case GaussianRenderStatus::kDisabled:
      return source.reason == "render_disabled";
    case GaussianRenderStatus::kError:
      return source.reason == "mapper_failed" ||
             source.reason == "invalid_snapshot" ||
             source.reason == "publication_failed";
    case GaussianRenderStatus::kStopping:
      return source.reason == "stopping";
    case GaussianRenderStatus::kStopped:
      return source.reason == "stopped" || source.reason == "mapper_failed" ||
             source.reason == "render_disabled";
  }
  return false;
}

}  // namespace

void ValidateGaussianRenderSnapshotHeader(const GaussianRenderSnapshot& s,
                                          bool require_output_identity) {
  if (!Identity(s.session_id) || s.producer_id.size() > 32 ||
      (require_output_identity && (!Identity(s.producer_id) || !s.revision)) ||
      !Reason(s) || s.frame_count > 512 ||
      s.optimized_frame_count > s.frame_count ||
      (s.pose_version == 0
           ? s.parent_frame != PoseFrame::kOdom || s.optimized_frame_count
           : s.parent_frame != PoseFrame::kMap || !s.optimized_frame_count) ||
      (s.policy != GaussianRenderPolicy::kTemporal &&
       s.policy != GaussianRenderPolicy::kSpatial))
    throw std::invalid_argument("Invalid Gaussian render snapshot header");
  if (s.status != GaussianRenderStatus::kReady) {
    if (s.render)
      throw std::invalid_argument("Non-ready Gaussian snapshot owns pixels");
    return;
  }
  if (!s.render)
    throw std::invalid_argument("Ready Gaussian snapshot has no pixels");
  const auto& r = *s.render;
  const auto& q = r.request;
  const auto pixels = static_cast<std::int64_t>(r.width) * r.height;
  if (r.width <= 0 || r.height <= 0 || pixels > 1000000 ||
      r.rgb.size() != 3 * static_cast<std::uint64_t>(pixels) ||
      q.session_id != s.session_id || q.generation != s.generation ||
      q.map_sequence != s.map_sequence || q.pose_version != s.pose_version ||
      q.frame_count != s.frame_count ||
      q.optimized_frame_count != s.optimized_frame_count ||
      q.parent_frame != s.parent_frame || q.policy != s.policy || !q.job_id ||
      q.blocks.empty() || q.blocks.size() > 2 || q.neighbor_reason.size() > 64)
    throw std::invalid_argument("Invalid Gaussian render snapshot payload");
}

void ValidateGaussianRenderSnapshot(const GaussianRenderSnapshot& source) {
  ValidateGaussianRenderSnapshotHeader(source, true);
  if (source.render) ValidateRenderImage(*source.render);
}

GaussianRenderSnapshot OnlineGaussianMapper::GetRenderSnapshot() const {
  GaussianRenderSnapshot result;
  result.session_id = session_id_;
  result.generation = generation_;
  result.map_sequence = map_index_.sequence();
  result.policy = options_.joint_render_policy;
  if (poses_) {
    result.pose_version = poses_->version;
    result.frame_count = poses_->frame_count;
    result.optimized_frame_count = poses_->optimized_frame_count;
    result.parent_frame = poses_->parent_frame;
  }
  // Derive lifecycle from current owner state, never from the last wait log.
  const bool disabled = !options_.enabled || !options_.joint_render_enabled ||
                        render_stats_.disabled;
  if (finished_) {
    result.status = GaussianRenderStatus::kStopped;
    result.reason = failed_    ? "mapper_failed"
                    : disabled ? "render_disabled"
                               : "stopped";
  } else if (failed_) {
    result.status = GaussianRenderStatus::kError;
    result.reason = "mapper_failed";
  } else if (finishing_ || finalization_phase_ != FinalizationPhase::kNone) {
    result.status = GaussianRenderStatus::kStopping;
    result.reason = "stopping";
  } else if (disabled) {
    result.status = GaussianRenderStatus::kDisabled;
    result.reason = "render_disabled";
  } else {
    if (latest_render_) {
      const auto& q = latest_render_->request;
      if (q.session_id == result.session_id &&
          q.generation == result.generation &&
          q.map_sequence == result.map_sequence &&
          q.pose_version == result.pose_version &&
          q.frame_count == result.frame_count &&
          q.optimized_frame_count == result.optimized_frame_count &&
          q.parent_frame == result.parent_frame && q.policy == result.policy) {
        result.status = GaussianRenderStatus::kReady;
        result.reason.clear();
        result.render = latest_render_;
        return result;
      }
    }
    if (render_stats_.accepted) {
      result.status = GaussianRenderStatus::kInvalidated;
      result.reason = "source_changed";
    }
  }
  return result;
}

}  // namespace livo_atlas
