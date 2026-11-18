#ifndef LIVO_ATLAS_GAUSSIAN_GAUSSIAN_MAP_H_
#define LIVO_ATLAS_GAUSSIAN_GAUSSIAN_MAP_H_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "livo_atlas/mapping/online_submap.h"
#include "livo_atlas/mapping/pose_snapshot.h"

namespace livo_atlas {

enum class GaussianOperation { kView, kReuse, kRebuild };
enum class GaussianAvailability { kMissing, kCurrent, kNeedsRebuild, kFailed };

struct GaussianContentId {
  std::string session_id;
  std::uint64_t submap_id = 0;
  std::uint64_t source_generation = 0;
  std::uint64_t content_job_id = 0;
};

bool operator==(const GaussianContentId& left, const GaussianContentId& right);
bool operator!=(const GaussianContentId& left, const GaussianContentId& right);

// Bounds are local optimized Gaussian means, grouped by their retained origin.
struct GaussianExtent {
  std::uint64_t origin_keyframe_id = 0;
  Eigen::Vector3d min = Eigen::Vector3d::Zero();
  Eigen::Vector3d max = Eigen::Vector3d::Zero();
  double max_scale = 0;
};

struct GaussianCamera {
  std::uint64_t keyframe_id = 0;
  std::uint64_t calibration_id = 0;
  int width = 0;
  int height = 0;
  // Original integer pixel centers; gsplat conversion adds 0.5 exactly once.
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  RigidTransform camera_from_body;
  double capture_time = 0;
};

struct GaussianRenderMetadata {
  GaussianContentId content;
  std::vector<GaussianExtent> extents;
  GaussianCamera camera;
};

bool operator==(const GaussianRenderMetadata& a,
                const GaussianRenderMetadata& b);
bool operator!=(const GaussianRenderMetadata& a,
                const GaussianRenderMetadata& b);

void ValidateGaussianRenderMetadata(
    const GaussianRenderMetadata& metadata, const GaussianContentId& content,
    const std::vector<std::uint64_t>& frame_ids);

// Primary body poses are local to the first primary body, never the LiDAR.
// Generation authorizes an operation; it is not inferred from stored content.
struct GaussianTarget {
  std::string session_id;
  std::uint64_t generation = 0;
  OnlineSubmap group;
  std::uint64_t pose_version = 0;
  std::uint64_t frame_count = 0;
  std::uint64_t optimized_frame_count = 0;
  PoseFrame parent_frame = PoseFrame::kOdom;
  RigidTransform parent_from_submap;
  std::vector<RigidTransform> submap_from_body;
};

struct GaussianAssetReference {
  std::uint64_t asset_id = 0;
  GaussianContentId content;
  std::string manifest_sha256;
  std::uint64_t manifest_bytes = 0;
  std::uint64_t gaussian_bytes = 0;
  std::uint64_t observation_bytes = 0;
};

struct GaussianSubmapEntry {
  OnlineSubmap group;
  GaussianAvailability availability = GaussianAvailability::kMissing;
  std::string reason;
  bool resident = false;
  bool dirty = false;
  GaussianTarget binding;
  std::uint64_t binding_job_id = 0;
  std::optional<GaussianContentId> content;
  // This baseline changes only when parameters are trained, never on reuse.
  GaussianTarget content_target;
  std::vector<std::uint64_t> frame_ids;
  int gaussian_count = 0;
  std::optional<GaussianAssetReference> asset;
  // Durable coverage can lag the resident confirmation, even at one version.
  GaussianTarget asset_source;
  std::vector<std::uint64_t> asset_frame_ids;
  int asset_gaussian_count = 0;
  bool asset_current = false;
  std::uint64_t last_repair_attempt = 0;
  // Present only after an explicitly negotiated spatial candidate is accepted.
  // Asset/map schemas remain independent of this in-memory description.
  std::optional<GaussianRenderMetadata> render_metadata;
};

struct GaussianMapSnapshot {
  std::string session_id;
  std::uint64_t map_sequence = 0;
  std::uint64_t pose_version = 0;
  std::uint64_t frame_count = 0;
  std::uint64_t optimized_frame_count = 0;
  PoseFrame parent_frame = PoseFrame::kOdom;
  std::vector<GaussianSubmapEntry> entries;
};

enum class GaussianFinalizationStatus { kSucceeded, kTimedOut, kFailed };

struct GaussianFinalizationCounters {
  std::uint64_t accepted = 0, effective_training = 0, rebuilds = 0,
                checkpoints = 0;
};

// Only metadata is returned. Actual view/parameter coverage remains in
// map.json.
struct GaussianFinalizationResult {
  GaussianFinalizationStatus status = GaussianFinalizationStatus::kFailed;
  std::string reason;
  bool target_valid = false, eligible_set_frozen = false;
  std::string session_id;
  std::uint64_t pose_version = 0, frame_count = 0, optimized_frame_count = 0;
  PoseFrame parent_frame = PoseFrame::kOdom;
  std::vector<std::uint64_t> eligible_submap_ids;
  std::uint64_t current = 0, needs_rebuild = 0, missing = 0, dirty = 0;
  bool durable = false, worker_stopped = false;
  std::uint64_t map_sequence = 0, repair_attempts = 0;
  std::vector<std::uint64_t> asset_ids;
  GaussianFinalizationCounters before, after;
  double elapsed_seconds = 0, repair_seconds = 0, shutdown_seconds = 0;
  // These describe publishing the report, so the file cannot assert them.
  bool report_persisted = false;
  std::string report_error;
  double total_seconds = 0;
};

void ValidateGaussianTarget(const GaussianTarget& target);
void ValidateGaussianAssetReference(const GaussianAssetReference& asset);

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_GAUSSIAN_GAUSSIAN_MAP_H_
