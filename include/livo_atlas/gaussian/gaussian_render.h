#ifndef LIVO_ATLAS_GAUSSIAN_GAUSSIAN_RENDER_H_
#define LIVO_ATLAS_GAUSSIAN_GAUSSIAN_RENDER_H_

#include <Eigen/Core>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "livo_atlas/gaussian/gaussian_map.h"
#include "livo_atlas/gaussian/gaussian_render_policy.h"

namespace livo_atlas {

// Metadata owns its poses and coverage; it is not an on-disk asset lease.
struct GaussianRenderBlock {
  GaussianTarget target;
  GaussianContentId content;
  std::optional<GaussianAssetReference> asset;
  std::vector<std::uint64_t> frame_ids;
  int gaussian_count = 0;
  bool resident = false;
  std::optional<GaussianRenderMetadata> render_metadata;
};

struct GaussianRenderRequest {
  std::string session_id;
  std::uint64_t job_id = 0;      // Assigned only when admitted to the mailbox.
  std::uint64_t generation = 0;  // Fence for the resident model, not a reset.
  std::uint64_t map_sequence = 0;
  std::uint64_t pose_version = 0;
  std::uint64_t frame_count = 0;
  std::uint64_t optimized_frame_count = 0;
  PoseFrame parent_frame = PoseFrame::kOdom;
  std::uint64_t query_keyframe_id = 0;
  std::uint64_t query_submap_id = 0;
  RigidTransform parent_from_body;
  std::vector<GaussianRenderBlock> blocks;
  std::string neighbor_reason;
  GaussianRenderPolicy policy = GaussianRenderPolicy::kTemporal;
  std::uint64_t candidate_count = 0;
  std::uint64_t candidate_pixels = 0;
  bool projection_uncertain = false;
  double selection_seconds = 0;
};

struct GaussianRenderMetrics {
  int image_pixels = 0;
  int covered_pixels = 0;
  int depth_pixels_available = 0;
  int depth_pixels_used = 0;
  bool rgb_perfect = false;
  std::optional<double> rgb_l1;
  std::optional<double> rgb_mse;
  std::optional<double> rgb_psnr_db;
  std::optional<double> alpha_coverage;
  std::optional<double> depth_l1_m;
  std::optional<double> depth_rmse_m;
};

// One accepted training-view image. Files retain this exact as-of identity.
struct GaussianRender {
  GaussianRenderRequest request;
  int width = 0;
  int height = 0;
  std::vector<std::uint8_t> rgb;
  std::uint64_t calibration_id = 0;
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  RigidTransform camera_from_body;
  RigidTransform camera_from_parent;
  double capture_time = 0;
  double state_timestamp = 0;
  double cloud_timestamp = 0;
  double inverse_exposure_time = 1;
  double accepted_at_unix_seconds = 0;
  double accepted_at_steady_seconds = 0;
  double latency_seconds = 0;
  double worker_seconds = 0;
  GaussianRenderMetrics single;
  GaussianRenderMetrics joint;
  int changed_pixels = 0;
  std::uint64_t gpu_peak_allocated_bytes = 0;
  std::uint64_t gpu_peak_reserved_bytes = 0;
  std::uint64_t gpu_allocated_bytes = 0;
  std::uint64_t gpu_reserved_bytes = 0;
  std::uint64_t worker_peak_rss_bytes = 0;
};

struct GaussianRenderStats {
  std::uint64_t submitted = 0, accepted = 0, stale = 0, coalesced = 0;
  std::uint64_t skipped = 0, failed = 0, saved = 0, dual_accepted = 0;
  std::uint64_t pending = 0, in_flight = 0, peak_pending = 0;
  std::uint64_t peak_in_flight = 0;
  double last_latency_seconds = 0, max_latency_seconds = 0;
  double total_latency_seconds = 0;
  bool disabled = false;
  std::string error;
  std::uint64_t pending_storage = 0;
  // Codes describe owner state, rather than emitting a line on every Poll.
  std::string wait_reason;
  std::uint64_t wait_map_sequence = 0;
  std::uint64_t wait_changes = 0;
};

std::optional<GaussianRenderRequest> SelectGaussianRender(
    const GaussianMapSnapshot& map, const PoseSnapshot& poses,
    std::uint64_t generation, std::string* reason);

std::optional<GaussianRenderRequest> SelectGaussianRender(
    const GaussianMapSnapshot& map, const PoseSnapshot& poses,
    std::uint64_t generation, GaussianRenderPolicy policy, std::string* reason);

bool IsCurrentGaussianRender(const GaussianRenderRequest& request,
                             const GaussianMapSnapshot& map,
                             const PoseSnapshot& poses);

void ValidateGaussianRenderRequest(const GaussianRenderRequest& request);

// Shared source/pose checks. Preview adds its retained-camera requirement.
void ValidateGaussianSourceRequest(const GaussianRenderRequest& request);

std::optional<GaussianRenderRequest> SelectGaussianEvaluation(
    const GaussianMapSnapshot& map, const PoseSnapshot& poses,
    std::uint64_t generation, const GaussianCamera& camera,
    GaussianRenderPolicy policy, std::string* reason);

// Bounded, single-line encoding of the complete accepted request header.
std::string GaussianRenderSourceIdentity(const GaussianRenderRequest& request);

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_GAUSSIAN_GAUSSIAN_RENDER_H_
