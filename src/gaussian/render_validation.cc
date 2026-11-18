#include "gaussian/render_validation.h"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/LU>

#include "mapping/pose_math.h"

namespace livo_atlas {
namespace {

void Require(bool condition, const char* detail) {
  if (!condition) throw std::invalid_argument(detail);
}

void Range(const std::optional<double>& value, double high) {
  Require(!value || (std::isfinite(*value) && *value >= 0 && *value <= high),
          "Render metric exceeds its range");
}

}  // namespace

bool SameRenderPose(const RigidTransform& a, const RigidTransform& b) {
  return (a.rotation.array() == b.rotation.array()).all() &&
         (a.translation.array() == b.translation.array()).all();
}

bool SameRenderCamera(const GaussianCamera& a, const GaussianCamera& b) {
  return a.keyframe_id == b.keyframe_id &&
         a.calibration_id == b.calibration_id && a.width == b.width &&
         a.height == b.height && (a.K.array() == b.K.array()).all() &&
         SameRenderPose(a.camera_from_body, b.camera_from_body) &&
         a.capture_time == b.capture_time;
}

void ValidateRenderPose(const RigidTransform& pose) {
  Require(pose.rotation.allFinite() && pose.translation.allFinite() &&
              (pose.rotation.transpose() * pose.rotation -
               Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() <= 1e-6 &&
              std::abs(pose.rotation.determinant() - 1) <= 1e-6,
          "Invalid render rigid pose");
}

void ValidateRenderCamera(const GaussianCamera& camera) {
  Require(camera.keyframe_id < 512 && camera.calibration_id == 0 &&
              camera.width > 0 && camera.height > 0 &&
              static_cast<std::int64_t>(camera.width) * camera.height <=
                  1000000 && std::isfinite(camera.capture_time) &&
              camera.K.allFinite() && camera.K(0, 0) > 0 &&
              camera.K(1, 1) > 0 && camera.K(0, 1) == 0 &&
              camera.K(1, 0) == 0 && camera.K(2, 0) == 0 &&
              camera.K(2, 1) == 0 && camera.K(2, 2) == 1,
          "Invalid retained render camera");
  ValidateRenderPose(camera.camera_from_body);
}

void ValidateRenderSelection(const GaussianRenderRequest& request) {
  const bool spatial = request.policy == GaussianRenderPolicy::kSpatial;
  Require((spatial || request.policy == GaussianRenderPolicy::kTemporal) &&
              !request.blocks.empty() && request.blocks.size() <= 2,
          "Invalid render policy or selection size");
  const bool dual = request.blocks.size() == 2;
  Require(request.neighbor_reason ==
              (dual ? "" : spatial ? "no_spatial_candidate"
                  : request.query_submap_id ? "neighbor_unavailable"
                                             : "no_previous_group"),
          "Missing or contradictory render coverage reason");
  if (!spatial) {
    Require(!request.candidate_count && !request.candidate_pixels &&
                !request.projection_uncertain && request.selection_seconds == 0,
            "Temporal request has a spatial selection summary");
    return;
  }
  const auto& owner = request.blocks.back();
  Require(owner.render_metadata.has_value(), "Missing spatial owner metadata");
  const auto& camera = owner.render_metadata->camera;
  const auto pixels =
      static_cast<std::uint64_t>(camera.width) * camera.height;
  Require(request.candidate_count <= 31 &&
              (request.candidate_count > 0) == dual &&
              (request.candidate_pixels > 0) == dual &&
              request.candidate_pixels <= pixels &&
              (!request.projection_uncertain ||
               request.candidate_pixels == pixels) &&
              std::isfinite(request.selection_seconds) &&
              request.selection_seconds >= 0,
          "Invalid spatial selection summary");
}

void ValidateRenderMetrics(const GaussianRenderMetrics& m, int pixels) {
  Require(pixels > 0 && m.image_pixels >= 0 && m.image_pixels <= pixels &&
              m.covered_pixels >= 0 && m.covered_pixels <= m.image_pixels &&
              m.depth_pixels_available >= 0 &&
              m.depth_pixels_available <= m.image_pixels &&
              m.depth_pixels_used >= 0 &&
              m.depth_pixels_used <= m.depth_pixels_available &&
              m.depth_pixels_used <= m.covered_pixels &&
              m.rgb_l1.has_value() == (m.image_pixels > 0) &&
              m.rgb_mse.has_value() == (m.image_pixels > 0) &&
              m.alpha_coverage.has_value() == (m.image_pixels > 0) &&
              m.depth_l1_m.has_value() == (m.depth_pixels_used > 0) &&
              m.depth_rmse_m.has_value() == (m.depth_pixels_used > 0) &&
              m.rgb_perfect == (m.rgb_mse && *m.rgb_mse == 0) &&
              m.rgb_psnr_db.has_value() == (m.rgb_mse && *m.rgb_mse > 0),
          "Inconsistent render metric validity");
  Range(m.rgb_l1, 1);
  Range(m.rgb_mse, 1);
  Range(m.alpha_coverage, 1);
  Range(m.depth_l1_m, std::numeric_limits<double>::max());
  Range(m.depth_rmse_m, std::numeric_limits<double>::max());
  if (m.rgb_psnr_db)
    Require(std::isfinite(*m.rgb_psnr_db) &&
                std::abs(*m.rgb_psnr_db + 10 * std::log10(*m.rgb_mse)) < 1e-6,
            "Incorrect render PSNR");
  const double coverage = m.image_pixels
      ? static_cast<double>(m.covered_pixels) / m.image_pixels : 0;
  Require(!m.alpha_coverage || std::abs(*m.alpha_coverage - coverage) < 1e-9,
          "Incorrect render alpha coverage");
}

void ValidateRenderImage(const GaussianRender& r) {
  ValidateGaussianRenderRequest(r.request);
  const auto pixels = static_cast<std::int64_t>(r.width) * r.height;
  Require(r.width > 0 && r.height > 0 && pixels <= 1000000 &&
              r.rgb.size() == 3 * static_cast<std::uint64_t>(pixels) &&
              r.changed_pixels >= 0 && r.changed_pixels <= pixels,
          "Invalid render pixels");
  Require(r.calibration_id == 0 && r.K.allFinite() && r.K(0, 0) > 0 &&
              r.K(1, 1) > 0 && r.K(0, 1) == 0 && r.K(1, 0) == 0 &&
              r.K(2, 0) == 0 && r.K(2, 1) == 0 && r.K(2, 2) == 1,
          "Invalid render camera");
  ValidateRenderPose(r.camera_from_body);
  ValidateRenderPose(r.camera_from_parent);
  if (r.request.policy == GaussianRenderPolicy::kSpatial) {
    const auto& camera = r.request.blocks.back().render_metadata->camera;
    Require(r.width == camera.width && r.height == camera.height &&
                r.calibration_id == camera.calibration_id &&
                (r.K.array() == camera.K.array()).all() &&
                SameRenderPose(r.camera_from_body, camera.camera_from_body) &&
                r.capture_time == camera.capture_time,
            "Render replaced the retained spatial query camera");
  }
  Require(pose_math::Equal(r.camera_from_parent,
              pose_math::Compose(r.camera_from_body,
                  pose_math::Inverse(r.request.parent_from_body))),
          "Incorrect render camera composition");
  for (const double value : {r.capture_time, r.state_timestamp,
           r.cloud_timestamp, r.inverse_exposure_time,
           r.accepted_at_unix_seconds, r.accepted_at_steady_seconds,
           r.latency_seconds, r.worker_seconds})
    Require(std::isfinite(value), "Nonfinite render timing");
  Require(r.accepted_at_unix_seconds >= 0 &&
              r.accepted_at_steady_seconds >= 0 && r.latency_seconds >= 0 &&
              r.worker_seconds >= 0,
          "Negative render acceptance timing");
  ValidateRenderMetrics(r.single, static_cast<int>(pixels));
  ValidateRenderMetrics(r.joint, static_cast<int>(pixels));
}

}  // namespace livo_atlas
