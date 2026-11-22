#ifndef LIVO_ATLAS_SRC_BACKEND_LOOP_OPTION_FIELDS_H_
#define LIVO_ATLAS_SRC_BACKEND_LOOP_OPTION_FIELDS_H_

#include "livo_atlas/backend/loop_admission.h"

namespace livo_atlas::loop_option_fields {

template <typename Visit>
void SearchFields(LoopSearchOptions& o, Visit visit) {
  visit("min_submap_gap", o.min_submap_gap);
  visit("min_keyframe_gap", o.min_keyframe_gap);
  visit("min_time_gap_s", o.min_time_gap_s);
  visit("search_radius_m", o.search_radius_m);
  visit("max_candidates", o.max_candidates);
}

template <typename Visit>
void RegistrationFields(LoopRegistrationOptions& o, Visit visit) {
  visit("voxel_size_m", o.voxel_size_m);
  visit("min_points", o.min_points);
  visit("max_points", o.max_points);
  visit("max_voxels", o.max_voxels);
  visit("neighbors", o.neighbors);
  visit("max_iterations", o.max_iterations);
  visit("max_optimizer_iterations", o.max_optimizer_iterations);
  visit("correspondence_distance_m", o.correspondence_distance_m);
  visit("translation_epsilon", o.translation_epsilon);
  visit("rotation_epsilon", o.rotation_epsilon);
  visit("match_distance_m", o.match_distance_m);
  visit("min_inliers", o.min_inliers);
  visit("min_coverage", o.min_coverage);
  visit("max_rmse_m", o.max_rmse_m);
  visit("max_p95_m", o.max_p95_m);
  visit("max_cycle_translation_m", o.max_cycle_translation_m);
  visit("max_cycle_rotation_deg", o.max_cycle_rotation_deg);
  visit("max_correction_m", o.max_correction_m);
  visit("max_correction_deg", o.max_correction_deg);
  visit("min_normal_pairs", o.min_normal_pairs);
  visit("min_eigenvalue_ratio", o.min_eigenvalue_ratio);
}

template <typename Visit>
void GraphFields(PoseGraphOptions& o, Visit visit) {
  visit("prior_rotation_sigma_rad", o.prior_rotation_sigma_rad);
  visit("prior_translation_sigma_m", o.prior_translation_sigma_m);
  visit("odometry_rotation_sigma_rad", o.odometry_rotation_sigma_rad);
  visit("odometry_translation_sigma_m", o.odometry_translation_sigma_m);
  visit("max_iterations", o.max_iterations);
  visit("loop_huber_delta", o.loop_huber_delta);
}

template <typename Visit>
void AdmissionFields(LoopAdmissionOptions& o, Visit visit) {
  visit("loop_translation_sigma_m", o.loop_translation_sigma_m);
  visit("loop_rotation_sigma_rad", o.loop_rotation_sigma_rad);
  visit("max_hypothesis_translation_m", o.max_hypothesis_translation_m);
  visit("max_hypothesis_rotation_deg", o.max_hypothesis_rotation_deg);
  visit("correlated_submap_gap", o.correlated_submap_gap);
  visit("max_factor_residual", o.max_factor_residual);
  visit("max_accepted_residual_increase", o.max_accepted_residual_increase);
  visit("max_pose_correction_m", o.max_pose_correction_m);
  visit("max_pose_correction_deg", o.max_pose_correction_deg);
  visit("max_keyframes", o.max_keyframes);
  visit("max_loops", o.max_loops);
}

template <typename Visit>
void GeometryFields(GeometryOptions& o, Visit visit) {
  visit("voxel_size_m", o.voxel_size);
  visit("min_points_per_voxel", o.min_points_per_voxel);
  visit("max_plane_rms", o.max_plane_rms);
  visit("max_plane_ratio", o.max_plane_ratio);
  visit("max_voxels_per_submap", o.max_voxels_per_submap);
  visit("max_points_per_keyframe", o.max_points_per_keyframe);
  visit("max_resident_submaps", o.max_resident_submaps);
}

}  // namespace livo_atlas::loop_option_fields

#endif  // LIVO_ATLAS_SRC_BACKEND_LOOP_OPTION_FIELDS_H_
