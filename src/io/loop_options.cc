#include "io/loop_options.h"

namespace livo_atlas::loop_options {
namespace {

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

}  // namespace

void ReadSearch(const cv::FileNode& node, LoopSearchOptions& options) {
  ReadFields(node, options,
             [](auto& o, auto visit) { SearchFields(o, visit); });
}

void ReadRegistration(const cv::FileNode& node,
                      LoopRegistrationOptions& options) {
  ReadFields(node, options,
             [](auto& o, auto visit) { RegistrationFields(o, visit); });
}

void Write(cv::FileStorage& yaml, LoopSearchOptions search,
           LoopRegistrationOptions registration) {
  const auto write = [&](const char* name, auto value) {
    WriteField(yaml, name, value);
  };
  yaml << "search"
       << "{";
  SearchFields(search, write);
  yaml << "}"
       << "registration"
       << "{";
  RegistrationFields(registration, write);
  yaml << "}";
}

}  // namespace livo_atlas::loop_options
