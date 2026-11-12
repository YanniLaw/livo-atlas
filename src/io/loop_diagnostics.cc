#include "io/loop_diagnostics.h"

#include <array>
#include <cstddef>
#include <string>

#include "io/dataset_metadata.h"

namespace livo_atlas::loop_diagnostics {
namespace {
using dataset_metadata::Decimal;
using dataset_metadata::WriteTransform;

void WriteProvenance(cv::FileStorage& yaml, const char* name,
                     const LoopProvenance& value) {
  yaml << name << "{"
       << "submap_id" << std::to_string(value.submap_id) << "anchor_keyframe_id"
       << std::to_string(value.anchor_keyframe_id) << "membership_revision"
       << std::to_string(value.membership_revision) << "representation_revision"
       << std::to_string(value.representation_revision) << "}";
}

void WriteAlignment(cv::FileStorage& yaml, const char* name,
                    const LoopAlignment& value) {
  yaml << name << "{"
       << "iterations" << value.iterations << "converged"
       << static_cast<int>(value.converged);
  if (value.transform) WriteTransform(yaml, "transform", *value.transform);
  yaml << "metrics_computed" << static_cast<int>(value.metrics.has_value());
  if (value.metrics) {
    WriteMetrics(yaml, *value.metrics);
  }
  yaml << "}";
}

}  // namespace

void WriteMetrics(cv::FileStorage& yaml, const LoopMetrics& m) {
  yaml << "inliers" << std::to_string(m.inliers) << "normal_pairs"
       << std::to_string(m.normal_pairs) << "source_coverage"
       << Decimal(m.source_coverage) << "target_coverage"
       << Decimal(m.target_coverage);
  if (m.rmse_m) yaml << "rmse_m" << Decimal(*m.rmse_m);
  if (m.p95_m) yaml << "p95_m" << Decimal(*m.p95_m);
  if (m.eigenvalue_ratio)
    yaml << "eigenvalue_ratio" << Decimal(*m.eigenvalue_ratio);
}

void WriteMatch(cv::FileStorage& yaml, const LoopCandidate& candidate,
                const LoopVerification& result) {
  yaml << "schema_version" << 1 << "status" << LoopStatusName(result.status)
       << "pose_version" << std::to_string(result.pose_version) << "frame_count"
       << std::to_string(result.frame_count) << "optimized_frame_count"
       << std::to_string(result.optimized_frame_count) << "anchor_distance_m"
       << Decimal(candidate.anchor_distance_m) << "target_points"
       << std::to_string(result.target_points) << "source_points"
       << std::to_string(result.source_points) << "elapsed_seconds"
       << Decimal(result.elapsed_seconds);
  WriteProvenance(yaml, "target", result.target);
  WriteProvenance(yaml, "source", result.source);
  WriteTransform(yaml, "initial_target_from_source",
                 result.initial_target_from_source);
  WriteAlignment(yaml, "forward", result.forward);
  WriteAlignment(yaml, "reverse", result.reverse);
  if (result.transform_errors) {
    const std::array<const char*, 4> names = {"target_cycle", "source_cycle",
                                              "forward_correction",
                                              "reverse_correction"};
    for (std::size_t i = 0; i < names.size(); ++i) {
      const auto& error = (*result.transform_errors)[i];
      yaml << names[i] << "{"
           << "translation_m" << Decimal(error.translation_m) << "rotation_deg"
           << Decimal(error.rotation_deg) << "}";
    }
  }
  if (result.target_from_source)
    WriteTransform(yaml, "target_from_source", *result.target_from_source);
}

}  // namespace livo_atlas::loop_diagnostics
