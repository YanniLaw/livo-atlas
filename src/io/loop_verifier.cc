#include "io/loop_verifier.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <type_traits>

#include "io/dataset_metadata.h"
#include "livo_atlas/io/online_submap_store.h"
#include "livo_atlas/mapping/geometry_mapper.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;
using dataset_metadata::Decimal;
using dataset_metadata::WriteTransform;
using dataset_metadata::WriteYaml;

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

template <typename Fields, typename Options>
void ReadFields(const cv::FileNode& group, Options& options, Fields fields) {
  if (group.empty()) return;
  if (!group.isMap())
    throw std::invalid_argument("Loop option group must be a map");
  std::set<std::string> names;
  fields(options, [&](const char* name, auto& value) {
    names.insert(name);
    const auto node = group[name];
    if (node.empty()) return;
    if (!node.isReal() && !node.isInt())
      throw std::invalid_argument("Loop option must be numeric");
    const double number = static_cast<double>(node);
    using Value = std::decay_t<decltype(value)>;
    if (!std::isfinite(number))
      throw std::invalid_argument("Loop option must be finite");
    if constexpr (std::is_integral_v<Value>) {
      // Config integers must be exactly representable before casting.
      if (number < 0 || number > 9007199254740991.0 ||
          std::floor(number) != number ||
          number >= static_cast<double>(std::numeric_limits<Value>::max())) {
        throw std::invalid_argument("Loop count option is out of range");
      }
    }
    value = static_cast<Value>(number);
  });
  for (const auto& node : group) {
    if (!names.count(node.name()))
      throw std::invalid_argument("Unknown loop option: " + node.name());
  }
}

void WriteOptions(cv::FileStorage& yaml, LoopVerifierOptions options) {
  const auto write = [&](const char* name, auto value) {
    if constexpr (std::is_integral_v<decltype(value)>)
      yaml << name << std::to_string(value);
    else
      yaml << name << Decimal(value);
  };
  yaml << "search"
       << "{";
  SearchFields(options.search, write);
  yaml << "}"
       << "registration"
       << "{";
  RegistrationFields(options.registration, write);
  yaml << "}";
}

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
    const auto& m = *value.metrics;
    yaml << "inliers" << std::to_string(m.inliers) << "normal_pairs"
         << std::to_string(m.normal_pairs) << "source_coverage"
         << Decimal(m.source_coverage) << "target_coverage"
         << Decimal(m.target_coverage);
    if (m.rmse_m) yaml << "rmse_m" << Decimal(*m.rmse_m);
    if (m.p95_m) yaml << "p95_m" << Decimal(*m.p95_m);
    if (m.eigenvalue_ratio)
      yaml << "eigenvalue_ratio" << Decimal(*m.eigenvalue_ratio);
  }
  yaml << "}";
}

void WriteMatch(const fs::path& path, const LoopCandidate& candidate,
                const LoopVerification& result) {
  WriteYaml(path, [&](auto& yaml) {
    yaml << "schema_version" << 1 << "status" << LoopStatusName(result.status)
         << "pose_version" << std::to_string(result.pose_version)
         << "frame_count" << std::to_string(result.frame_count)
         << "optimized_frame_count"
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
             << "translation_m" << Decimal(error.translation_m)
             << "rotation_deg" << Decimal(error.rotation_deg) << "}";
      }
    }
    if (result.target_from_source)
      WriteTransform(yaml, "target_from_source", *result.target_from_source);
  });
}

bool Within(const fs::path& child, const fs::path& parent) {
  auto a = child.begin();
  for (auto b = parent.begin(); b != parent.end(); ++b, ++a) {
    if (a == child.end() || *a != *b) return false;
  }
  return true;
}

}  // namespace

LoopVerifierOptions ReadLoopVerifierOptions(const fs::path& path) {
  const auto yaml = dataset_metadata::ReadYaml(path);
  if (!yaml.root().isMap())
    throw std::invalid_argument("Loop configuration must be a map");
  for (const auto& node : yaml.root()) {
    if (node.name() != "search" && node.name() != "registration") {
      throw std::invalid_argument("Unknown loop option group: " + node.name());
    }
  }
  LoopVerifierOptions options;
  ReadFields(yaml["search"], options.search,
             [](auto& o, auto visit) { SearchFields(o, visit); });
  ReadFields(yaml["registration"], options.registration,
             [](auto& o, auto visit) { RegistrationFields(o, visit); });
  ValidateLoopSearchOptions(options.search);
  ValidateLoopRegistrationOptions(options.registration);
  return options;
}

void VerifyLoopDataset(const fs::path& dataset, const fs::path& output,
                       const LoopVerifierOptions& options) {
  ValidateLoopSearchOptions(options.search);
  ValidateLoopRegistrationOptions(options.registration);
  if (output.empty() ||
      Within(fs::weakly_canonical(output), fs::canonical(dataset))) {
    throw std::invalid_argument(
        "Loop output must be outside the input dataset");
  }
  if (!output.parent_path().empty())
    fs::create_directories(output.parent_path());
  if (!fs::create_directory(output))
    throw std::runtime_error("Loop output already exists");
  std::size_t candidates = 0, verified = 0;
  std::map<std::string, std::size_t> counts;
  std::size_t submap_count = 0, frame_count = 0, max_pinned_voxels = 0;
  GeometryOptions geometry;
  geometry.enabled = true;
  const auto summary = [&](bool complete) {
    WriteYaml(output / "summary.yml", [&](auto& yaml) {
      yaml << "schema_version" << 1 << "complete" << static_cast<int>(complete)
           << "input" << fs::canonical(dataset).string() << "pose_version"
           << "0"
           << "frame_count" << std::to_string(frame_count) << "sealed_submaps"
           << std::to_string(submap_count) << "candidates"
           << std::to_string(candidates) << "verified"
           << std::to_string(verified) << "max_pinned_voxels"
           << std::to_string(max_pinned_voxels) << "geometry_voxel_size_m"
           << Decimal(geometry.voxel_size) << "geometry_min_points"
           << geometry.min_points_per_voxel << "geometry_max_plane_rms"
           << Decimal(geometry.max_plane_rms) << "geometry_max_plane_ratio"
           << Decimal(geometry.max_plane_ratio) << "geometry_cache_submaps"
           << geometry.max_resident_submaps;
      WriteOptions(yaml, options);
      yaml << "statuses"
           << "{";
      for (const auto& count : counts)
        yaml << count.first << std::to_string(count.second);
      yaml << "}";
    });
  };
  try {
    summary(false);
    fs::create_directory(output / "matches");
    KeyframeStore frames(dataset);
    OnlineSubmapStore store(dataset / "online_submaps", frames);
    frame_count = frames.ids().size();
    submap_count = store.size();
    if (store.indexed_frame_count() != frame_count) {
      throw std::invalid_argument(
          "Loop verification requires a fully indexed dataset");
    }
    std::vector<double> timestamps;
    for (const auto id : frames.ids()) {
      const auto timestamp = frames.ReadTimestamp(id);
      if (!timestamps.empty() && timestamp < timestamps.back())
        throw std::invalid_argument("Loop dataset timestamps moved backwards");
      timestamps.push_back(timestamp);
    }
    auto poses = std::make_shared<const PoseSnapshot>(
        store.MaterializePoseSnapshot(0, frame_count));
    std::vector<OnlineSubmap> groups;
    std::vector<LoopSubmapInfo> infos;
    for (std::size_t id = 0; id < submap_count; ++id) {
      const auto group = store.Read(id);
      if (group.state != OnlineSubmapState::kSealed)
        throw std::invalid_argument("Loop dataset still has an active submap");
      groups.push_back(group);
      const auto first = group.primary.keyframe_ids.front(),
                 last = group.primary.keyframe_ids.back();
      infos.push_back({group, timestamps.at(first), timestamps.at(last),
                       poses->poses.at(first).map_from_body, 0, frame_count,
                       0});
    }
    GeometryMapper mapper(geometry, [&](std::uint64_t id) {
      return frames.ReadBodyCloud(id, geometry.max_points_per_keyframe);
    });
    mapper.Update(groups, poses);
    std::vector<LoopSubmapInfo> history;
    for (const auto& query : infos) {
      for (const auto& candidate :
           FindLoopCandidates(query, history, options.search)) {
        // References live for this candidate only, including evicted cache
        // data.
        const auto source = mapper.GetSubmap(candidate.source_id);
        const auto target = mapper.GetSubmap(candidate.target_id);
        max_pinned_voxels =
            std::max(max_pinned_voxels,
                     source.data->voxels.size() + target.data->voxels.size());
        const auto result = VerifyLoop(target, source, options.registration);
        WriteMatch(output / "matches" /
                       (std::to_string(candidate.target_id) + "-" +
                        std::to_string(candidate.source_id) + ".yml"),
                   candidate, result);
        ++candidates;
        verified += result.status == LoopStatus::kVerified;
        ++counts[LoopStatusName(result.status)];
      }
      history.push_back(query);
    }
    summary(true);
  } catch (const std::exception& error) {
    // Keep already written evidence; never reuse this output for a retry.
    try {
      summary(false);
      WriteYaml(output / "failure.yml",
                [&](auto& yaml) { yaml << "error" << error.what(); });
    } catch (...) {
      // The original I/O failure remains the useful cause if reporting also
      // fails.
    }
    throw;
  }
}

}  // namespace livo_atlas
