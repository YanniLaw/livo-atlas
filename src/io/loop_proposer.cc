#include "io/loop_proposer.h"

#include <algorithm>
#include <chrono>
#include <exception>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "io/dataset_metadata.h"
#include "io/loop_diagnostics.h"
#include "io/loop_options.h"
#include "livo_atlas/io/online_submap_store.h"
#include "livo_atlas/mapping/geometry_mapper.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;
using dataset_metadata::Decimal;
using dataset_metadata::WriteTransform;
using dataset_metadata::WriteYaml;

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

void WriteOptions(cv::FileStorage& yaml, LoopAdmissionOptions options,
                  const GeometryOptions& geometry) {
  loop_options::Write(yaml, options.search, options.registration);
  const auto write = [&](const char* name, auto value) {
    loop_options::WriteField(yaml, name, value);
  };
  yaml << "graph"
       << "{";
  GraphFields(options.graph, write);
  yaml << "}"
       << "admission"
       << "{";
  AdmissionFields(options, write);
  yaml << "}"
       << "geometry"
       << "{"
       << "enabled" << static_cast<int>(geometry.enabled) << "voxel_size_m"
       << Decimal(geometry.voxel_size) << "min_points_per_voxel"
       << geometry.min_points_per_voxel << "max_plane_rms"
       << Decimal(geometry.max_plane_rms) << "max_plane_ratio"
       << Decimal(geometry.max_plane_ratio) << "max_voxels_per_submap"
       << geometry.max_voxels_per_submap << "max_points_per_keyframe"
       << geometry.max_points_per_keyframe << "max_resident_submaps"
       << geometry.max_resident_submaps << "}";
}

void WriteMembership(cv::FileStorage& yaml, const char* name,
                     const OnlineSubmap& group) {
  yaml << name << "{"
       << "id" << std::to_string(group.primary.id) << "anchor_keyframe_id"
       << std::to_string(group.primary.anchor_keyframe_id)
       << "membership_revision" << std::to_string(group.membership_revision)
       << "state"
       << "sealed"
       << "keyframe_ids"
       << "[";
  for (auto id : group.primary.keyframe_ids) yaml << std::to_string(id);
  yaml << "]"
       << "support_keyframe_ids"
       << "[";
  for (auto id : group.support_keyframe_ids) yaml << std::to_string(id);
  yaml << "]"
       << "}";
}

void WriteResiduals(cv::FileStorage& yaml, const char* name,
                    const std::vector<FactorResidual>& residuals) {
  yaml << name << "[";
  for (const auto& r : residuals) {
    yaml << "{"
         << "from_id" << std::to_string(r.from_id) << "to_id"
         << std::to_string(r.to_id) << "initial_whitened_norm"
         << Decimal(r.initial_whitened_norm) << "final_whitened_norm"
         << Decimal(r.final_whitened_norm) << "final_rotation_rad"
         << Decimal(r.final_rotation_rad) << "final_translation_m"
         << Decimal(r.final_translation_m) << "}";
  }
  yaml << "]";
}

void WriteCheck(cv::FileStorage& yaml, const char* name,
                const LoopGeometryCheck& check) {
  yaml << name << "{"
       << "status" << LoopStatusName(check.status) << "forward"
       << "{";
  loop_diagnostics::WriteMetrics(yaml, check.forward);
  yaml << "}"
       << "reverse"
       << "{";
  loop_diagnostics::WriteMetrics(yaml, check.reverse);
  yaml << "}"
       << "}";
}

void WriteQuery(const fs::path& path, const LoopSubmapInfo& source,
                const LoopAdmissionInput& input,
                const LoopAdmissionResult& result, double elapsed_seconds) {
  WriteYaml(path, [&](auto& yaml) {
    yaml << "schema_version" << 1 << "query_id"
         << std::to_string(input.query_id) << "status"
         << LoopAdmissionStatusName(result.status) << "base_version"
         << std::to_string(input.poses.version) << "pose_version"
         << std::to_string(input.poses.version + (result.proposal ? 1 : 0))
         << "frame_count" << std::to_string(input.poses.frame_count)
         << "first_timestamp" << Decimal(source.first_timestamp)
         << "last_timestamp" << Decimal(source.last_timestamp)
         << "elapsed_seconds" << Decimal(elapsed_seconds);
    WriteMembership(yaml, "source", source.membership);
    yaml << "candidates"
         << "[";
    for (const auto& candidate : result.candidates) {
      yaml << "{"
           << "target_id" << std::to_string(candidate.candidate.target_id)
           << "source_id" << std::to_string(candidate.candidate.source_id);
      loop_diagnostics::WriteMatch(yaml, candidate.candidate,
                                   candidate.verification);
      yaml << "}";
    }
    yaml << "]";
    if (result.optimization_error)
      yaml << "optimization_error" << *result.optimization_error;
    if (result.trial) {
      const auto& trial = *result.trial;
      yaml << "trial"
           << "{"
           << "initial_error" << Decimal(trial.initial_error) << "final_error"
           << Decimal(trial.final_error) << "iterations"
           << std::to_string(trial.iterations) << "iteration_limit_reached"
           << static_cast<int>(trial.iteration_limit_reached)
           << "graph_build_seconds" << Decimal(trial.graph_build_seconds)
           << "solve_seconds" << Decimal(trial.solve_seconds);
      WriteResiduals(yaml, "odometry_residuals", trial.odometry_residuals);
      WriteResiduals(yaml, "loop_residuals", trial.loop_residuals);
      yaml << "}";
    }
    yaml << "geometry_checks"
         << "[";
    for (const auto& check : result.geometry_checks) {
      yaml << "{"
           << "target_id" << std::to_string(check.target_id) << "source_id"
           << std::to_string(check.source_id) << "new_constraint"
           << static_cast<int>(check.new_constraint);
      if (check.measurement)
        WriteCheck(yaml, "measurement", *check.measurement);
      if (check.prediction) WriteCheck(yaml, "prediction", *check.prediction);
      yaml << "}";
    }
    yaml << "]";
  });
}

void WriteProposal(const fs::path& path, const LoopAdmissionInput& input,
                   const LoopAdmissionResult& result) {
  const auto& proposal = *result.proposal;
  WriteYaml(path, [&](auto& yaml) {
    yaml << "schema_version" << 1 << "query_id"
         << std::to_string(input.query_id) << "pose_version"
         << std::to_string(input.poses.version + 1) << "frame_count"
         << std::to_string(input.poses.frame_count);
    WriteMembership(yaml, "target", proposal.loop.target);
    WriteMembership(yaml, "source", proposal.loop.source);
    const auto& c = proposal.loop.constraint;
    yaml << "constraint"
         << "{"
         << "from_id" << std::to_string(c.from_id) << "to_id"
         << std::to_string(c.to_id) << "rotation_sigma_rad"
         << Decimal(c.rotation_sigma_rad) << "translation_sigma_m"
         << Decimal(c.translation_sigma_m);
    WriteTransform(yaml, "from_body_from_to_body", c.from_body_from_to_body);
    yaml << "}"
         << "update"
         << "{"
         << "base_version" << std::to_string(proposal.update.base_version)
         << "optimized_frame_count"
         << std::to_string(proposal.update.optimized_frame_count)
         << "tail_policy"
         << "projected_last_optimized_anchor"
         << "poses"
         << "[";
    for (const auto& pose : proposal.update.poses) {
      yaml << "{"
           << "id" << std::to_string(pose.id);
      WriteTransform(yaml, "map_from_body", pose.map_from_body);
      yaml << "}";
    }
    yaml << "]"
         << "}"
         << "verification"
         << "{";
    const auto candidate = std::find_if(
        result.candidates.begin(), result.candidates.end(),
        [&](const auto& row) {
          return row.candidate.target_id == proposal.loop.target.primary.id;
        });
    loop_diagnostics::WriteMatch(yaml, candidate->candidate,
                                 proposal.verification);
    yaml << "}";
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

LoopAdmissionOptions ReadLoopAdmissionOptions(const fs::path& path) {
  const auto yaml = dataset_metadata::ReadYaml(path);
  loop_options::ValidateKeys(yaml.root(),
                             {"search", "registration", "graph", "admission"});
  LoopAdmissionOptions options;
  loop_options::ReadSearch(yaml["search"], options.search);
  loop_options::ReadRegistration(yaml["registration"], options.registration);
  loop_options::ReadFields(yaml["graph"], options.graph,
                           [](auto& o, auto visit) { GraphFields(o, visit); });
  loop_options::ReadFields(yaml["admission"], options, [](auto& o, auto visit) {
    AdmissionFields(o, visit);
  });
  ValidateLoopAdmissionOptions(options);
  return options;
}

void ProposeLoopDataset(const fs::path& dataset, const fs::path& output,
                        const LoopAdmissionOptions& options) {
  ValidateLoopAdmissionOptions(options);
  const auto input_path = fs::canonical(dataset);
  if (output.empty() || Within(fs::weakly_canonical(output), input_path)) {
    throw std::invalid_argument(
        "Loop output must be outside the input dataset");
  }
  if (!output.parent_path().empty())
    fs::create_directories(output.parent_path());
  if (!fs::create_directory(output))
    throw std::runtime_error("Loop output already exists");
  LoopAdmissionInput input;
  std::size_t queries = 0, proposals = 0, frame_count = 0, submap_count = 0;
  std::map<std::string, std::size_t> counts;
  GeometryOptions geometry;
  geometry.enabled = true;
  const auto summary = [&](bool complete) {
    // Keep the previous incomplete summary if the final write cannot finish.
    const auto pending = output / ".summary.pending.yml";
    WriteYaml(pending, [&](auto& yaml) {
      yaml << "schema_version" << 1 << "complete" << static_cast<int>(complete)
           << "input" << input_path.string() << "frontier"
           << "closed_dataset"
           << "frame_count" << std::to_string(frame_count) << "sealed_submaps"
           << std::to_string(submap_count) << "query_count"
           << std::to_string(queries) << "proposal_count"
           << std::to_string(proposals) << "pose_version"
           << std::to_string(input.poses.version);
      WriteOptions(yaml, options, geometry);
      yaml << "statuses"
           << "{";
      for (const auto& count : counts)
        yaml << count.first << std::to_string(count.second);
      yaml << "}";
    });
    fs::rename(pending, output / "summary.yml");
  };
  try {
    summary(false);
    fs::create_directory(output / "queries");
    fs::create_directory(output / "proposals");
    KeyframeStore frames(dataset);
    OnlineSubmapStore store(dataset / "online_submaps", frames);
    frame_count = frames.ids().size();
    submap_count = store.size();
    if (store.indexed_frame_count() != frame_count) {
      throw std::invalid_argument(
          "Loop admission requires a fully indexed dataset");
    }
    std::vector<double> timestamps;
    for (auto id : frames.ids()) {
      const auto timestamp = frames.ReadTimestamp(id);
      if (!timestamps.empty() && timestamp < timestamps.back())
        throw std::invalid_argument("Loop dataset timestamps moved backwards");
      timestamps.push_back(timestamp);
    }
    input.poses = store.MaterializePoseSnapshot(0, frame_count);
    for (const auto& pose : input.poses.poses)
      input.raw_poses.push_back(pose.map_from_body);
    std::vector<OnlineSubmap> groups;
    for (std::size_t id = 0; id < submap_count; ++id) {
      const auto group = store.Read(id);
      if (group.state != OnlineSubmapState::kSealed)
        throw std::invalid_argument("Loop dataset still has an active submap");
      groups.push_back(group);
      input.submaps.push_back(
          {group, timestamps.at(group.primary.keyframe_ids.front()),
           timestamps.at(group.primary.keyframe_ids.back()),
           input.poses.poses.at(group.primary.anchor_keyframe_id).map_from_body,
           0, frame_count, 0});
    }
    std::unique_ptr<GeometryMapper> mapper;
    std::shared_ptr<const PoseSnapshot> view;
    const LoopGeometryReader reader = [&](auto id, const auto& poses) {
      // Every trial first reads current geometry, then preview geometry. Reset
      // on a view change so rejected previews cannot poison the next query.
      // A single three-submap cache is alive at a time.
      if (!view || view->version != poses.version) {
        mapper.reset();
        view = std::make_shared<const PoseSnapshot>(poses);
        mapper = std::make_unique<GeometryMapper>(
            geometry, [&](std::uint64_t frame_id) {
              return frames.ReadBodyCloud(frame_id,
                                          geometry.max_points_per_keyframe);
            });
        mapper->Update(groups, view);
      }
      return mapper->GetSubmap(id);
    };
    for (const auto& source : input.submaps) {
      input.query_id = source.membership.primary.id;
      const auto start = std::chrono::steady_clock::now();
      const auto result = ProposeLoopUpdate(input, reader, options);
      const double elapsed = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - start)
                                 .count();
      std::optional<PoseSnapshot> next_poses;
      std::vector<AcceptedLoop> next_loops;
      if (result.proposal) {
        next_poses = MakePoseSnapshot(result.proposal->update, input.poses,
                                      input.raw_poses);
        next_loops = input.accepted_loops;
        next_loops.push_back(result.proposal->loop);
        WriteProposal(output / "proposals" /
                          (std::to_string(next_poses->version) + ".yml"),
                      input, result);
      }
      WriteQuery(output / "queries" / (std::to_string(input.query_id) + ".yml"),
                 source, input, result, elapsed);
      // All allocations and writes succeed before the two no-throw swaps.
      if (next_poses) {
        std::swap(input.poses, *next_poses);
        input.accepted_loops.swap(next_loops);
        ++proposals;
        for (auto& info : input.submaps) {
          info.parent_from_submap =
              input.poses.poses[info.membership.primary.anchor_keyframe_id]
                  .map_from_body;
          info.pose_version = input.poses.version;
          info.optimized_frame_count = input.poses.optimized_frame_count;
        }
      }
      ++queries;
      ++counts[LoopAdmissionStatusName(result.status)];
    }
    summary(true);
  } catch (...) {
    const auto failure = std::current_exception();
    // Reporting may itself fail (for example, a full disk). Preserve the
    // original exception and every record already written.
    try {
      std::string message = "Unknown exception";
      try {
        std::rethrow_exception(failure);
      } catch (const std::exception& error) {
        message = error.what();
      } catch (...) {
      }
      summary(false);
      WriteYaml(output / "failure.yml",
                [&](auto& yaml) { yaml << "error" << message; });
    } catch (...) {
    }
    std::rethrow_exception(failure);
  }
}

}  // namespace livo_atlas
