#include "io/loop_admission_io.h"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <set>

#include "io/dataset_metadata.h"
#include "io/loop_diagnostics.h"
#include "io/loop_options.h"

namespace livo_atlas {
namespace fs = std::filesystem;

namespace loop_admission_io {
using dataset_metadata::Decimal;
using dataset_metadata::WriteTransform;

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
  if (name) yaml << name;
  yaml << "{"
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

void WriteQuery(cv::FileStorage& yaml, const LoopSubmapInfo& source,
                const LoopAdmissionInput& input,
                const LoopAdmissionResult& result, double elapsed_seconds) {
  yaml << "schema_version" << 1 << "query_id" << std::to_string(input.query_id)
       << "status" << LoopAdmissionStatusName(result.status) << "base_version"
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
    if (check.measurement) WriteCheck(yaml, "measurement", *check.measurement);
    if (check.prediction) WriteCheck(yaml, "prediction", *check.prediction);
    yaml << "}";
  }
  yaml << "]";
}

void WriteProposal(cv::FileStorage& yaml, const LoopAdmissionInput& input,
                   const LoopAdmissionResult& result) {
  const auto& proposal = *result.proposal;
  yaml << "schema_version" << 1 << "query_id" << std::to_string(input.query_id)
       << "pose_version" << std::to_string(input.poses.version + 1)
       << "frame_count" << std::to_string(input.poses.frame_count);
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
       << std::to_string(proposal.update.optimized_frame_count) << "tail_policy"
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
      result.candidates.begin(), result.candidates.end(), [&](const auto& row) {
        return row.candidate.target_id == proposal.loop.target.primary.id;
      });
  if (candidate == result.candidates.end())
    throw std::invalid_argument("Proposal has no candidate diagnostics");
  loop_diagnostics::WriteMatch(yaml, candidate->candidate,
                               proposal.verification);
  yaml << "}";
}

}  // namespace loop_admission_io

LoopAdmissionOptions ReadLoopAdmissionOptions(const fs::path& path) {
  const auto yaml = dataset_metadata::ReadYaml(path);
  return loop_admission_io::ReadOptions(yaml.root());
}

namespace loop_admission_io {
LoopAdmissionOptions ReadOptions(const cv::FileNode& yaml, bool with_geometry) {
  auto keys =
      std::set<std::string>{"search", "registration", "graph", "admission"};
  if (with_geometry) keys.insert("geometry");
  loop_options::ValidateKeys(yaml, keys);
  LoopAdmissionOptions options;
  loop_options::ReadSearch(yaml["search"], options.search, with_geometry);
  loop_options::ReadRegistration(yaml["registration"], options.registration,
                                 with_geometry);
  loop_options::ReadFields(
      yaml["graph"], options.graph,
      [](auto& o, auto visit) { GraphFields(o, visit); }, with_geometry);
  loop_options::ReadFields(
      yaml["admission"], options,
      [](auto& o, auto visit) { AdmissionFields(o, visit); }, with_geometry);
  ValidateLoopAdmissionOptions(options);
  return options;
}

void CheckUniqueKeys(const cv::FileNode& node) {
  if (!node.isMap() && !node.isSeq()) return;
  std::set<std::string> names;
  for (const auto& child : node) {
    if (node.isMap() && !names.insert(child.name()).second)
      throw std::invalid_argument("Duplicate record field: " + child.name());
    CheckUniqueKeys(child);
  }
}

namespace {
namespace meta = dataset_metadata;

void WritePoses(cv::FileStorage& yaml, const std::vector<KeyframePose>& poses) {
  yaml << "poses"
       << "[";
  for (const auto& pose : poses) {
    yaml << "{"
         << "id" << std::to_string(pose.id);
    meta::WriteTransform(yaml, "map_from_body", pose.map_from_body);
    yaml << "}";
  }
  yaml << "]";
}

std::vector<KeyframePose> ReadPoses(const cv::FileNode& node,
                                    std::uint64_t count) {
  meta::Sequence(node, count);
  std::vector<KeyframePose> result;
  for (const auto& pose : node) {
    loop_options::ValidateKeys(pose, {"id", "map_from_body"});
    result.push_back(
        {meta::Id(pose["id"]), meta::ReadTransform(pose["map_from_body"])});
  }
  return result;
}
}  // namespace

void WriteSnapshot(cv::FileStorage& yaml, const char* name,
                   const PoseSnapshot& snapshot) {
  ValidatePoseSnapshot(snapshot);
  yaml << name << "{"
       << "version" << std::to_string(snapshot.version) << "base_version"
       << std::to_string(snapshot.base_version) << "frame_count"
       << std::to_string(snapshot.frame_count) << "optimized_frame_count"
       << std::to_string(snapshot.optimized_frame_count) << "parent_frame"
       << (snapshot.parent_frame == PoseFrame::kMap ? "map" : "odom")
       << "tail_policy"
       << (snapshot.tail_policy == PoseTailPolicy::kProjectedLastOptimizedAnchor
               ? "projected"
               : "legacy");
  WritePoses(yaml, snapshot.poses);
  yaml << "}";
}

PoseSnapshot ReadSnapshot(const cv::FileNode& node) {
  loop_options::ValidateKeys(
      node, {"version", "base_version", "frame_count", "optimized_frame_count",
             "parent_frame", "tail_policy", "poses"});
  PoseSnapshot snapshot;
  snapshot.version = meta::Id(node["version"]);
  snapshot.base_version = meta::Id(node["base_version"]);
  snapshot.frame_count = meta::Id(node["frame_count"]);
  snapshot.optimized_frame_count = meta::Id(node["optimized_frame_count"]);
  const auto frame = meta::String(node["parent_frame"]);
  const auto tail = meta::String(node["tail_policy"]);
  if ((frame != "map" && frame != "odom") ||
      (tail != "projected" && tail != "legacy"))
    throw std::invalid_argument("Invalid snapshot convention");
  snapshot.parent_frame = frame == "map" ? PoseFrame::kMap : PoseFrame::kOdom;
  snapshot.tail_policy = tail == "projected"
                             ? PoseTailPolicy::kProjectedLastOptimizedAnchor
                             : PoseTailPolicy::kLegacyLastOptimizedAnchor;
  snapshot.poses = ReadPoses(node["poses"], snapshot.frame_count);
  ValidatePoseSnapshot(snapshot);
  return snapshot;
}

OnlineSubmap ReadMembership(const cv::FileNode& node) {
  loop_options::ValidateKeys(
      node, {"id", "anchor_keyframe_id", "membership_revision", "state",
             "keyframe_ids", "support_keyframe_ids"});
  OnlineSubmap group;
  group.primary.id = meta::Id(node["id"]);
  group.primary.anchor_keyframe_id = meta::Id(node["anchor_keyframe_id"]);
  group.membership_revision = meta::Id(node["membership_revision"]);
  if (meta::String(node["state"]) != "sealed" || !group.membership_revision)
    throw std::invalid_argument("Loop member must be sealed");
  group.state = OnlineSubmapState::kSealed;
  for (const auto* key : {"keyframe_ids", "support_keyframe_ids"}) {
    if (!node[key].isSeq()) throw std::invalid_argument("Invalid member list");
    auto& ids = std::string(key) == "keyframe_ids" ? group.primary.keyframe_ids
                                                   : group.support_keyframe_ids;
    for (const auto& id : node[key]) ids.push_back(meta::Id(id));
  }
  ValidateSubmap(group.primary);
  return group;
}

AcceptedLoop ReadLoop(const cv::FileNode& node) {
  AcceptedLoop result;
  result.target = ReadMembership(node["target"]);
  result.source = ReadMembership(node["source"]);
  const auto constraint = node["constraint"];
  loop_options::ValidateKeys(constraint,
                             {"from_id", "to_id", "rotation_sigma_rad",
                              "translation_sigma_m", "from_body_from_to_body"});
  result.constraint.from_id = meta::Id(constraint["from_id"]);
  result.constraint.to_id = meta::Id(constraint["to_id"]);
  result.constraint.rotation_sigma_rad =
      meta::Number(constraint["rotation_sigma_rad"]);
  result.constraint.translation_sigma_m =
      meta::Number(constraint["translation_sigma_m"]);
  result.constraint.from_body_from_to_body =
      meta::ReadTransform(constraint["from_body_from_to_body"]);
  return result;
}

PoseUpdate ReadUpdate(const cv::FileNode& node) {
  loop_options::ValidateKeys(
      node, {"base_version", "optimized_frame_count", "tail_policy", "poses"});
  PoseUpdate update;
  update.base_version = meta::Id(node["base_version"]);
  update.optimized_frame_count = meta::Id(node["optimized_frame_count"]);
  if (meta::String(node["tail_policy"]) != "projected_last_optimized_anchor")
    throw std::invalid_argument("Invalid loop update tail policy");
  update.tail_policy = PoseTailPolicy::kProjectedLastOptimizedAnchor;
  update.poses = ReadPoses(node["poses"], update.optimized_frame_count);
  return update;
}

bool SameMembership(const OnlineSubmap& a, const OnlineSubmap& b) {
  return a.primary.id == b.primary.id &&
         a.primary.anchor_keyframe_id == b.primary.anchor_keyframe_id &&
         a.primary.keyframe_ids == b.primary.keyframe_ids &&
         a.support_keyframe_ids == b.support_keyframe_ids &&
         a.membership_revision == b.membership_revision && a.state == b.state;
}

bool SameSnapshot(const PoseSnapshot& a, const PoseSnapshot& b) {
  if (a.version != b.version || a.base_version != b.base_version ||
      a.frame_count != b.frame_count ||
      a.optimized_frame_count != b.optimized_frame_count ||
      a.parent_frame != b.parent_frame || a.tail_policy != b.tail_policy ||
      a.poses.size() != b.poses.size())
    return false;
  for (std::size_t i = 0; i < a.poses.size(); ++i) {
    if (a.poses[i].id != b.poses[i].id ||
        (a.poses[i].map_from_body.rotation - b.poses[i].map_from_body.rotation)
                .cwiseAbs()
                .maxCoeff() > 1e-9 ||
        (a.poses[i].map_from_body.translation -
         b.poses[i].map_from_body.translation)
                .cwiseAbs()
                .maxCoeff() > 1e-9)
      return false;
  }
  return true;
}

}  // namespace loop_admission_io
}  // namespace livo_atlas
