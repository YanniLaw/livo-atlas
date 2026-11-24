#include "io/loop_admission_io.h"

#include <algorithm>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <type_traits>

#include "pose_graph/loop_option_fields.h"
#include "io/dataset_metadata.h"
#include "io/loop_diagnostics.h"
#include "io/loop_options.h"
#include "livo_atlas/io/online_submap_store.h"

namespace livo_atlas {
namespace fs = std::filesystem;

namespace loop_admission_io {
using dataset_metadata::Decimal;
using dataset_metadata::WriteTransform;
using loop_option_fields::AdmissionFields;
using loop_option_fields::GeometryFields;
using loop_option_fields::GraphFields;

void WriteOptions(cv::FileStorage& yaml, LoopAdmissionOptions options) {
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
       << "enabled" << static_cast<int>(options.geometry.enabled);
  GeometryFields(options.geometry, [&](const char* name, auto value) {
    if constexpr (std::is_integral_v<decltype(value)>)
      yaml << name << value;
    else
      yaml << name << Decimal(value);
  });
  yaml << "}";
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
  if (result.detected_target_id)
    yaml << "detected_target_id" << std::to_string(*result.detected_target_id);
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
    WriteTransform(yaml, "map_from_body", pose.parent_from_body());
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
namespace {

void ReadGeometryOptions(const cv::FileNode& node, GeometryOptions& geometry) {
  std::set<std::string> keys{"enabled"};
  GeometryFields(geometry, [&](const char* name, auto&) { keys.insert(name); });
  loop_options::ValidateKeys(node, keys);
  if (dataset_metadata::Integer(node["enabled"]) != 1)
    throw std::invalid_argument("Loop geometry must be enabled");
  GeometryFields(geometry, [&](const char* name, auto& value) {
    using Value = std::decay_t<decltype(value)>;
    if constexpr (std::is_integral_v<Value>)
      value = dataset_metadata::Integer(node[name]);
    else
      value = dataset_metadata::Number(node[name]);
  });
}

}  // namespace

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
  if (with_geometry) ReadGeometryOptions(yaml["geometry"], options.geometry);
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
    meta::WriteTransform(yaml, "map_from_body", pose.parent_from_body());
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
        (a.poses[i].parent_from_body().rotation -
         b.poses[i].parent_from_body().rotation)
                .cwiseAbs()
                .maxCoeff() > 1e-9 ||
        (a.poses[i].parent_from_body().translation -
         b.poses[i].parent_from_body().translation)
                .cwiseAbs()
                .maxCoeff() > 1e-9)
      return false;
  }
  return true;
}

namespace {
void RequireDiagnostic(bool condition) {
  if (!condition)
    throw std::invalid_argument("Invalid loop admission diagnostic record");
}

bool Flag(const cv::FileNode& node) {
  const auto value = meta::Integer(node);
  RequireDiagnostic(value == 0 || value == 1);
  return value != 0;
}

void Nonnegative(const cv::FileNode& node) {
  RequireDiagnostic(meta::Number(node) >= 0);
}

void CheckStatus(const cv::FileNode& node) {
  const std::set<std::string> statuses = {
      "verified",      "insufficient_points",    "resource_limit",
      "not_converged", "iteration_limit",        "low_overlap",
      "high_residual", "inconsistent_transform", "large_correction",
      "degenerate"};
  RequireDiagnostic(statuses.count(meta::String(node)) != 0);
}

void CheckMetrics(const cv::FileNode& node) {
  const auto inliers = meta::Id(node["inliers"]);
  RequireDiagnostic(meta::Id(node["normal_pairs"]) <= inliers);
  for (const auto* key : {"source_coverage", "target_coverage"}) {
    const auto value = meta::Number(node[key]);
    RequireDiagnostic(value >= 0 && value <= 1);
  }
  for (const auto* key : {"rmse_m", "p95_m", "eigenvalue_ratio"})
    if (!node[key].empty()) Nonnegative(node[key]);
  RequireDiagnostic(!inliers ||
                    (!node["rmse_m"].empty() && !node["p95_m"].empty()));
}

bool CheckAlignment(const cv::FileNode& node) {
  loop_options::ValidateKeys(
      node, {"iterations", "converged", "transform", "metrics_computed",
             "inliers", "normal_pairs", "source_coverage", "target_coverage",
             "rmse_m", "p95_m", "eigenvalue_ratio"});
  RequireDiagnostic(meta::Integer(node["iterations"]) >= 0);
  const auto converged = Flag(node["converged"]);
  const auto metrics = Flag(node["metrics_computed"]);
  if (!node["transform"].empty()) {
    (void)meta::ReadTransform(node["transform"]);
    RequireDiagnostic(converged);
  }
  if (metrics) {
    RequireDiagnostic(!node["transform"].empty());
    CheckMetrics(node);
  } else {
    for (const auto* key :
         {"inliers", "normal_pairs", "source_coverage", "target_coverage",
          "rmse_m", "p95_m", "eigenvalue_ratio"})
      RequireDiagnostic(node[key].empty());
  }
  return metrics;
}

void CheckProvenance(const cv::FileNode& node, const OnlineSubmap& member) {
  loop_options::ValidateKeys(
      node, {"submap_id", "anchor_keyframe_id", "membership_revision",
             "representation_revision"});
  RequireDiagnostic(meta::Id(node["submap_id"]) == member.primary.id &&
                    meta::Id(node["anchor_keyframe_id"]) ==
                        member.primary.anchor_keyframe_id &&
                    meta::Id(node["membership_revision"]) ==
                        member.membership_revision &&
                    meta::Id(node["representation_revision"]) > 0);
}

void CheckResiduals(const cv::FileNode& node, std::uint64_t count,
                    bool odometry) {
  RequireDiagnostic(node.isSeq() && (!odometry || node.size() == count - 1));
  std::uint64_t index = 0;
  for (const auto& row : node) {
    loop_options::ValidateKeys(
        row,
        {"from_id", "to_id", "initial_whitened_norm", "final_whitened_norm",
         "final_rotation_rad", "final_translation_m"});
    const auto from = meta::Id(row["from_id"]), to = meta::Id(row["to_id"]);
    RequireDiagnostic(from < to && to < count &&
                      (!odometry || (from == index && to == index + 1)));
    for (const auto* key : {"initial_whitened_norm", "final_whitened_norm",
                            "final_rotation_rad", "final_translation_m"})
      Nonnegative(row[key]);
    ++index;
  }
}

void CheckTrial(const cv::FileNode& node, const PoseSnapshot& base) {
  loop_options::ValidateKeys(
      node, {"initial_error", "final_error", "iterations",
             "iteration_limit_reached", "graph_build_seconds", "solve_seconds",
             "odometry_residuals", "loop_residuals"});
  for (const auto* key :
       {"initial_error", "final_error", "graph_build_seconds", "solve_seconds"})
    Nonnegative(node[key]);
  (void)meta::Id(node["iterations"]);
  (void)Flag(node["iteration_limit_reached"]);
  CheckResiduals(node["odometry_residuals"], base.frame_count, true);
  CheckResiduals(node["loop_residuals"], base.frame_count, false);
  RequireDiagnostic(node["loop_residuals"].size() == base.version + 1);
}

bool CheckGeometry(const cv::FileNode& node) {
  loop_options::ValidateKeys(node, {"status", "forward", "reverse"});
  CheckStatus(node["status"]);
  for (const auto* direction : {"forward", "reverse"}) {
    const auto metrics = node[direction];
    loop_options::ValidateKeys(
        metrics, {"inliers", "normal_pairs", "source_coverage",
                  "target_coverage", "rmse_m", "p95_m", "eigenvalue_ratio"});
    CheckMetrics(metrics);
  }
  return meta::String(node["status"]) == "verified";
}
}  // namespace

void CheckVerification(const cv::FileNode& node, const OnlineSubmap& target,
                       const OnlineSubmap& source, const PoseSnapshot& base) {
  loop_options::ValidateKeys(node, {"target_id",
                                    "source_id",
                                    "schema_version",
                                    "status",
                                    "pose_version",
                                    "frame_count",
                                    "optimized_frame_count",
                                    "anchor_distance_m",
                                    "target_points",
                                    "source_points",
                                    "elapsed_seconds",
                                    "target",
                                    "source",
                                    "initial_target_from_source",
                                    "forward",
                                    "reverse",
                                    "target_cycle",
                                    "source_cycle",
                                    "forward_correction",
                                    "reverse_correction",
                                    "target_from_source"});
  CheckStatus(node["status"]);
  RequireDiagnostic(meta::Integer(node["schema_version"]) == 1 &&
                    meta::Id(node["pose_version"]) == base.version &&
                    meta::Id(node["frame_count"]) == base.frame_count &&
                    meta::Id(node["optimized_frame_count"]) ==
                        base.optimized_frame_count);
  CheckProvenance(node["target"], target);
  CheckProvenance(node["source"], source);
  (void)meta::Id(node["target_points"]);
  (void)meta::Id(node["source_points"]);
  Nonnegative(node["anchor_distance_m"]);
  Nonnegative(node["elapsed_seconds"]);
  (void)meta::ReadTransform(node["initial_target_from_source"]);
  const auto forward = CheckAlignment(node["forward"]);
  const auto reverse = CheckAlignment(node["reverse"]);
  for (const auto* key : {"target_cycle", "source_cycle", "forward_correction",
                          "reverse_correction"}) {
    const auto error = node[key];
    RequireDiagnostic(!error.empty() == (forward && reverse));
    if (error.empty()) continue;
    loop_options::ValidateKeys(error, {"translation_m", "rotation_deg"});
    Nonnegative(error["translation_m"]);
    Nonnegative(error["rotation_deg"]);
  }
  const auto verified = meta::String(node["status"]) == "verified";
  RequireDiagnostic(!node["target_from_source"].empty() == verified);
  if (verified) {
    RequireDiagnostic(forward && reverse);
    (void)meta::ReadTransform(node["target_from_source"]);
    for (const auto* direction : {"forward", "reverse"})
      RequireDiagnostic(!node[direction]["eigenvalue_ratio"].empty());
  }
}

void CheckAdmissionDiagnostics(const cv::FileNode& node,
                               const OnlineSubmapStore& store,
                               const std::vector<double>& timestamps,
                               std::uint64_t query_id,
                               const PoseSnapshot& base) {
  loop_options::ValidateKeys(
      node, {"schema_version", "query_id", "status", "base_version",
             "pose_version", "frame_count", "first_timestamp", "last_timestamp",
             "elapsed_seconds", "source", "candidates", "optimization_error",
             "trial", "geometry_checks", "detected_target_id"});
  const auto status = meta::String(node["status"]);
  const std::set<std::string> statuses = {
      "no_candidate",        "geometry_rejected",
      "ambiguous",           "correlated",
      "resource_limit",      "optimization_rejected",
      "constraint_residual", "pose_correction",
      "geometry_changed",    "proposed",
      "detected"};
  RequireDiagnostic(statuses.count(status) && base.frame_count &&
                    meta::Integer(node["schema_version"]) == 1 &&
                    meta::Id(node["query_id"]) == query_id &&
                    meta::Id(node["base_version"]) == base.version &&
                    meta::Id(node["pose_version"]) ==
                        base.version + (status == "proposed") &&
                    meta::Id(node["frame_count"]) == base.frame_count);
  Nonnegative(node["elapsed_seconds"]);
  const auto source = ReadMembership(node["source"]);
  RequireDiagnostic(source.primary.id == query_id &&
                    source.primary.keyframe_ids.back() < base.frame_count &&
                    SameMembership(source, store.Read(query_id)) &&
                    meta::Number(node["first_timestamp"]) ==
                        timestamps.at(source.primary.keyframe_ids.front()) &&
                    meta::Number(node["last_timestamp"]) ==
                        timestamps.at(source.primary.keyframe_ids.back()));
  const auto candidates = node["candidates"];
  RequireDiagnostic(candidates.isSeq());
  const auto detected_id = node["detected_target_id"];
  std::optional<std::uint64_t> detected_target;
  if (!detected_id.empty()) detected_target = meta::Id(detected_id);
  RequireDiagnostic((status == "detected") == detected_target.has_value());
  std::set<std::uint64_t> targets;
  for (const auto& row : candidates) {
    RequireDiagnostic(row.isMap());
    const auto target_id = meta::Id(row["target_id"]);
    RequireDiagnostic(target_id < query_id &&
                      meta::Id(row["source_id"]) == query_id &&
                      targets.insert(target_id).second);
    const auto target = store.Read(target_id);
    RequireDiagnostic(target.state == OnlineSubmapState::kSealed &&
                      target.primary.keyframe_ids.back() < base.frame_count);
    CheckVerification(row, target, source, base);
    if (detected_target == target_id)
      RequireDiagnostic(meta::String(row["status"]) == "verified");
  }
  RequireDiagnostic(!detected_target || targets.count(*detected_target));
  const bool early = status == "no_candidate" || status == "resource_limit";
  RequireDiagnostic(early == (candidates.size() == 0));
  const bool error = !node["optimization_error"].empty();
  const bool trial = !node["trial"].empty();
  const bool post_trial = status == "constraint_residual" ||
                          status == "pose_correction" ||
                          status == "geometry_changed" || status == "proposed";
  RequireDiagnostic(status == "optimization_rejected"
                        ? error != trial
                        : !error && trial == post_trial);
  if (error)
    RequireDiagnostic(!meta::String(node["optimization_error"]).empty());
  if (trial) CheckTrial(node["trial"], base);
  const auto checks = node["geometry_checks"];
  RequireDiagnostic(checks.isSeq());
  const bool post_geometry =
      status == "geometry_changed" || status == "proposed";
  RequireDiagnostic(post_geometry == (checks.size() != 0));
  std::set<std::pair<std::uint64_t, std::uint64_t>> checked;
  std::size_t new_constraints = 0;
  for (const auto& row : checks) {
    loop_options::ValidateKeys(row, {"target_id", "source_id", "new_constraint",
                                     "measurement", "prediction"});
    const auto target_id = meta::Id(row["target_id"]);
    const auto source_id = meta::Id(row["source_id"]);
    RequireDiagnostic(target_id < source_id && source_id <= query_id &&
                      checked.emplace(target_id, source_id).second);
    const bool is_new = Flag(row["new_constraint"]);
    RequireDiagnostic(!is_new ||
                      (source_id == query_id && targets.count(target_id)));
    new_constraints += is_new;
    const auto measured = CheckGeometry(row["measurement"]);
    RequireDiagnostic(measured != row["prediction"].empty());
    const auto predicted = measured && CheckGeometry(row["prediction"]);
    RequireDiagnostic(status != "proposed" || predicted);
  }
  RequireDiagnostic(
      status != "proposed" ||
      (new_constraints == 1 && checks.size() == base.version + 1));
}

}  // namespace loop_admission_io
}  // namespace livo_atlas
