#include "livo_atlas/io/loop_closure_store.h"

#include <algorithm>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>

#include "io/dataset_metadata.h"
#include "io/loop_admission_io.h"
#include "io/loop_options.h"
#include "livo_atlas/io/keyframe_store.h"
#include "livo_atlas/io/online_submap_store.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;
namespace meta = dataset_metadata;
namespace io = loop_admission_io;

void Require(bool condition, const std::string& message) {
  if (!condition) throw std::invalid_argument(message);
}

std::string SourceMetadata(const fs::path& dataset) {
  const auto path = dataset / "dataset.yml";
  Require(fs::file_size(path) <= 64 * 1024, "Dataset metadata exceeds 64 KiB");
  std::ifstream input(path, std::ios::binary);
  input.exceptions(std::ios::badbit);
  Require(input.is_open(), "Cannot read dataset metadata");
  return {std::istreambuf_iterator<char>(input), {}};
}

template <typename Write>
void Publish(const fs::path& path, bool replace, Write write) {
  if (!replace && fs::exists(path))
    throw std::runtime_error("Loop record already exists: " + path.string());
  const auto pending =
      path.parent_path() / (".pending-" + path.filename().string());
  if (fs::exists(pending))
    throw std::runtime_error("Pending loop record already exists: " +
                             pending.string());
  meta::WriteYaml(pending, write);
  fs::rename(pending, path);
}

template <typename Visit>
void StatsFields(LoopSessionStats& s, Visit visit) {
  visit("frame_count", s.frame_count);
  visit("pose_version", s.pose_version);
  visit("submitted", s.submitted);
  visit("completed", s.completed);
  visit("rejected", s.rejected);
  visit("accepted", s.accepted);
  visit("stale", s.stale);
  visit("skipped", s.skipped);
  visit("resource_limited", s.resource_limited);
  visit("peak_work_payloads", s.peak_work_payloads);
  visit("peak_pending", s.peak_pending);
}

const char* DispositionName(LoopDisposition disposition) {
  switch (disposition) {
    case LoopDisposition::kRejected:
      return "rejected";
    case LoopDisposition::kAccepted:
      return "accepted";
    case LoopDisposition::kStale:
      return "stale";
    case LoopDisposition::kResourceLimit:
      return "resource_limit";
    case LoopDisposition::kFailed:
      return "failed";
  }
  throw std::invalid_argument("Unknown loop disposition");
}

std::string ErrorMessage(std::exception_ptr error) {
  try {
    if (error) std::rethrow_exception(error);
  } catch (const std::exception& e) {
    return e.what();
  } catch (...) {
    return "Unknown exception";
  }
  return "Unspecified failure";
}

cv::FileStorage Read(const fs::path& path) {
  auto yaml = meta::ReadYaml(path);
  io::CheckUniqueKeys(yaml.root());
  Require(meta::Integer(yaml["schema_version"]) == 1,
          "Unsupported loop schema");
  return yaml;
}

std::vector<std::uint64_t> Records(const fs::path& directory) {
  std::vector<std::uint64_t> ids;
  for (const auto& entry : fs::directory_iterator(directory)) {
    if (entry.path().filename().string().rfind(".pending-", 0) == 0) continue;
    Require(entry.is_regular_file() && entry.path().extension() == ".yml",
            "Unexpected loop record asset");
    ids.push_back(meta::ParseId(entry.path().stem().string()));
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

void CheckGeometryOptions(const cv::FileNode& node) {
  loop_options::ValidateKeys(
      node, {"enabled", "voxel_size_m", "min_points_per_voxel", "max_plane_rms",
             "max_plane_ratio", "max_voxels_per_submap",
             "max_points_per_keyframe", "max_resident_submaps"});
  GeometryOptions options;
  Require(
      meta::Integer(node["enabled"]) == 1 &&
          meta::Number(node["voxel_size_m"]) == options.voxel_size &&
          meta::Integer(node["min_points_per_voxel"]) ==
              options.min_points_per_voxel &&
          meta::Number(node["max_plane_rms"]) == options.max_plane_rms &&
          meta::Number(node["max_plane_ratio"]) == options.max_plane_ratio &&
          meta::Integer(node["max_voxels_per_submap"]) ==
              options.max_voxels_per_submap &&
          meta::Integer(node["max_points_per_keyframe"]) ==
              options.max_points_per_keyframe &&
          meta::Integer(node["max_resident_submaps"]) ==
              options.max_resident_submaps,
      "Loop worker geometry differs from the fixed policy");
}

AcceptedLoop CheckPrepared(const cv::FileStorage& yaml, std::uint64_t version,
                           const KeyframeReadView& frames,
                           const OnlineSubmapStore& store,
                           const LoopAdmissionOptions& options,
                           const std::vector<RigidTransform>& raw,
                           const std::vector<double>& timestamps,
                           const std::vector<AcceptedLoop>& history) {
  loop_options::ValidateKeys(
      yaml.root(),
      {"schema_version", "version", "base_version", "task_id", "query_id",
       "dispatch_frame_count", "frame_count", "raw_frames", "base_poses",
       "submaps", "proposal", "diagnostics", "expected"});
  const auto n = meta::Id(yaml["dispatch_frame_count"]);
  const auto m = meta::Id(yaml["frame_count"]);
  Require(version && meta::Id(yaml["version"]) == version &&
              meta::Id(yaml["base_version"]) == version - 1 &&
              meta::Id(yaml["task_id"]) > 0 && n && n <= m &&
              m <= frames.frame_count() && m <= options.max_keyframes &&
              version <= options.max_loops,
          "Invalid prepared version or frontier");
  meta::Sequence(yaml["raw_frames"], n);
  std::size_t index = 0;
  for (const auto& row : yaml["raw_frames"]) {
    loop_options::ValidateKeys(row, {"id", "timestamp", "odom_from_body"});
    const auto pose = meta::ReadTransform(row["odom_from_body"]);
    Require(
        meta::Id(row["id"]) == index &&
            meta::Number(row["timestamp"]) == timestamps[index] &&
            (pose.rotation.array() == raw[index].rotation.array()).all() &&
            (pose.translation.array() == raw[index].translation.array()).all(),
        "Prepared raw metadata differs from source");
    ++index;
  }
  const auto base = io::ReadSnapshot(yaml["base_poses"]);
  Require(
      base.frame_count == n && base.version == version - 1 &&
          io::SameSnapshot(base, store.MaterializePoseSnapshot(version - 1, n)),
      "Prepared base pose differs from committed history");
  const auto groups = yaml["submaps"];
  Require(groups.isSeq() && !groups.empty(), "Missing prepared sealed prefix");
  std::vector<OnlineSubmap> members;
  for (const auto& row : groups) {
    auto group = io::ReadMembership(row);
    Require(group.primary.id == members.size() &&
                group.primary.keyframe_ids.back() < n &&
                io::SameMembership(group, store.Read(group.primary.id)),
            "Prepared sealed membership differs from source");
    members.push_back(std::move(group));
  }
  const auto proposal = yaml["proposal"];
  loop_options::ValidateKeys(
      proposal, {"schema_version", "query_id", "pose_version", "frame_count",
                 "target", "source", "constraint", "update", "verification"});
  const auto loop = io::ReadLoop(proposal);
  const auto& c = loop.constraint;
  Require(
      loop.target.primary.id < loop.source.primary.id &&
          loop.source.primary.id < members.size() &&
          io::SameMembership(loop.target, members[loop.target.primary.id]) &&
          io::SameMembership(loop.source, members[loop.source.primary.id]) &&
          meta::Id(yaml["query_id"]) == loop.source.primary.id &&
          meta::Id(proposal["query_id"]) == loop.source.primary.id &&
          meta::Id(proposal["pose_version"]) == version &&
          meta::Id(proposal["frame_count"]) == n &&
          meta::Integer(proposal["schema_version"]) == 1 &&
          c.from_id == loop.target.primary.anchor_keyframe_id &&
          c.to_id == loop.source.primary.anchor_keyframe_id &&
          c.to_id > c.from_id && c.to_id - c.from_id > 1 &&
          c.rotation_sigma_rad == options.loop_rotation_sigma_rad &&
          c.translation_sigma_m == options.loop_translation_sigma_m,
      "Prepared constraint differs from membership or fixed noise");
  for (const auto& old : history)
    Require(
        old.constraint.from_id != c.from_id || old.constraint.to_id != c.to_id,
        "Repeated loop constraint endpoints");
  const auto verification = proposal["verification"];
  const auto measurement =
      meta::ReadTransform(verification["target_from_source"]);
  Require(meta::String(verification["status"]) == "verified" &&
              meta::Id(verification["pose_version"]) == version - 1 &&
              meta::Id(verification["frame_count"]) == n &&
              meta::Id(verification["optimized_frame_count"]) ==
                  base.optimized_frame_count &&
              (measurement.rotation - c.from_body_from_to_body.rotation)
                      .cwiseAbs()
                      .maxCoeff() <= 1e-9 &&
              (measurement.translation - c.from_body_from_to_body.translation)
                      .cwiseAbs()
                      .maxCoeff() <= 1e-9,
          "Constraint differs from its original verification");
  for (const auto* name : {"target", "source"}) {
    const auto& member =
        std::string(name) == "target" ? loop.target : loop.source;
    const auto provenance = verification[name];
    Require(meta::Id(provenance["submap_id"]) == member.primary.id &&
                meta::Id(provenance["anchor_keyframe_id"]) ==
                    member.primary.anchor_keyframe_id &&
                meta::Id(provenance["membership_revision"]) ==
                    member.membership_revision &&
                meta::Id(provenance["representation_revision"]) > 0,
            "Verification membership provenance changed");
  }
  const auto update = io::ReadUpdate(proposal["update"]);
  Require(
      update.base_version == version - 1 && update.optimized_frame_count == n,
      "Prepared update does not cover the dispatch frontier");
  const auto expected = io::ReadSnapshot(yaml["expected"]);
  const std::vector<RigidTransform> prefix(raw.begin(), raw.begin() + m);
  const auto current = store.MaterializePoseSnapshot(version - 1, m);
  Require(
      expected.frame_count == m &&
          io::SameSnapshot(expected, MakePoseSnapshot(update, current, prefix)),
      "Prepared expected pose does not follow the update and tail policy");
  const auto diagnostics = yaml["diagnostics"];
  Require(meta::String(diagnostics["status"]) == "proposed" &&
              meta::Id(diagnostics["base_version"]) == version - 1 &&
              meta::Id(diagnostics["frame_count"]) == n &&
              meta::Id(diagnostics["query_id"]) == loop.source.primary.id,
          "Prepared admission diagnostics do not match the proposal");
  return loop;
}
}  // namespace

LoopClosureStore::LoopClosureStore(const KeyframeReadView& frames,
                                   const LoopAdmissionOptions& options)
    : directory_(frames.directory() / "loop_closure") {
  ValidateLoopAdmissionOptions(options);
  const auto metadata = SourceMetadata(frames.directory());
  if (!fs::create_directory(directory_))
    throw std::runtime_error("Loop session already exists");
  for (const auto* name : {"prepared", "queries", "events"})
    fs::create_directory(directory_ / name);
  Publish(directory_ / "session.yml", false, [&](auto& yaml) {
    yaml << "schema_version" << 1 << "frontier"
         << "dispatch_prefix"
         << "body_frame"
         << "imu"
         << "raw_parent_frame"
         << "odom"
         << "optimized_parent_frame"
         << "map"
         << "tail_policy"
         << "projected_last_optimized_anchor"
         << "dataset_metadata" << metadata << "options"
         << "{";
    GeometryOptions geometry;
    geometry.enabled = true;
    io::WriteOptions(yaml, options, geometry);
    yaml << "}";
  });
  WriteSummary({}, false);
}

void LoopClosureStore::WritePrepared(const LoopTaskResult& result,
                                     const PoseSnapshot& candidate) {
  Require(!result.error && result.admission && result.admission->proposal &&
              result.admission->status == LoopAdmissionStatus::kProposed,
          "Only a valid proposal can be prepared");
  const auto& task = result.task;
  const auto& input = task.input;
  Require(task.timestamps.size() == input.raw_poses.size() &&
              input.raw_poses.size() == input.poses.frame_count &&
              task.frames.frame_count() == input.poses.frame_count &&
              candidate.version == input.poses.version + 1 &&
              candidate.frame_count >= input.poses.frame_count,
          "Invalid prepared task frontier");
  Publish(
      directory_ / "prepared" / (std::to_string(candidate.version) + ".yml"),
      false, [&](auto& yaml) {
        yaml << "schema_version" << 1 << "version"
             << std::to_string(candidate.version) << "base_version"
             << std::to_string(input.poses.version) << "task_id"
             << std::to_string(task.task_id) << "query_id"
             << std::to_string(input.query_id) << "dispatch_frame_count"
             << std::to_string(input.poses.frame_count) << "frame_count"
             << std::to_string(candidate.frame_count) << "raw_frames"
             << "[";
        for (std::size_t id = 0; id < input.raw_poses.size(); ++id) {
          yaml << "{"
               << "id" << std::to_string(id) << "timestamp"
               << meta::Decimal(task.timestamps[id]);
          meta::WriteTransform(yaml, "odom_from_body", input.raw_poses[id]);
          yaml << "}";
        }
        yaml << "]";
        io::WriteSnapshot(yaml, "base_poses", input.poses);
        yaml << "submaps"
             << "[";
        for (const auto& info : input.submaps) {
          io::WriteMembership(yaml, nullptr, info.membership);
        }
        yaml << "]"
             << "proposal"
             << "{";
        io::WriteProposal(yaml, input, *result.admission);
        yaml << "}"
             << "diagnostics"
             << "{";
        const auto source = std::find_if(
            input.submaps.begin(), input.submaps.end(), [&](const auto& info) {
              return info.membership.primary.id == input.query_id;
            });
        Require(source != input.submaps.end(), "Missing query metadata");
        io::WriteQuery(yaml, *source, input, *result.admission, 0);
        yaml << "}";
        io::WriteSnapshot(yaml, "expected", candidate);
      });
}

void LoopClosureStore::WriteQuery(const LoopTaskResult& result,
                                  LoopDisposition disposition,
                                  std::uint64_t current_version,
                                  std::uint64_t frame_count) {
  Require(bool(result.error) != bool(result.admission),
          "Invalid loop result variant");
  Publish(
      directory_ / "queries" / (std::to_string(result.task.task_id) + ".yml"),
      false, [&](auto& yaml) {
        yaml << "schema_version" << 1 << "task_id"
             << std::to_string(result.task.task_id) << "query_id"
             << std::to_string(result.task.input.query_id) << "base_version"
             << std::to_string(result.task.input.poses.version)
             << "dispatch_frame_count"
             << std::to_string(result.task.input.poses.frame_count)
             << "current_version" << std::to_string(current_version)
             << "frame_count" << std::to_string(frame_count) << "disposition"
             << DispositionName(disposition);
        if (result.error) {
          yaml << "error" << ErrorMessage(result.error);
        } else {
          yaml << "diagnostics"
               << "{";
          const auto& input = result.task.input;
          const auto source = std::find_if(
              input.submaps.begin(), input.submaps.end(),
              [&](const auto& info) {
                return info.membership.primary.id == input.query_id;
              });
          Require(source != input.submaps.end(), "Missing query metadata");
          io::WriteQuery(yaml, *source, input, *result.admission, 0);
          yaml << "}";
        }
      });
}

void LoopClosureStore::WriteCoalesced(std::uint64_t first, std::uint64_t last,
                                      std::uint64_t count) {
  Require(first <= last && count && count <= last - first + 1,
          "Invalid coalesced query range");
  Publish(directory_ / "events" / (std::to_string(next_event_) + ".yml"), false,
          [&](auto& yaml) {
            yaml << "schema_version" << 1 << "sequence"
                 << std::to_string(next_event_) << "kind"
                 << "coalesced"
                 << "first_query_id" << std::to_string(first) << "last_query_id"
                 << std::to_string(last) << "count" << std::to_string(count);
          });
  ++next_event_;
}

void LoopClosureStore::WriteResourceLimit(std::uint64_t query_id,
                                          std::uint64_t frame_count) {
  Publish(directory_ / "events" / (std::to_string(next_event_) + ".yml"), false,
          [&](auto& yaml) {
            yaml << "schema_version" << 1 << "sequence"
                 << std::to_string(next_event_) << "kind"
                 << "resource_limit"
                 << "query_id" << std::to_string(query_id) << "frame_count"
                 << std::to_string(frame_count);
          });
  ++next_event_;
}

void LoopClosureStore::WriteSummary(const LoopSessionStats& stats,
                                    bool complete) {
  Publish(directory_ / "summary.yml", true, [&](auto& yaml) {
    yaml << "schema_version" << 1 << "complete" << static_cast<int>(complete);
    auto copy = stats;
    StatsFields(copy, [&](const char* key, auto value) {
      yaml << key << std::to_string(value);
    });
  });
}

void LoopClosureStore::Finish(const LoopSessionStats& stats) {
  WriteSummary(stats, true);
}

void LoopClosureStore::WriteFailure(std::exception_ptr error) noexcept {
  try {
    Publish(directory_ / "failure.yml", false, [&](auto& yaml) {
      yaml << "schema_version" << 1 << "error" << ErrorMessage(error);
    });
  } catch (...) {
    // Diagnostics cannot replace the exception that stopped the session.
  }
}

LoopRecoveryReport InspectLoopDataset(const fs::path& dataset) {
  LoopRecoveryReport report;
  try {
    const auto directory = dataset / "loop_closure";
    for (auto iterator = fs::recursive_directory_iterator(dataset);
         iterator != fs::recursive_directory_iterator(); ++iterator) {
      const auto& entry = *iterator;
      if (entry.path().filename().string().rfind(".pending-", 0) == 0) {
        report.issues.push_back("Pending asset: " +
                                fs::relative(entry.path(), dataset).string());
        if (entry.is_directory()) iterator.disable_recursion_pending();
      } else if (entry.is_regular_file() &&
                 entry.path().extension() == ".yml") {
        io::CheckUniqueKeys(meta::ReadYaml(entry.path()).root());
      }
    }
    const auto session = Read(directory / "session.yml");
    loop_options::ValidateKeys(session.root(),
                               {"schema_version", "frontier", "body_frame",
                                "raw_parent_frame", "optimized_parent_frame",
                                "tail_policy", "dataset_metadata", "options"});
    Require(meta::String(session["frontier"]) == "dispatch_prefix" &&
                meta::String(session["body_frame"]) == "imu" &&
                meta::String(session["raw_parent_frame"]) == "odom" &&
                meta::String(session["optimized_parent_frame"]) == "map" &&
                meta::String(session["tail_policy"]) ==
                    "projected_last_optimized_anchor" &&
                meta::String(session["dataset_metadata"]) ==
                    SourceMetadata(dataset),
            "Loop session coordinate or calibration source changed");
    const auto options = io::ReadOptions(session["options"], true);
    CheckGeometryOptions(session["options"]["geometry"]);
    KeyframeStore frames(dataset);
    const auto view = frames.CaptureReadView();
    OnlineSubmapStore store(dataset / "online_submaps", frames);
    std::vector<RigidTransform> raw;
    std::vector<double> timestamps;
    for (auto id : frames.ids()) {
      raw.push_back(view.ReadRawBodyPose(id));
      timestamps.push_back(view.ReadTimestamp(id));
      Require(timestamps.size() < 2 ||
                  timestamps.back() >= timestamps[timestamps.size() - 2],
              "Source timestamps moved backwards");
    }
    const auto prepared = Records(directory / "prepared");
    const auto versions = Records(dataset / "online_submaps/poses");
    std::map<std::uint64_t, std::uint64_t> accepted_tasks;
    std::vector<AcceptedLoop> history;
    for (auto version : prepared) {
      Require(version == history.size() + 1,
              "Prepared versions must be consecutive");
      const auto yaml =
          Read(directory / "prepared" / (std::to_string(version) + ".yml"));
      auto loop = CheckPrepared(yaml, version, view, store, options, raw,
                                timestamps, history);
      if (version <= versions.size()) {
        Require(versions[version - 1] == version &&
                    io::SameSnapshot(io::ReadSnapshot(yaml["expected"]),
                                     store.ReadPoseSnapshot(version)),
                "Published pose differs from prepared record");
        Require(
            accepted_tasks.emplace(meta::Id(yaml["task_id"]), version).second,
            "One task cannot commit multiple versions");
        history.push_back(std::move(loop));
        report.committed_version = version;
      } else {
        Require(version == versions.size() + 1 && version == prepared.back(),
                "Only the next version may be an uncommitted prepared record");
        report.uncommitted_prepared.push_back(version);
      }
    }
    Require(report.committed_version == versions.size(),
            "Published pose has no matching prepared record");
    const auto queries = Records(directory / "queries");
    LoopSessionStats observed;
    bool failed_query = false;
    for (auto id : queries) {
      const auto query =
          Read(directory / "queries" / (std::to_string(id) + ".yml"));
      loop_options::ValidateKeys(
          query.root(),
          {"schema_version", "task_id", "query_id", "base_version",
           "dispatch_frame_count", "current_version", "frame_count",
           "disposition", "error", "diagnostics"});
      Require(
          id == observed.completed + 1 && meta::Id(query["task_id"]) == id &&
              meta::Id(query["dispatch_frame_count"]) <=
                  meta::Id(query["frame_count"]) &&
              meta::Id(query["frame_count"]) <= view.frame_count() &&
              meta::Id(query["current_version"]) <= report.committed_version,
          "Invalid query identity or frontier");
      const auto disposition = meta::String(query["disposition"]);
      if (disposition == "accepted") {
        const auto match = accepted_tasks.find(id);
        Require(match != accepted_tasks.end() &&
                    meta::Id(query["current_version"]) == match->second &&
                    meta::Id(query["base_version"]) + 1 == match->second &&
                    meta::String(query["diagnostics"]["status"]) == "proposed",
                "Accepted query lacks its committed version");
        ++observed.accepted;
      } else if (disposition == "rejected")
        ++observed.rejected;
      else if (disposition == "stale")
        ++observed.stale;
      else if (disposition == "resource_limit")
        ++observed.resource_limited;
      else {
        Require(disposition == "failed", "Unknown query disposition");
        failed_query = true;
      }
      ++observed.completed;
    }
    const auto resource_queries = observed.resource_limited;
    const auto events = Records(directory / "events");
    std::uint64_t sequence = 0;
    for (auto id : events) {
      const auto event =
          Read(directory / "events" / (std::to_string(id) + ".yml"));
      Require(id == ++sequence && meta::Id(event["sequence"]) == id,
              "Event sequence must be consecutive");
      const auto kind = meta::String(event["kind"]);
      if (kind == "coalesced") {
        loop_options::ValidateKeys(
            event.root(), {"schema_version", "sequence", "kind",
                           "first_query_id", "last_query_id", "count"});
        const auto first = meta::Id(event["first_query_id"]),
                   last = meta::Id(event["last_query_id"]),
                   count = meta::Id(event["count"]);
        Require(first <= last && count && count <= last - first + 1,
                "Invalid coalesced range");
        observed.skipped += count;
      } else {
        Require(kind == "resource_limit", "Unknown loop event kind");
        loop_options::ValidateKeys(
            event.root(),
            {"schema_version", "sequence", "kind", "query_id", "frame_count"});
        Require(meta::Id(event["frame_count"]) <= view.frame_count(),
                "Invalid resource event frontier");
        ++observed.resource_limited;
      }
    }
    const auto summary = Read(directory / "summary.yml");
    auto keys = std::set<std::string>{"schema_version", "complete"};
    LoopSessionStats saved;
    StatsFields(saved, [&](const char* key, auto& value) {
      keys.insert(key);
      value = meta::Id(summary[key]);
    });
    loop_options::ValidateKeys(summary.root(), keys);
    const auto complete = meta::Integer(summary["complete"]);
    Require(complete == 0 || complete == 1, "Invalid completion flag");
    const bool sealed =
        store.size() == 0 ||
        store.Read(store.size() - 1).state == OnlineSubmapState::kSealed;
    const bool indexed = store.indexed_frame_count() == view.frame_count();
    if (!sealed || !indexed)
      report.issues.push_back("Input tail is not fully sealed and indexed");
    if (!complete) report.issues.push_back("Session did not complete normally");
    if (!report.uncommitted_prepared.empty())
      report.issues.push_back("Prepared update was not committed");
    if (fs::exists(directory / "failure.yml"))
      report.issues.push_back("Session has a failure report");
    if (complete) {
      Require(report.issues.empty() &&
                  saved.frame_count == view.frame_count() &&
                  saved.pose_version == report.committed_version &&
                  saved.accepted == report.committed_version &&
                  saved.submitted == observed.completed &&
                  saved.completed == observed.completed &&
                  saved.accepted == observed.accepted &&
                  saved.rejected == observed.rejected &&
                  saved.stale == observed.stale &&
                  saved.skipped == observed.skipped &&
                  saved.resource_limited == observed.resource_limited &&
                  saved.peak_work_payloads <= 1 && saved.peak_pending <= 1 &&
                  !failed_query &&
                  saved.completed == saved.accepted + saved.rejected +
                                         saved.stale + resource_queries,
              "Completion summary does not match committed records");
      report.status = LoopRecoveryStatus::kConsistent;
    }
  } catch (const std::exception& error) {
    report.status = LoopRecoveryStatus::kInconsistent;
    report.issues.push_back(error.what());
  } catch (...) {
    report.status = LoopRecoveryStatus::kInconsistent;
    report.issues.push_back("Unknown inspection failure");
  }
  return report;
}

}  // namespace livo_atlas
