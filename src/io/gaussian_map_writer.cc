#include "livo_atlas/io/gaussian_map_writer.h"

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <opencv2/core.hpp>
#include <set>
#include <string>
#include <system_error>
#include <vector>

#include "gaussian/submap_protocol.h"
#include "mapping/pose_math.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;

void Require(bool condition, const char* message) {
  if (!condition) throw std::invalid_argument(message);
}

const char* Availability(GaussianAvailability value) {
  switch (value) {
    case GaussianAvailability::kMissing:
      return "missing";
    case GaussianAvailability::kCurrent:
      return "current";
    case GaussianAvailability::kNeedsRebuild:
      return "needs_rebuild";
    case GaussianAvailability::kFailed:
      return "failed";
  }
  throw std::invalid_argument("Unknown Gaussian availability");
}

bool Covered(const GaussianTarget& source, const GaussianTarget& target) {
  const auto& ids = source.group.primary.keyframe_ids;
  const auto& current = target.group.primary.keyframe_ids;
  return source.session_id == target.session_id &&
         source.group.primary.id == target.group.primary.id &&
         ids.size() <= current.size() &&
         std::equal(ids.begin(), ids.end(), current.begin()) &&
         source.frame_count <= target.frame_count &&
         source.pose_version <= target.pose_version;
}

bool Unchanged(const GaussianTarget& source, const GaussianTarget& target) {
  for (std::size_t i = 0; i < source.submap_from_body.size(); ++i)
    if (!pose_math::Equal(source.submap_from_body[i],
                          target.submap_from_body[i]))
      return false;
  return true;
}

void Coverage(const GaussianTarget& source,
              const std::vector<std::uint64_t>& ids, int count) {
  Require(!ids.empty() && ids.size() <= 10 &&
              count >= static_cast<int>(ids.size()) &&
              count <= static_cast<int>(ids.size()) * 1000 &&
              std::is_sorted(ids.begin(), ids.end()) &&
              std::adjacent_find(ids.begin(), ids.end()) == ids.end() &&
              ids.front() >= source.group.primary.anchor_keyframe_id &&
              ids.back() <= source.group.primary.keyframe_ids.back(),
          "Invalid Gaussian coverage");
}

void Validate(const GaussianMapSnapshot& snapshot) {
  const auto& session = snapshot.session_id;
  Require(
      session.size() == 32 &&
          std::all_of(session.begin(), session.end(),
                      [](char c) {
                        return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
                      }) &&
          snapshot.map_sequence > 0 && snapshot.frame_count <= 512 &&
          snapshot.entries.size() <= 512 &&
          snapshot.optimized_frame_count <= snapshot.frame_count &&
          (snapshot.pose_version ? snapshot.parent_frame == PoseFrame::kMap &&
                                       snapshot.optimized_frame_count > 0
                                 : snapshot.parent_frame == PoseFrame::kOdom &&
                                       snapshot.optimized_frame_count == 0),
      "Invalid Gaussian map header");
  std::set<std::uint64_t> assets;
  std::size_t contents = 0;
  std::size_t residents = 0;
  std::uint64_t next_primary = 0;
  for (std::size_t i = 0; i < snapshot.entries.size(); ++i) {
    const auto& entry = snapshot.entries[i];
    ValidateGaussianTarget(entry.binding);
    const auto& group = entry.group;
    const auto& bound = entry.binding.group;
    Require(group.primary.id == i &&
                group.primary.anchor_keyframe_id == next_primary &&
                group.primary.keyframe_ids == bound.primary.keyframe_ids &&
                group.primary.id == bound.primary.id &&
                group.membership_revision == bound.membership_revision &&
                group.state == bound.state &&
                group.support_keyframe_ids == bound.support_keyframe_ids &&
                entry.binding.session_id == session &&
                entry.binding.frame_count == snapshot.frame_count &&
                entry.binding.pose_version == snapshot.pose_version &&
                entry.binding.optimized_frame_count ==
                    snapshot.optimized_frame_count &&
                entry.binding.parent_frame == snapshot.parent_frame &&
                entry.reason.size() <= 256 &&
                entry.reason.find('\0') == std::string::npos,
            "Invalid Gaussian logical group or binding");
    next_primary = group.primary.keyframe_ids.back() + 1;
    (void)Availability(entry.availability);
    residents += entry.resident ? 1 : 0;
    if (!entry.content) {
      Require(entry.availability == GaussianAvailability::kMissing &&
                  !entry.asset && !entry.resident && !entry.dirty &&
                  entry.frame_ids.empty() && entry.gaussian_count == 0,
              "Missing group claims Gaussian content");
      continue;
    }
    ++contents;
    const auto& content = *entry.content;
    ValidateGaussianTarget(entry.content_target);
    Require(content.session_id == session && content.submap_id == i &&
                content.content_job_id > 0 &&
                content.source_generation == entry.content_target.generation &&
                Covered(entry.content_target, entry.binding),
            "Invalid confirmed content identity");
    Coverage(entry.content_target, entry.frame_ids, entry.gaussian_count);
    if (entry.availability != GaussianAvailability::kFailed)
      Require(
          entry.availability == (Unchanged(entry.content_target, entry.binding)
                                     ? GaussianAvailability::kCurrent
                                     : GaussianAvailability::kNeedsRebuild),
          "Gaussian currentness differs from complete source primary");
    Require(entry.dirty == (!entry.asset || entry.asset->content != content),
            "Volatile content is confused with its durable asset");
    if (entry.asset) {
      const auto& asset = *entry.asset;
      ValidateGaussianAssetReference(asset);
      Require(assets.insert(asset.asset_id).second,
              "Gaussian map repeats an asset identity");
      ValidateGaussianTarget(entry.asset_source);
      Require(
          asset.content.session_id == session && asset.content.submap_id == i &&
              asset.content.source_generation ==
                  entry.asset_source.generation &&
              Covered(entry.asset_source, entry.binding) &&
              entry.asset_current ==
                  Unchanged(entry.asset_source, entry.binding),
          "Durable asset identity or currentness differs from source primary");
      Coverage(entry.asset_source, entry.asset_frame_ids,
               entry.asset_gaussian_count);
    } else {
      Require(!entry.asset_current, "A missing asset cannot be current");
    }
  }
  Require(
      next_primary == snapshot.frame_count && contents <= 32 && residents <= 1,
      "Gaussian map omits logical coverage or exceeds its budget");
}

void Id(cv::FileStorage& json, const char* name, std::uint64_t value) {
  json << name << std::to_string(value);
}

void Content(cv::FileStorage& json, const GaussianContentId& content) {
  json << "session_id" << content.session_id;
  Id(json, "submap_id", content.submap_id);
  Id(json, "source_generation", content.source_generation);
  Id(json, "content_job_id", content.content_job_id);
}

void Ids(cv::FileStorage& json, const char* name,
         const std::vector<std::uint64_t>& ids) {
  json << name << "[";
  for (const auto id : ids) json << std::to_string(id);
  json << "]";
}

void Target(cv::FileStorage& json, const char* name,
            const GaussianTarget& target, bool primary) {
  json << name << "{";
  WriteGaussianTargetHeader(json, target);
  if (primary) {
    json << "primary"
         << "[";
    for (std::size_t i = 0; i < target.submap_from_body.size(); ++i) {
      json << "{";
      Id(json, "keyframe_id", target.group.primary.keyframe_ids[i]);
      json << "submap_from_body"
           << "[";
      const auto& pose = target.submap_from_body[i];
      for (int row = 0; row < 4; ++row)
        for (int column = 0; column < 4; ++column)
          json << (row == 3 ? (column == 3 ? 1.0 : 0.0)
                            : column == 3 ? pose.translation[row]
                                          : pose.rotation(row, column));
      json << "]"
           << "}";
    }
    json << "]";
  }
  json << "}";
}

// Keep the full bounded metadata useful within its byte budget. Strip only
// indentation outside strings; diagnostics and escaped quotes are intact.
// Retain line breaks for OpenCV's bounded-line JSON reader.
std::string CompactJson(std::string value) {
  bool quoted = false;
  bool escaped = false;
  std::size_t size = 0;
  for (const char c : value) {
    if (quoted) {
      value[size++] = c;
      if (escaped)
        escaped = false;
      else if (c == '\\')
        escaped = true;
      else if (c == '\"')
        quoted = false;
    } else if (c == '\"') {
      quoted = true;
      value[size++] = c;
    } else if (c != ' ' && c != '\r' && c != '\t') {
      value[size++] = c;
    }
  }
  value.resize(size);
  return value;
}

std::string Serialize(const GaussianMapSnapshot& snapshot) {
  Validate(snapshot);
  cv::FileStorage json("", cv::FileStorage::WRITE | cv::FileStorage::MEMORY |
                               cv::FileStorage::FORMAT_JSON);
  json << "schema" << 1 << "kind"
       << "livo_atlas.gaussian_map"
       << "session_id" << snapshot.session_id;
  Id(json, "map_sequence", snapshot.map_sequence);
  Id(json, "pose_version", snapshot.pose_version);
  Id(json, "frame_count", snapshot.frame_count);
  Id(json, "optimized_frame_count", snapshot.optimized_frame_count);
  const auto now = std::chrono::duration<double>(
                       std::chrono::system_clock::now().time_since_epoch())
                       .count();
  json << "parent_frame"
       << (snapshot.parent_frame == PoseFrame::kMap ? "map" : "odom")
       << "written_at_unix_seconds" << now << "entries"
       << "[";
  for (const auto& entry : snapshot.entries) {
    json << "{";
    Id(json, "submap_id", entry.group.primary.id);
    Id(json, "anchor_keyframe_id", entry.group.primary.anchor_keyframe_id);
    Id(json, "membership_revision", entry.group.membership_revision);
    json << "primary_count"
         << static_cast<int>(entry.group.primary.keyframe_ids.size())
         << "submap_state"
         << (entry.group.state == OnlineSubmapState::kSealed ? "sealed"
                                                             : "active")
         << "availability" << Availability(entry.availability) << "reason"
         << entry.reason << "resident" << static_cast<int>(entry.resident);
    Target(json, "binding", entry.binding, false);
    Id(json, "binding_job_id", entry.binding_job_id);
    if (entry.content) {
      json << "content"
           << "{";
      Content(json, *entry.content);
      json << "}"
           << "volatile" << static_cast<int>(entry.dirty);
      Target(json, "content_source", entry.content_target, true);
      Ids(json, "trained_frame_ids", entry.frame_ids);
      json << "gaussian_count" << entry.gaussian_count;
    }
    if (entry.asset) {
      const auto& asset = *entry.asset;
      json << "asset"
           << "{";
      Id(json, "asset_id", asset.asset_id);
      Content(json, asset.content);
      json << "manifest_sha256" << asset.manifest_sha256;
      Id(json, "manifest_bytes", asset.manifest_bytes);
      Id(json, "gaussian_bytes", asset.gaussian_bytes);
      Id(json, "observation_bytes", asset.observation_bytes);
      json << "}";
      Target(json, "asset_source", entry.asset_source, true);
      Ids(json, "asset_trained_frame_ids", entry.asset_frame_ids);
      json << "asset_gaussian_count" << entry.asset_gaussian_count
           << "asset_current" << static_cast<int>(entry.asset_current);
    }
    json << "}";
  }
  json << "]";
  auto result = CompactJson(json.releaseAndGetString());
  Require(!result.empty() && result.size() <= 512 * 1024,
          "Gaussian map JSON exceeds its budget");
  return result;
}

void SystemError(const char* operation) {
  throw std::system_error(errno, std::generic_category(), operation);
}

void SyncDirectory(const fs::path& path) {
  const int fd =
      open(path.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
  if (fd < 0) SystemError("Open Gaussian map directory");
  const int synced = fsync(fd);
  const int error = errno;
  const int closed = close(fd);
  if (synced < 0) {
    errno = error;
    SystemError("Synchronize Gaussian map directory");
  }
  if (closed < 0) SystemError("Close Gaussian map directory");
}

const char* FinalizationStatus(GaussianFinalizationStatus status) {
  switch (status) {
    case GaussianFinalizationStatus::kSucceeded:
      return "succeeded";
    case GaussianFinalizationStatus::kTimedOut:
      return "timed_out";
    case GaussianFinalizationStatus::kFailed:
      return "failed";
  }
  throw std::invalid_argument("Invalid Gaussian finalization status");
}

void Counters(cv::FileStorage& json, const char* name,
              const GaussianFinalizationCounters& value) {
  json << name << "{";
  Id(json, "accepted", value.accepted);
  Id(json, "effective_training", value.effective_training);
  Id(json, "rebuilds", value.rebuilds);
  Id(json, "checkpoints", value.checkpoints);
  json << "}";
}

std::string SerializeFinalization(const GaussianFinalizationResult& r) {
  const char* status = FinalizationStatus(r.status);
  const auto ordered = [](const auto& ids) {
    return ids.size() <= 32 && std::is_sorted(ids.begin(), ids.end()) &&
           std::adjacent_find(ids.begin(), ids.end()) == ids.end();
  };
  Require(
      r.session_id.size() == 32 &&
          std::all_of(r.session_id.begin(), r.session_id.end(),
                      [](char c) {
                        return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
                      }) &&
          r.reason.size() <= 1024 && r.reason.find('\0') == std::string::npos &&
          r.frame_count <= 512 && r.optimized_frame_count <= r.frame_count &&
          ((r.pose_version == 0 && r.parent_frame == PoseFrame::kOdom &&
            r.optimized_frame_count == 0) ||
           (r.pose_version > 0 && r.parent_frame == PoseFrame::kMap &&
            r.optimized_frame_count > 0)) &&
          ordered(r.eligible_submap_ids) && ordered(r.asset_ids) &&
          (r.eligible_submap_ids.empty() ||
           r.eligible_submap_ids.back() < r.frame_count) &&
          (r.asset_ids.empty() || r.asset_ids.front() > 0) && r.current <= 32 &&
          r.needs_rebuild <= 32 && r.dirty <= 32 && r.missing <= 512,
      "Invalid Gaussian finalization metadata");
  for (const double seconds :
       {r.elapsed_seconds, r.repair_seconds, r.shutdown_seconds})
    Require(std::isfinite(seconds) && seconds >= 0,
            "Invalid finalization duration");
  Require(r.after.accepted >= r.before.accepted &&
              r.after.effective_training >= r.before.effective_training &&
              r.after.rebuilds >= r.before.rebuilds &&
              r.after.checkpoints >= r.before.checkpoints,
          "Finalization counters went backwards");
  for (const auto& counts : {r.before, r.after})
    Require(counts.effective_training <= counts.accepted &&
                counts.rebuilds <= counts.accepted,
            "Invalid finalization training counters");
  if (r.eligible_set_frozen)
    Require(r.current + r.needs_rebuild <= r.eligible_submap_ids.size(),
            "Finalization classification exceeds frozen content");
  if (r.status == GaussianFinalizationStatus::kSucceeded)
    Require(
        r.target_valid && r.eligible_set_frozen && r.durable &&
            r.worker_stopped && r.map_sequence > 0 && r.dirty == 0 &&
            r.needs_rebuild == 0 && r.current == r.eligible_submap_ids.size() &&
            r.asset_ids.size() == r.current && r.after.accepted >= r.current,
        "Unfinished Gaussian finalization claims success");
  cv::FileStorage json(".json", cv::FileStorage::WRITE |
                                    cv::FileStorage::MEMORY |
                                    cv::FileStorage::FORMAT_JSON);
  json << "schema" << 1 << "kind"
       << "livo_atlas.gaussian_finalization"
       << "session_id" << r.session_id << "status" << status << "reason"
       << r.reason;
  Id(json, "pose_version", r.pose_version);
  Id(json, "frame_count", r.frame_count);
  Id(json, "optimized_frame_count", r.optimized_frame_count);
  json << "parent_frame"
       << (r.parent_frame == PoseFrame::kMap ? "map" : "odom");
  Id(json, "map_sequence", r.map_sequence);
  Ids(json, "eligible_submap_ids", r.eligible_submap_ids);
  Ids(json, "asset_ids", r.asset_ids);
  json << "eligible" << static_cast<int>(r.eligible_submap_ids.size());
  json << "target_valid" << static_cast<int>(r.target_valid)
       << "eligible_set_frozen" << static_cast<int>(r.eligible_set_frozen)
       << "durable" << static_cast<int>(r.durable) << "worker_stopped"
       << static_cast<int>(r.worker_stopped);
  json << "current" << static_cast<int>(r.current) << "needs_rebuild"
       << static_cast<int>(r.needs_rebuild) << "missing"
       << static_cast<int>(r.missing) << "dirty" << static_cast<int>(r.dirty)
       << "elapsed_seconds" << r.elapsed_seconds << "repair_seconds"
       << r.repair_seconds << "shutdown_seconds" << r.shutdown_seconds;
  Id(json, "repair_attempts", r.repair_attempts);
  Counters(json, "before", r.before);
  Counters(json, "after", r.after);
  const auto data = CompactJson(json.releaseAndGetString());
  Require(data.size() <= 16 * 1024,
          "Gaussian finalization report exceeds 16 KiB");
  return data;
}

void Publish(const fs::path& output, const std::string& stem,
             const std::string& data) {
  Require(fs::is_directory(fs::symlink_status(output)),
          "Gaussian output must be an existing real directory");
  const auto temporary = output / (stem + ".tmp.json");
  const auto destination = output / (stem + ".json");
  int fd = open(temporary.c_str(),
                O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600);
  if (fd < 0) SystemError("Create Gaussian map temporary file");
  bool published = false;
  try {
    std::size_t offset = 0;
    while (offset < data.size()) {
      const auto written =
          write(fd, data.data() + offset, data.size() - offset);
      if (written < 0 && errno == EINTR) continue;
      if (written <= 0) {
        if (written == 0) errno = EIO;
        SystemError("Write Gaussian map");
      }
      offset += static_cast<std::size_t>(written);
    }
    if (fsync(fd) < 0) SystemError("Synchronize Gaussian map file");
    const int closing = fd;
    fd = -1;
    if (close(closing) < 0) SystemError("Close Gaussian map file");
    if (rename(temporary.c_str(), destination.c_str()) < 0)
      SystemError("Publish Gaussian map");
    published = true;
    SyncDirectory(output);
    // The mapper created output itself, so its directory entry also needs a
    // durable parent before a worker can release or prune any source asset.
    SyncDirectory(fs::absolute(output).parent_path());
  } catch (const std::exception& error) {
    if (fd >= 0) close(fd);
    if (published)
      throw GaussianMapCommitUncertainError(
          "Gaussian " + stem +
          " is visible; durability uncertain: " + error.what());
    unlink(temporary.c_str());
    throw;
  }
}

}  // namespace

void WriteGaussianMapIndex(const fs::path& output,
                           const GaussianMapSnapshot& snapshot) {
  Publish(output, "map", Serialize(snapshot));
}

void WriteGaussianFinalizationResult(const fs::path& output,
                                     const GaussianFinalizationResult& result) {
  Publish(output, "finalization", SerializeFinalization(result));
}

}  // namespace livo_atlas
