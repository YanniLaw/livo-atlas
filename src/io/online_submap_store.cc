#include "livo_atlas/io/online_submap_store.h"

#include <algorithm>
#include <limits>
#include <stdexcept>

#include "dataset_metadata.h"
namespace livo_atlas {
namespace {
namespace fs = std::filesystem;
namespace meta = dataset_metadata;
std::vector<std::uint64_t> Entries(const fs::path& path, bool directories) {
  std::vector<std::uint64_t> ids;
  for (const auto& entry : fs::directory_iterator(path)) {
    const auto name = entry.path().filename().string();
    if (name.rfind(".pending-", 0) == 0) continue;
    if (directories
            ? !entry.is_directory()
            : !entry.is_regular_file() || entry.path().extension() != ".yml")
      meta::Fail(entry.path(), "Unexpected online index asset");
    ids.push_back(
        meta::ParseId(directories ? name : entry.path().stem().string()));
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

void Schema(const cv::FileStorage& yaml) {
  if (!yaml.isOpened() || meta::Integer(yaml["schema_version"]) != 1)
    throw std::invalid_argument("Unsupported online submap schema");
}

void WriteIds(cv::FileStorage& yaml, const char* name,
              const std::vector<std::uint64_t>& ids) {
  yaml << name << "[";
  for (auto id : ids) yaml << std::to_string(id);
  yaml << "]";
}

std::vector<std::uint64_t> ReadIds(const cv::FileNode& node,
                                   std::size_t limit) {
  if (!node.isSeq() || node.size() > limit)
    throw std::invalid_argument("Invalid online membership shape");
  std::vector<std::uint64_t> ids;
  for (const auto& item : node) ids.push_back(meta::Id(item));
  return ids;
}
}  // namespace
OnlineSubmapStore::OnlineSubmapStore(const fs::path& directory,
                                     const KeyframeStore& keyframes,
                                     int max_keyframes,
                                     OnlineSubmapOptions options)
    : directory_(directory),
      keyframes_(keyframes),
      max_keyframes_(max_keyframes),
      options_(options),
      writable_(true) {
  meta::ValidateInput(directory_, [&] {
    ValidateOnlineSubmapOptions(max_keyframes_, options_);
    if (directory_.empty())
      throw std::invalid_argument("Empty online index path");
  });
  try {
    if (!directory_.parent_path().empty())
      fs::create_directories(directory_.parent_path());
    if (!fs::create_directory(directory_))
      meta::Fail(directory_, "Online index already exists");
    fs::create_directory(directory_ / "records");
    fs::create_directory(directory_ / "poses");
    meta::WriteYaml(directory_ / "dataset.yml", [&](auto& yaml) {
      yaml << "schema_version" << 1 << "mode"
           << "online"
           << "max_keyframes" << max_keyframes_ << "translation_threshold"
           << meta::Decimal(options_.translation_threshold)
           << "rotation_threshold_deg"
           << meta::Decimal(options_.rotation_threshold_deg) << "time_threshold"
           << meta::Decimal(options_.time_threshold) << "support_keyframes"
           << options_.support_keyframes << "body_frame"
           << "imu"
           << "anchor_policy"
           << "first_primary"
           << "raw_parent_frame"
           << "odom"
           << "optimized_parent_frame"
           << "map";
    });
  } catch (const std::exception& error) {
    meta::Fail(directory_, error.what());
  }
}

OnlineSubmapStore::OnlineSubmapStore(const fs::path& directory,
                                     const KeyframeStore& keyframes)
    : directory_(directory), keyframes_(keyframes) {
  try {
    auto yaml = meta::ReadYaml(directory_ / "dataset.yml");
    Schema(yaml);
    if (meta::String(yaml["mode"]) != "online" ||
        meta::String(yaml["body_frame"]) != "imu" ||
        meta::String(yaml["anchor_policy"]) != "first_primary" ||
        meta::String(yaml["raw_parent_frame"]) != "odom" ||
        meta::String(yaml["optimized_parent_frame"]) != "map")
      meta::Fail(directory_, "Invalid online coordinate convention");
    max_keyframes_ = meta::Integer(yaml["max_keyframes"]);
    options_.translation_threshold =
        meta::Number(yaml["translation_threshold"]);
    options_.rotation_threshold_deg =
        meta::Number(yaml["rotation_threshold_deg"]);
    options_.time_threshold = meta::Number(yaml["time_threshold"]);
    options_.support_keyframes = meta::Integer(yaml["support_keyframes"]);
    ValidateOnlineSubmapOptions(max_keyframes_, options_);
    for (auto id : Entries(directory_ / "records", true)) {
      auto revisions =
          Entries(directory_ / "records" / std::to_string(id), false);
      for (auto revision : revisions) {
        auto record = ReadRecord(id, revision);
        ValidateNext(record);
        Accept(record);
      }
    }
    for (auto version : Entries(directory_ / "poses", false)) {
      if (pose_version_ == std::numeric_limits<std::uint64_t>::max() ||
          version != pose_version_ + 1)
        meta::Fail(directory_, "Pose versions must be consecutive");
      const auto snapshot = ReadPoseFile(version);
      if (snapshot.optimized_frame_count < optimized_count_)
        meta::Fail(directory_, "Optimized prefix cannot shrink");
      pose_version_ = version;
      optimized_count_ = snapshot.optimized_frame_count;
    }
  } catch (const std::exception& error) {
    meta::Fail(directory_, error.what());
  }
}

std::uint64_t OnlineSubmapStore::indexed_frame_count() const {
  return submaps_.empty() ? 0 : submaps_.back().primary.keyframe_ids.back() + 1;
}

void OnlineSubmapStore::CheckWritable() const {
  if (failure_) std::rethrow_exception(failure_);
  if (!writable_)
    throw std::logic_error(directory_.string() + ": Online index is read-only");
}

void OnlineSubmapStore::SaveFailure(const fs::path& path,
                                    const std::exception& error) {
  failure_ = std::make_exception_ptr(
      std::runtime_error(path.string() + ": " + error.what()));
  std::rethrow_exception(failure_);
}

void OnlineSubmapStore::ValidateNext(const OnlineSubmap& submap) const {
  ValidateSubmap(submap.primary);
  const auto& primary = submap.primary;
  if (primary.id > submaps_.size() ||
      primary.keyframe_ids.size() > static_cast<std::size_t>(max_keyframes_) ||
      primary.keyframe_ids.back() >= keyframes_.ids().size() ||
      (submap.state != OnlineSubmapState::kActive &&
       submap.state != OnlineSubmapState::kSealed))
    throw std::invalid_argument(
        "Invalid online submap identity, state or coverage");
  if (primary.id == submaps_.size()) {
    if ((!submaps_.empty() &&
         submaps_.back().state != OnlineSubmapState::kSealed) ||
        submap.membership_revision != 1 ||
        submap.state != OnlineSubmapState::kActive ||
        primary.keyframe_ids.size() != 1 ||
        primary.keyframe_ids.front() != indexed_frame_count())
      throw std::invalid_argument(
          "New online submap must extend the sealed prefix");
    std::vector<std::uint64_t> support;
    if (!submaps_.empty()) {
      const auto& previous = submaps_.back().primary.keyframe_ids;
      const auto count =
          std::min(previous.size(),
                   static_cast<std::size_t>(options_.support_keyframes));
      support.assign(previous.end() - count, previous.end());
    }
    if (submap.support_keyframe_ids != support)
      throw std::invalid_argument("Invalid support references");
    return;
  }
  const auto& previous = submaps_[primary.id];
  if (primary.id != submaps_.size() - 1 ||
      previous.state != OnlineSubmapState::kActive ||
      previous.membership_revision ==
          std::numeric_limits<std::uint64_t>::max() ||
      submap.membership_revision != previous.membership_revision + 1 ||
      primary.anchor_keyframe_id != previous.primary.anchor_keyframe_id ||
      submap.support_keyframe_ids != previous.support_keyframe_ids)
    throw std::invalid_argument("Invalid online revision transition");
  const auto& old_ids = previous.primary.keyframe_ids;
  if (submap.state == OnlineSubmapState::kSealed) {
    if (primary.keyframe_ids != old_ids)
      throw std::invalid_argument("Sealing must preserve membership");
  } else if (old_ids.size() >= static_cast<std::size_t>(max_keyframes_) ||
             primary.keyframe_ids.size() != old_ids.size() + 1 ||
             !std::equal(old_ids.begin(), old_ids.end(),
                         primary.keyframe_ids.begin())) {
    throw std::invalid_argument("Insertion must append one primary member");
  }
}

void OnlineSubmapStore::Accept(const OnlineSubmap& submap) {
  if (submap.primary.id == submaps_.size())
    submaps_.push_back(submap);
  else
    submaps_.back() = submap;
}

OnlineSubmap OnlineSubmapStore::ReadRecord(std::uint64_t id,
                                           std::uint64_t revision) const {
  const auto path = directory_ / "records" / std::to_string(id) /
                    (std::to_string(revision) + ".yml");
  try {
    auto yaml = meta::ReadYaml(path);
    Schema(yaml);
    OnlineSubmap result;
    result.primary.id = meta::Id(yaml["id"]);
    result.membership_revision = meta::Id(yaml["membership_revision"]);
    result.primary.anchor_keyframe_id = meta::Id(yaml["anchor_keyframe_id"]);
    result.primary.keyframe_ids =
        ReadIds(yaml["keyframe_ids"],
                std::min(keyframes_.ids().size(),
                         static_cast<std::size_t>(max_keyframes_)));
    result.support_keyframe_ids =
        ReadIds(yaml["support_keyframe_ids"], options_.support_keyframes);
    const auto state = meta::String(yaml["state"]);
    if (state != "active" && state != "sealed")
      meta::Fail(path, "Invalid lifecycle");
    result.state = state == "active" ? OnlineSubmapState::kActive
                                     : OnlineSubmapState::kSealed;
    if (result.primary.id != id || result.membership_revision != revision)
      meta::Fail(path, "Online record identity mismatch");
    ValidateSubmap(result.primary);
    return result;
  } catch (const std::exception& error) {
    meta::Fail(path, error.what());
  }
}

OnlineSubmap OnlineSubmapStore::Read(std::uint64_t id) const {
  if (id >= size()) throw std::out_of_range("Unknown online submap");
  const auto& expected = submaps_[id];
  auto record = ReadRecord(id, expected.membership_revision);
  if (record.primary.anchor_keyframe_id !=
          expected.primary.anchor_keyframe_id ||
      record.primary.keyframe_ids != expected.primary.keyframe_ids ||
      record.support_keyframe_ids != expected.support_keyframe_ids ||
      record.state != expected.state)
    meta::Fail(directory_, "Committed online record changed");
  return record;
}

void OnlineSubmapStore::AppendRevision(const OnlineSubmap& submap) {
  CheckWritable();
  const auto folder =
      directory_ / "records" / std::to_string(submap.primary.id);
  const auto path =
      folder / (std::to_string(submap.membership_revision) + ".yml");
  meta::ValidateInput(path, [&] { ValidateNext(submap); });
  if (fs::exists(path))
    throw std::invalid_argument(path.string() + ": Revision already exists");
  try {
    fs::create_directories(folder);
    const auto pending = folder / (".pending-" + path.filename().string());
    if (fs::exists(pending))
      meta::Fail(pending, "Pending revision already exists");
    meta::WriteYaml(pending, [&](auto& yaml) {
      yaml << "schema_version" << 1 << "id" << std::to_string(submap.primary.id)
           << "membership_revision"
           << std::to_string(submap.membership_revision) << "state"
           << (submap.state == OnlineSubmapState::kActive ? "active" : "sealed")
           << "anchor_keyframe_id"
           << std::to_string(submap.primary.anchor_keyframe_id);
      WriteIds(yaml, "keyframe_ids", submap.primary.keyframe_ids);
      WriteIds(yaml, "support_keyframe_ids", submap.support_keyframe_ids);
    });
    fs::rename(pending, path);
    Accept(submap);
  } catch (const std::exception& error) {
    SaveFailure(path, error);
  }
}

std::vector<RigidTransform> OnlineSubmapStore::RawPoses() const {
  std::vector<RigidTransform> raw;
  for (auto id : keyframes_.ids())
    raw.push_back(keyframes_.ReadRawBodyPose(id));
  return raw;
}

PoseSnapshot OnlineSubmapStore::ReadPoseFile(std::uint64_t version) const {
  const auto path = directory_ / "poses" / (std::to_string(version) + ".yml");
  try {
    auto yaml = meta::ReadYaml(path);
    Schema(yaml);
    PoseSnapshot snapshot;
    snapshot.version = meta::Id(yaml["version"]);
    snapshot.base_version = meta::Id(yaml["base_version"]);
    snapshot.frame_count = meta::Id(yaml["frame_count"]);
    snapshot.optimized_frame_count = meta::Id(yaml["optimized_frame_count"]);
    snapshot.parent_frame = PoseFrame::kMap;
    if (!version || snapshot.version != version ||
        snapshot.frame_count > keyframes_.ids().size() ||
        snapshot.optimized_frame_count == 0 ||
        meta::String(yaml["parent_frame"]) != "map" ||
        meta::String(yaml["tail_policy"]) != "last_optimized_anchor" ||
        meta::Id(yaml["tail_anchor_keyframe_id"]) !=
            snapshot.optimized_frame_count - 1)
      meta::Fail(path, "Invalid frozen pose header");
    const auto poses = yaml["poses"];
    meta::Sequence(poses, snapshot.frame_count);
    for (const auto& row : poses)
      snapshot.poses.push_back(
          {meta::Id(row["id"]), meta::ReadTransform(row["map_from_body"])});
    return livo_atlas::MaterializePoseSnapshot(snapshot, RawPoses(),
                                               snapshot.frame_count);
  } catch (const std::exception& error) {
    meta::Fail(path, error.what());
  }
}

PoseSnapshot OnlineSubmapStore::ReadPoseSnapshot(std::uint64_t version) const {
  if (!version)
    return livo_atlas::MaterializePoseSnapshot({}, RawPoses(),
                                               keyframes_.ids().size());
  if (version > pose_version_)
    throw std::out_of_range("Unknown online pose version");
  return ReadPoseFile(version);
}

PoseSnapshot OnlineSubmapStore::MaterializePoseSnapshot(
    std::uint64_t version, std::uint64_t frame_count) const {
  return livo_atlas::MaterializePoseSnapshot(ReadPoseSnapshot(version),
                                             RawPoses(), frame_count);
}

void OnlineSubmapStore::WritePoseSnapshot(const PoseSnapshot& snapshot) {
  CheckWritable();
  const auto path =
      directory_ / "poses" / (std::to_string(snapshot.version) + ".yml");
  meta::ValidateInput(path, [&] {
    ValidatePoseSnapshot(snapshot);
    if (pose_version_ == std::numeric_limits<std::uint64_t>::max() ||
        snapshot.version != pose_version_ + 1 ||
        snapshot.optimized_frame_count < optimized_count_ ||
        snapshot.frame_count != keyframes_.ids().size())
      throw std::invalid_argument(
          "Invalid next online pose version or frontier");
    (void)livo_atlas::MaterializePoseSnapshot(snapshot, RawPoses(),
                                              snapshot.frame_count);
  });
  if (fs::exists(path))
    throw std::invalid_argument(path.string() +
                                ": Pose version already exists");
  try {
    const auto pending =
        path.parent_path() / (".pending-" + path.filename().string());
    if (fs::exists(pending)) meta::Fail(pending, "Pending pose already exists");
    meta::WriteYaml(pending, [&](auto& yaml) {
      yaml << "schema_version" << 1 << "version"
           << std::to_string(snapshot.version) << "base_version"
           << std::to_string(snapshot.base_version) << "parent_frame"
           << "map"
           << "frame_count" << std::to_string(snapshot.frame_count)
           << "optimized_frame_count"
           << std::to_string(snapshot.optimized_frame_count) << "tail_policy"
           << "last_optimized_anchor"
           << "tail_anchor_keyframe_id"
           << std::to_string(snapshot.optimized_frame_count - 1) << "poses"
           << "[";
      for (const auto& pose : snapshot.poses) {
        yaml << "{"
             << "id" << std::to_string(pose.id);
        meta::WriteTransform(yaml, "map_from_body", pose.map_from_body);
        yaml << "}";
      }
      yaml << "]";
    });
    fs::rename(pending, path);
    pose_version_ = snapshot.version;
    optimized_count_ = snapshot.optimized_frame_count;
  } catch (const std::exception& error) {
    SaveFailure(path, error);
  }
}
}  // namespace livo_atlas
