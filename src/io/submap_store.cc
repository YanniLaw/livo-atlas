#include "livo_atlas/io/submap_store.h"

#include <algorithm>
#include <limits>
#include <map>
#include <opencv2/core/persistence.hpp>
#include <stdexcept>
#include <string>
#include <utility>

#include "dataset_metadata.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;
using dataset_metadata::Fail;
using dataset_metadata::Id;
using dataset_metadata::Integer;
using dataset_metadata::ParseId;
using dataset_metadata::ValidateInput;
using dataset_metadata::WriteYaml;

RigidTransform Inverse(const RigidTransform& pose) {
  const Eigen::Matrix3d rotation = pose.rotation.transpose();
  return {rotation, -rotation * pose.translation};
}
RigidTransform Compose(const RigidTransform& a, const RigidTransform& b) {
  return {a.rotation * b.rotation, a.rotation * b.translation + a.translation};
}
}  // namespace

SubmapStore::SubmapStore(const fs::path& directory,
                         const KeyframeStore& keyframes, int max_keyframes)
    : directory_(directory),
      keyframes_(keyframes),
      max_keyframes_(max_keyframes),
      append_enabled_(true) {
  ValidateInput(directory_, [&] {
    ValidateSubmapOptions({true, max_keyframes_});
    if (directory_.empty())
      throw std::invalid_argument("Empty submap directory");
  });
  try {
    if (!directory_.parent_path().empty())
      fs::create_directories(directory_.parent_path());
    if (!fs::create_directory(directory_))
      Fail(directory_, "Submaps already exist");
    WriteYaml(directory_ / "dataset.yml", [&](auto& yaml) {
      yaml << "schema_version" << 1 << "max_keyframes" << max_keyframes_;
    });
  } catch (const std::exception& error) {
    Fail(directory_, error.what());
  }
}

SubmapStore::SubmapStore(const fs::path& directory,
                         const KeyframeStore& keyframes)
    : directory_(directory), keyframes_(keyframes) {
  try {
    const auto metadata = directory_ / "dataset.yml";
    if (!fs::is_regular_file(metadata))
      Fail(metadata, "Missing submap metadata");
    cv::FileStorage yaml(metadata.string(), cv::FileStorage::READ);
    if (!yaml.isOpened() || Integer(yaml["schema_version"]) != 1)
      Fail(metadata, "Unsupported submap schema");
    max_keyframes_ = Integer(yaml["max_keyframes"]);
    ValidateSubmapOptions({true, max_keyframes_});
    std::vector<std::uint64_t> ids;
    for (const auto& entry : fs::directory_iterator(directory_)) {
      const auto name = entry.path().filename().string();
      if (name == "dataset.yml" || name.rfind(".pending-", 0) == 0) continue;
      if (!entry.is_regular_file() || entry.path().extension() != ".yml")
        Fail(entry.path(), "Unexpected submap asset");
      ids.push_back(ParseId(entry.path().stem().string()));
    }
    std::sort(ids.begin(), ids.end());
    for (auto id : ids) {
      auto submap = ReadFile(id);
      ValidateNext(submap);
      submaps_.push_back(std::move(submap));
    }
  } catch (const std::exception& error) {
    Fail(directory_, error.what());
  }
}

void SubmapStore::ValidateNext(const Submap& submap) const {
  ValidateSubmap(submap);
  const auto& ids = submap.keyframe_ids;
  if (submap.id != submaps_.size() ||
      ids.size() > static_cast<std::size_t>(max_keyframes_) ||
      ids.back() >= keyframes_.ids().size()) {
    throw std::invalid_argument(
        "Unexpected submap ID, size or uncommitted member");
  }
  if (submaps_.empty()) {
    if (ids.front() != 0)
      throw std::invalid_argument(
          "Submap membership must start at keyframe zero");
  } else {
    const auto& previous = submaps_.back().keyframe_ids;
    if (previous.size() != static_cast<std::size_t>(max_keyframes_) ||
        ids.front() <= previous.back() || ids.front() - previous.back() != 1) {
      throw std::invalid_argument(
          "Submap members must extend the committed prefix");
    }
  }
}

Submap SubmapStore::ReadFile(std::uint64_t id) const {
  const auto path = directory_ / (std::to_string(id) + ".yml");
  try {
    if (!fs::is_regular_file(path)) Fail(path, "Missing committed submap");
    cv::FileStorage yaml(path.string(), cv::FileStorage::READ);
    if (!yaml.isOpened()) Fail(path, "Cannot read submap");
    Submap result;
    result.id = Id(yaml["id"]);
    result.anchor_keyframe_id = Id(yaml["anchor_keyframe_id"]);
    const auto members = yaml["keyframe_ids"];
    if (result.id != id || !members.isSeq() || members.empty() ||
        members.size() > static_cast<std::size_t>(max_keyframes_) ||
        members.size() > keyframes_.ids().size()) {
      Fail(path, "Invalid submap membership shape");
    }
    for (const auto& member : members)
      result.keyframe_ids.push_back(Id(member));
    ValidateSubmap(result);
    if (result.keyframe_ids.back() >= keyframes_.ids().size())
      Fail(path, "Unknown submap member");
    return result;
  } catch (const std::exception& error) {
    Fail(path, error.what());
  }
}

void SubmapStore::Append(const Submap& submap) {
  if (failed_ || !append_enabled_)
    throw std::logic_error(directory_.string() +
                           ": Submap writer is not writable");
  const auto path = directory_ / (std::to_string(submap.id) + ".yml");
  ValidateInput(path, [&] { ValidateNext(submap); });
  const auto pending =
      directory_ / (".pending-" + std::to_string(submap.id) + ".yml");
  try {
    if (fs::exists(path) || fs::exists(pending))
      Fail(path, "Submap or uncommitted version already exists");
    WriteYaml(pending, [&](auto& yaml) {
      yaml << "id" << std::to_string(submap.id) << "anchor_keyframe_id"
           << std::to_string(submap.anchor_keyframe_id) << "keyframe_ids"
           << "[";
      for (auto id : submap.keyframe_ids) yaml << std::to_string(id);
      yaml << "]";
    });
    fs::rename(pending, path);
    submaps_.push_back(submap);
  } catch (const std::exception& error) {
    failed_ = true;
    Fail(path, error.what());
  }
}

Submap SubmapStore::Read(std::uint64_t id) const {
  if (id >= submaps_.size())
    throw std::out_of_range(directory_.string() + ": Unknown submap " +
                            std::to_string(id));
  auto result = ReadFile(id);
  const auto& expected = submaps_[id];
  if (result.anchor_keyframe_id != expected.anchor_keyframe_id ||
      result.keyframe_ids != expected.keyframe_ids)
    Fail(directory_ / (std::to_string(id) + ".yml"),
         "Committed submap membership changed");
  return result;
}

SubmapCloud SubmapStore::ReadCloud(std::uint64_t id,
                                   std::uint64_t pose_version) const {
  const auto group = Read(id);
  const auto path = directory_ / (std::to_string(id) + ".yml");
  try {
    std::map<std::uint64_t, RigidTransform> poses;
    if (pose_version) {
      for (const auto& pose : keyframes_.ReadPoses(pose_version))
        poses.emplace(pose.id, pose.map_from_body);
      for (auto member : group.keyframe_ids)
        if (!poses.count(member))
          Fail(path, "Pose version does not cover submap members");
    }
    const auto anchor = keyframes_.Read(group.anchor_keyframe_id);
    SubmapCloud result;
    result.id = id;
    result.pose_version = pose_version;
    result.parent_from_submap = pose_version
                                    ? poses.at(group.anchor_keyframe_id)
                                    : anchor.odom_from_body;
    const auto submap_from_parent = Inverse(result.parent_from_submap);
    for (auto member : group.keyframe_ids) {
      const auto frame = member == anchor.id ? anchor : keyframes_.Read(member);
      const auto& parent_from_body =
          pose_version ? poses.at(member) : frame.odom_from_body;
      const auto cloud = TransformKeyframeCloud(
          frame, Compose(submap_from_parent, parent_from_body));
      if (cloud.size() >
          std::numeric_limits<std::uint32_t>::max() - result.cloud.size())
        Fail(path, "Submap exceeds the PCL point count range");
      result.cloud += cloud;
    }
    return result;
  } catch (const std::exception& error) {
    Fail(path, error.what());
  }
}
}  // namespace livo_atlas
