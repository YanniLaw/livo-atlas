#include "livo_atlas/gaussian/gaussian_map_index.h"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

#include "mapping/pose_math.h"

namespace livo_atlas {
namespace {

constexpr std::size_t kMaxFrames = 512;
constexpr std::size_t kMaxSubmaps = 32;

bool Hex(const std::string& value, std::size_t size) {
  return value.size() == size &&
         std::all_of(value.begin(), value.end(), [](char c) {
           return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
         });
}

bool Prefix(const std::vector<std::uint64_t>& prefix,
            const std::vector<std::uint64_t>& ids) {
  return prefix.size() <= ids.size() &&
         std::equal(prefix.begin(), prefix.end(), ids.begin());
}

void ValidateGroup(const OnlineSubmap& group, std::uint64_t frontier) {
  ValidateSubmap(group.primary);
  const auto& ids = group.primary.keyframe_ids;
  const bool sealed = group.state == OnlineSubmapState::kSealed;
  if ((group.state != OnlineSubmapState::kActive && !sealed) ||
      ids.size() > kMaxFrames || ids.back() >= frontier ||
      group.membership_revision != ids.size() + (sealed ? 1 : 0) ||
      group.support_keyframe_ids.size() > kMaxFrames)
    throw std::invalid_argument("Invalid Gaussian primary membership");
  const auto& support = group.support_keyframe_ids;
  if (!std::is_sorted(support.begin(), support.end()) ||
      std::adjacent_find(support.begin(), support.end()) != support.end() ||
      (!support.empty() && support.back() >= ids.front()))
    throw std::invalid_argument("Invalid Gaussian support membership");
}

void ApplyRevision(std::vector<GaussianSubmapEntry>* entries,
                   const OnlineSubmap& group) {
  const auto id = group.primary.id;
  const auto& ids = group.primary.keyframe_ids;
  if (id == entries->size()) {
    if (entries->size() == kMaxFrames || ids.size() != 1 ||
        group.state != OnlineSubmapState::kActive)
      throw std::invalid_argument("Invalid new Gaussian submap");
    if (entries->empty()) {
      if (ids.front() != 0 || !group.support_keyframe_ids.empty())
        throw std::invalid_argument(
            "Gaussian grouping must start at frame zero");
    } else {
      const auto& previous = entries->back().group;
      const auto& old = previous.primary.keyframe_ids;
      const auto& support = group.support_keyframe_ids;
      if (previous.state != OnlineSubmapState::kSealed ||
          ids.front() != old.back() + 1 || support.size() > old.size() ||
          !std::equal(support.begin(), support.end(),
                      old.end() - support.size()))
        throw std::invalid_argument("Invalid Gaussian group boundary");
    }
    GaussianSubmapEntry entry;
    entry.group = group;
    entries->push_back(std::move(entry));
    return;
  }
  if (entries->empty() || id != entries->size() - 1)
    throw std::invalid_argument("Gaussian revision is not the current group");
  auto& previous = entries->back().group;
  const auto& old = previous.primary.keyframe_ids;
  if (group.primary.anchor_keyframe_id != previous.primary.anchor_keyframe_id ||
      group.support_keyframe_ids != previous.support_keyframe_ids)
    throw std::invalid_argument("Gaussian group identity changed");
  if (group.membership_revision == previous.membership_revision &&
      group.state == previous.state && ids == old)
    return;
  if (previous.state != OnlineSubmapState::kActive ||
      group.membership_revision != previous.membership_revision + 1 ||
      !Prefix(old, ids) ||
      !((ids.size() == old.size() + 1 &&
         group.state == OnlineSubmapState::kActive) ||
        (ids == old && group.state == OnlineSubmapState::kSealed)))
    throw std::invalid_argument("Invalid Gaussian membership transition");
  previous = group;
}

GaussianTarget Target(const std::string& session, const OnlineSubmap& group,
                      const PoseSnapshot& poses, std::uint64_t generation) {
  GaussianTarget result;
  result.session_id = session;
  result.generation = generation;
  result.group = group;
  result.pose_version = poses.version;
  result.frame_count = poses.frame_count;
  result.optimized_frame_count = poses.optimized_frame_count;
  result.parent_frame = poses.parent_frame;
  result.parent_from_submap =
      poses.poses.at(group.primary.anchor_keyframe_id).map_from_body;
  const auto inverse = pose_math::Inverse(result.parent_from_submap);
  for (const auto id : group.primary.keyframe_ids)
    result.submap_from_body.push_back(
        pose_math::Compose(inverse, poses.poses.at(id).map_from_body));
  return result;
}

bool Unchanged(const GaussianTarget& source, const PoseSnapshot& poses) {
  const auto anchor = source.group.primary.anchor_keyframe_id;
  const auto inverse = pose_math::Inverse(poses.poses.at(anchor).map_from_body);
  const auto& ids = source.group.primary.keyframe_ids;
  for (std::size_t i = 0; i < ids.size(); ++i) {
    const auto local =
        pose_math::Compose(inverse, poses.poses.at(ids[i]).map_from_body);
    if (!pose_math::Equal(source.submap_from_body[i], local)) return false;
  }
  return true;
}

void Advance(std::uint64_t* sequence) {
  if (*sequence == std::numeric_limits<std::uint64_t>::max())
    throw std::overflow_error("Gaussian metadata sequence overflow");
  ++*sequence;
}

}  // namespace

bool operator==(const GaussianContentId& left, const GaussianContentId& right) {
  return left.session_id == right.session_id &&
         left.submap_id == right.submap_id &&
         left.source_generation == right.source_generation &&
         left.content_job_id == right.content_job_id;
}

bool operator!=(const GaussianContentId& left, const GaussianContentId& right) {
  return !(left == right);
}

void ValidateGaussianTarget(const GaussianTarget& target) {
  if (!Hex(target.session_id, 32) || target.frame_count > kMaxFrames ||
      target.optimized_frame_count > target.frame_count ||
      (target.pose_version == 0 ? target.parent_frame != PoseFrame::kOdom ||
                                      target.optimized_frame_count != 0
                                : target.parent_frame != PoseFrame::kMap ||
                                      target.optimized_frame_count == 0))
    throw std::invalid_argument("Invalid Gaussian target identity or frontier");
  ValidateGroup(target.group, target.frame_count);
  if (target.submap_from_body.size() !=
      target.group.primary.keyframe_ids.size())
    throw std::invalid_argument("Gaussian target omits primary body poses");
  pose_math::Validate(target.parent_from_submap);
  for (const auto& pose : target.submap_from_body) pose_math::Validate(pose);
  if (!pose_math::Equal(target.submap_from_body.front(), {}) ||
      (target.submap_from_body.front().rotation - Eigen::Matrix3d::Identity())
              .cwiseAbs()
              .maxCoeff() > 1e-9)
    throw std::invalid_argument("Gaussian anchor body must define the origin");
}

void ValidateGaussianAssetReference(const GaussianAssetReference& asset) {
  if (!asset.asset_id || !Hex(asset.content.session_id, 32) ||
      !asset.content.content_job_id || !Hex(asset.manifest_sha256, 64) ||
      !asset.manifest_bytes || asset.manifest_bytes > 512 * 1024 ||
      !asset.gaussian_bytes || asset.gaussian_bytes > 1024 * 1024 ||
      !asset.observation_bytes || asset.observation_bytes > 192 * 1024 * 1024)
    throw std::invalid_argument("Invalid Gaussian asset reference");
}

GaussianMapIndex::GaussianMapIndex(std::string session_id)
    : session_id_(std::move(session_id)) {
  if (!Hex(session_id_, 32))
    throw std::invalid_argument("Invalid Gaussian session");
}

void GaussianMapIndex::Observe(const OnlineSubmapUpdate& update) {
  if (!update.poses || update.poses->frame_count > kMaxFrames ||
      update.revisions.size() > 2)
    throw std::invalid_argument("Gaussian observation exceeds metadata limits");
  const auto& next = *update.poses;
  ValidatePoseSnapshot(next);
  if (poses_) {
    if (next.version < poses_->version ||
        next.frame_count < poses_->frame_count ||
        next.optimized_frame_count < poses_->optimized_frame_count)
      throw std::invalid_argument("Gaussian pose frontier moved backwards");
    if (next.version == poses_->version) {
      if (next.optimized_frame_count != poses_->optimized_frame_count)
        throw std::invalid_argument(
            "Gaussian optimized frontier changed within a version");
      for (std::size_t i = 0; i < poses_->poses.size(); ++i)
        if (!pose_math::Equal(poses_->poses[i].map_from_body,
                              next.poses[i].map_from_body))
          throw std::invalid_argument("Gaussian pose changed within a version");
    }
  }
  auto entries = entries_;
  bool found = false;
  for (const auto& group : update.revisions) {
    ValidateGroup(group, next.frame_count);
    ApplyRevision(&entries, group);
    if (update.keyframe &&
        group.primary.keyframe_ids.back() == update.keyframe->id)
      found = true;
  }
  if (update.keyframe &&
      (!found || update.keyframe->id + 1 != next.frame_count))
    throw std::invalid_argument(
        "Gaussian observation is not the committed primary tail");
  Advance(&sequence_);
  entries_ = std::move(entries);
  poses_ = update.poses;
  Reclassify();
}

GaussianTarget GaussianMapIndex::MakeTarget(std::uint64_t submap_id,
                                            std::uint64_t generation) const {
  if (!poses_ || submap_id >= entries_.size())
    throw std::invalid_argument("Unknown Gaussian target group");
  return Target(session_id_, entries_[submap_id].group, *poses_, generation);
}

bool GaussianMapIndex::CanTrain(std::uint64_t submap_id) const {
  if (submap_id >= entries_.size()) return false;
  if (entries_[submap_id].content) return true;
  return std::count_if(entries_.begin(), entries_.end(), [](const auto& entry) {
           return entry.content.has_value();
         }) < static_cast<int>(kMaxSubmaps);
}

void GaussianMapIndex::RecordConfirmed(
    const GaussianTarget& target, GaussianOperation operation,
    std::uint64_t job_id, const std::vector<std::uint64_t>& frame_ids,
    int gaussian_count) {
  ValidateGaussianTarget(target);
  const auto id = target.group.primary.id;
  if (target.session_id != session_id_ || !poses_ || id >= entries_.size() ||
      target.frame_count > poses_->frame_count ||
      target.pose_version > poses_->version ||
      !Prefix(target.group.primary.keyframe_ids,
              entries_[id].group.primary.keyframe_ids) ||
      !job_id || job_id <= entries_[id].binding_job_id || frame_ids.empty() ||
      frame_ids.size() > 10 || gaussian_count < 1 || gaussian_count > 10000 ||
      gaussian_count > static_cast<int>(frame_ids.size()) * 1000 ||
      !std::is_sorted(frame_ids.begin(), frame_ids.end()) ||
      std::adjacent_find(frame_ids.begin(), frame_ids.end()) !=
          frame_ids.end() ||
      frame_ids.front() < target.group.primary.anchor_keyframe_id ||
      frame_ids.back() > target.group.primary.keyframe_ids.back() ||
      !CanTrain(id))
    throw std::invalid_argument("Invalid Gaussian confirmation or coverage");
  auto& entry = entries_[id];
  if (operation == GaussianOperation::kReuse) {
    if (!entry.content || frame_ids != entry.frame_ids ||
        gaussian_count != entry.gaussian_count ||
        target.submap_from_body.size() <
            entry.content_target.submap_from_body.size())
      throw std::invalid_argument("Gaussian reuse changed confirmed coverage");
    for (std::size_t i = 0; i < entry.content_target.submap_from_body.size();
         ++i)
      if (!pose_math::Equal(target.submap_from_body[i],
                            entry.content_target.submap_from_body[i]))
        throw std::invalid_argument(
            "Gaussian reuse changed canonical body poses");
  } else if (operation == GaussianOperation::kView ||
             operation == GaussianOperation::kRebuild) {
    entry.content =
        GaussianContentId{session_id_, id, target.generation, job_id};
    entry.content_target = target;
    entry.frame_ids = frame_ids;
    entry.gaussian_count = gaussian_count;
  } else {
    throw std::invalid_argument("Unknown Gaussian operation");
  }
  for (auto& item : entries_) item.resident = false;
  entry.resident = true;
  entry.binding_job_id = job_id;
  entry.binding.generation = target.generation;
  Advance(&sequence_);
  Reclassify();
}

bool GaussianMapIndex::RecordCheckpoint(const GaussianAssetReference& asset) {
  ValidateGaussianAssetReference(asset);
  const auto id = asset.content.submap_id;
  if (asset.content.session_id != session_id_ || id >= entries_.size())
    throw std::invalid_argument(
        "Gaussian checkpoint belongs to another session");
  auto& entry = entries_[id];
  if (!entry.content || asset.content != *entry.content) return false;
  if (asset.asset_id <= last_asset_id_)
    throw std::invalid_argument("Gaussian asset ID did not advance");
  entry.asset = asset;
  entry.asset_source = entry.content_target;
  entry.asset_frame_ids = entry.frame_ids;
  entry.asset_gaussian_count = entry.gaussian_count;
  last_asset_id_ = asset.asset_id;
  Advance(&sequence_);
  Reclassify();
  return true;
}

std::optional<std::uint64_t> GaussianMapIndex::NextRepair() const {
  const GaussianSubmapEntry* selected = nullptr;
  for (const auto& entry : entries_) {
    if (entry.availability != GaussianAvailability::kNeedsRebuild) continue;
    if (!selected || entry.last_repair_attempt < selected->last_repair_attempt)
      selected = &entry;
  }
  return selected ? std::optional<std::uint64_t>(selected->group.primary.id)
                  : std::nullopt;
}

void GaussianMapIndex::MarkRepairAttempt(std::uint64_t submap_id) {
  if (submap_id >= entries_.size() ||
      entries_[submap_id].availability != GaussianAvailability::kNeedsRebuild)
    throw std::invalid_argument("Gaussian group does not need repair");
  Advance(&repair_sequence_);
  entries_[submap_id].last_repair_attempt = repair_sequence_;
}

void GaussianMapIndex::Reclassify() {
  const bool has_capacity =
      std::count_if(entries_.begin(), entries_.end(), [](const auto& entry) {
        return entry.content.has_value();
      }) < static_cast<int>(kMaxSubmaps);
  for (auto& entry : entries_) {
    entry.binding =
        Target(session_id_, entry.group, *poses_, entry.binding.generation);
    if (!entry.content) {
      entry.availability = GaussianAvailability::kMissing;
      entry.reason = has_capacity ? "untrained" : "resource_limit";
      continue;
    }
    const bool current = Unchanged(entry.content_target, *poses_);
    entry.availability = current ? GaussianAvailability::kCurrent
                                 : GaussianAvailability::kNeedsRebuild;
    entry.reason = current ? "" : "primary_pose_changed";
    entry.dirty = !entry.asset || entry.asset->content != *entry.content;
    entry.asset_current = entry.asset && Unchanged(entry.asset_source, *poses_);
  }
}

GaussianMapSnapshot GaussianMapIndex::Snapshot() const {
  GaussianMapSnapshot result;
  result.session_id = session_id_;
  result.map_sequence = sequence_;
  if (poses_) {
    result.pose_version = poses_->version;
    result.frame_count = poses_->frame_count;
    result.optimized_frame_count = poses_->optimized_frame_count;
    result.parent_frame = poses_->parent_frame;
  }
  result.entries = entries_;
  return result;
}

}  // namespace livo_atlas
