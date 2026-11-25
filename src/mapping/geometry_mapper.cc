#include "livo_atlas/mapping/geometry_mapper.h"

#include <algorithm>
#include <chrono>
#include <limits>
#include <stdexcept>
#include <utility>

#include "geometry_validation.h"
#include "pose_math.h"

namespace livo_atlas {
namespace {

using Clock = std::chrono::steady_clock;

double Seconds(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

}  // namespace

namespace geometry_detail {

bool SameMembership(const OnlineSubmap& a, const OnlineSubmap& b) {
  return a.primary.id == b.primary.id &&
         a.primary.anchor_keyframe_id == b.primary.anchor_keyframe_id &&
         a.primary.keyframe_ids == b.primary.keyframe_ids &&
         a.support_keyframe_ids == b.support_keyframe_ids &&
         a.membership_revision == b.membership_revision && a.state == b.state;
}

void ValidateRevision(const OnlineSubmap& group, std::uint64_t frame_count) {
  ValidateSubmap(group.primary);
  const auto& members = group.primary.keyframe_ids;
  const bool sealed = group.state == OnlineSubmapState::kSealed;
  if ((group.state != OnlineSubmapState::kActive && !sealed) ||
      members.size() > kMaxGeometryKeyframes ||
      group.membership_revision != members.size() + (sealed ? 1 : 0) ||
      members.back() >= frame_count) {
    throw std::invalid_argument("Invalid geometry submap revision");
  }
}

void ValidateGroups(const std::vector<OnlineSubmap>& groups,
                    const PoseSnapshot& poses) {
  std::uint64_t next_frame = 0;
  for (std::size_t i = 0; i < groups.size(); ++i) {
    const auto& group = groups[i];
    ValidateRevision(group, poses.frame_count);
    const auto& members = group.primary.keyframe_ids;
    const bool sealed = group.state == OnlineSubmapState::kSealed;
    if (group.primary.id != i || members.front() != next_frame ||
        (!sealed && i + 1 != groups.size())) {
      throw std::invalid_argument("Invalid geometry submap membership");
    }
    next_frame = members.back() + 1;
    const auto& support = group.support_keyframe_ids;
    if (i == 0) {
      if (!support.empty())
        throw std::invalid_argument("First submap has support");
    } else {
      const auto& previous = groups[i - 1].primary.keyframe_ids;
      if (support.size() > previous.size() ||
          !std::equal(support.begin(), support.end(),
                      previous.end() - support.size())) {
        throw std::invalid_argument("Support must be a previous submap suffix");
      }
    }
  }
}

std::vector<OnlineSubmap> ReviseSubmapTail(
    const OnlineSubmap* last, const std::vector<OnlineSubmap>& revisions,
    std::uint64_t frame_count) {
  if (revisions.size() > 2)
    throw std::invalid_argument("Too many geometry membership revisions");
  std::vector<OnlineSubmap> tail;
  if (last) tail.push_back(*last);
  for (const auto& group : revisions) {
    ValidateRevision(group, frame_count);
    const auto id = group.primary.id;
    const auto& members = group.primary.keyframe_ids;
    const auto& support = group.support_keyframe_ids;
    const std::uint64_t next_id = tail.empty() ? 0 : tail.back().primary.id + 1;
    if (id == next_id) {
      if (group.state != OnlineSubmapState::kActive || members.size() != 1 ||
          group.membership_revision != 1)
        throw std::invalid_argument("Invalid new geometry membership");
      if (tail.empty()) {
        if (members.front() != 0 || !support.empty())
          throw std::invalid_argument("Invalid first geometry membership");
      } else {
        const auto& previous = tail.back();
        const auto& prior = previous.primary.keyframe_ids;
        if (previous.state != OnlineSubmapState::kSealed ||
            members.front() != prior.back() + 1 ||
            support.size() > prior.size() ||
            !std::equal(support.begin(), support.end(),
                        prior.end() - support.size()))
          throw std::invalid_argument("Invalid geometry membership boundary");
      }
      tail.push_back(group);
      continue;
    }
    if (tail.empty() || id != tail.back().primary.id)
      throw std::invalid_argument("Invalid geometry membership ID");
    auto& old = tail.back();
    if (SameMembership(old, group)) continue;
    const auto& before = old.primary.keyframe_ids;
    const bool appended = members.size() == before.size() + 1 &&
                          group.state == OnlineSubmapState::kActive;
    const bool sealed = members.size() == before.size() &&
                        group.state == OnlineSubmapState::kSealed;
    if (old.state != OnlineSubmapState::kActive ||
        group.membership_revision != old.membership_revision + 1 ||
        group.primary.anchor_keyframe_id != old.primary.anchor_keyframe_id ||
        support != old.support_keyframe_ids || (!appended && !sealed) ||
        !std::equal(before.begin(), before.end(), members.begin()))
      throw std::invalid_argument("Invalid geometry membership revision");
    old = group;
  }
  const auto covered =
      tail.empty() ? 0 : tail.back().primary.keyframe_ids.back() + 1;
  if (covered != frame_count)
    throw std::invalid_argument("Geometry membership does not cover poses");
  return tail;
}

}  // namespace geometry_detail

namespace {

using geometry_detail::SameMembership;
using geometry_detail::ValidateGroups;
using geometry_detail::ValidateRevision;

RigidTransform LocalPose(const OnlineSubmap& group, const PoseSnapshot& poses,
                         std::uint64_t id) {
  return pose_math::Compose(
      pose_math::Inverse(
          poses.poses[group.primary.anchor_keyframe_id].parent_from_body()),
      poses.poses[id].parent_from_body());
}

bool SamePoseExactly(const RigidTransform& a, const RigidTransform& b) {
  return (a.rotation.array() == b.rotation.array()).all() &&
         (a.translation.array() == b.translation.array()).all();
}

}  // namespace

GeometryMapper::GeometryMapper(GeometryOptions options,
                               CloudReader cloud_reader)
    : options_(options), cloud_reader_(std::move(cloud_reader)) {
  ValidateGeometryOptions(options_);
  if (!cloud_reader_)
    throw std::invalid_argument("Missing geometry cloud reader");
}

void GeometryMapper::ResetTimings() {
  stats_.last_update_seconds = 0;
  stats_.last_cloud_read_seconds = 0;
  stats_.last_full_build_seconds = 0;
  stats_.last_snapshot_seconds = 0;
}

const RigidTransform& GeometryMapper::PoseAt(std::uint64_t id) const {
  const auto prefix = poses_ ? poses_->frame_count : 0;
  if (id < prefix) return poses_->poses.at(id).parent_from_body();
  return tail_poses_.at(id - prefix);
}

RigidTransform GeometryMapper::SubmapFromBody(const OnlineSubmap& group,
                                              std::uint64_t id) const {
  return pose_math::Compose(
      pose_math::Inverse(PoseAt(group.primary.anchor_keyframe_id)), PoseAt(id));
}

void GeometryMapper::Update(const std::vector<OnlineSubmap>& revisions,
                            std::shared_ptr<const PoseSnapshot> poses,
                            const Keyframe* new_keyframe) {
  const auto start = Clock::now();
  ResetTimings();
  if (!poses) throw std::invalid_argument("Missing geometry pose snapshot");
  ValidatePoseSnapshot(*poses);
  if (poses->version < frontier_.version ||
      poses->frame_count < frontier_.frame_count ||
      poses->optimized_frame_count < frontier_.optimized_frame_count) {
    throw std::invalid_argument("Geometry pose snapshot moved backwards");
  }
  const bool version_changed = poses->version != frontier_.version;
  if (!version_changed) {
    auto expected = frontier_;
    expected.frame_count = poses->frame_count;
    if (!(poses->frontier() == expected))
      throw std::invalid_argument("Geometry pose version changed in place");
    for (std::uint64_t i = 0; i < frontier_.frame_count; ++i) {
      if (!SamePoseExactly(PoseAt(i), poses->poses[i].parent_from_body())) {
        throw std::invalid_argument("Geometry pose version changed in place");
      }
    }
  }

  std::vector<OnlineSubmap> target;
  target.reserve(entries_.size() + revisions.size());
  for (const auto& entry : entries_) target.push_back(entry.membership);
  for (const auto& revision : revisions) {
    ValidateRevision(revision, poses->frame_count);
    const auto id = revision.primary.id;
    if (id > target.size())
      throw std::invalid_argument("Geometry submap ID gap");
    if (id == target.size()) {
      if (!target.empty() &&
          target.back().state != OnlineSubmapState::kSealed) {
        throw std::invalid_argument("Previous geometry submap is still active");
      }
      target.push_back(revision);
      continue;
    }
    auto& old = target[id];
    if (SameMembership(old, revision)) continue;
    const auto& before = old.primary.keyframe_ids;
    const auto& after = revision.primary.keyframe_ids;
    if (revision.membership_revision <= old.membership_revision ||
        old.state == OnlineSubmapState::kSealed ||
        old.primary.anchor_keyframe_id != revision.primary.anchor_keyframe_id ||
        old.support_keyframe_ids != revision.support_keyframe_ids ||
        after.size() < before.size() ||
        !std::equal(before.begin(), before.end(), after.begin())) {
      throw std::invalid_argument("Invalid geometry submap revision");
    }
    old = revision;
  }
  ValidateGroups(target, *poses);
  if (new_keyframe &&
      (!new_keyframe->cloud || target.empty() || !poses->frame_count ||
       new_keyframe->id != poses->frame_count - 1 ||
       target.back().primary.keyframe_ids.back() != new_keyframe->id)) {
    throw std::invalid_argument(
        "Geometry new keyframe is not the committed tail");
  }

  const auto old_count = entries_.size();
  entries_.resize(target.size());
  for (std::size_t i = 0; i < target.size(); ++i) {
    auto& entry = entries_[i];
    const std::size_t prior_count =
        i < old_count ? entry.membership.primary.keyframe_ids.size() : 0;
    entry.membership = std::move(target[i]);
    if (!entry.data) continue;
    bool changed = false;
    for (const auto& source : entry.built_sources) {
      if (!pose_math::Equal(
              source.submap_from_body,
              LocalPose(entry.membership, *poses, source.keyframe_id))) {
        changed = true;
        break;
      }
    }
    const bool can_append =
        new_keyframe && i + 1 == entries_.size() &&
        prior_count + 1 == entry.membership.primary.keyframe_ids.size();
    if (changed || (entry.built_sources.size() !=
                        entry.membership.primary.keyframe_ids.size() &&
                    !can_append)) {
      ReleaseData(i);
    } else if (version_changed) {
      ++stats_.rigid_updates;
    }
  }
  frontier_ = poses->frontier();
  poses_ = std::move(poses);
  tail_poses_.clear();
  if (new_keyframe) UpdateNewKeyframe(*new_keyframe);
  for (auto& entry : entries_) {
    if (entry.membership.state == OnlineSubmapState::kSealed)
      entry.builder.reset();
  }
  stats_.last_update_seconds = Seconds(start);
}

void GeometryMapper::UpdateDelta(const std::vector<OnlineSubmap>& revisions,
                                 const PoseFrontier& frontier,
                                 const Keyframe* new_keyframe,
                                 const RigidTransform* new_pose) {
  const auto start = Clock::now();
  ResetTimings();
  auto expected = frontier_;
  if (new_keyframe) ++expected.frame_count;
  if (!(frontier == expected) || bool(new_keyframe) != bool(new_pose) ||
      (new_keyframe &&
       (new_keyframe->id != frontier_.frame_count || !new_keyframe->cloud))) {
    throw std::invalid_argument("Invalid geometry append or seal frontier");
  }
  if (new_pose)
    pose_math::ValidateSnapshotPose(*new_pose, frontier.tail_policy);

  const auto old_count = entries_.size();
  const auto first = entries_.empty() ? 0 : old_count - 1;
  auto tail = geometry_detail::ReviseSubmapTail(
      entries_.empty() ? nullptr : &entries_.back().membership, revisions,
      frontier.frame_count);
  // Stage the pose before allocating the new member slot. Invalid revisions
  // have already been rejected, leaving the existing cache intact.
  if (new_pose) tail_poses_.push_back(*new_pose);
  try {
    entries_.resize(first + tail.size());
  } catch (...) {
    if (new_pose) tail_poses_.pop_back();
    throw;
  }
  for (std::size_t i = 0; i < tail.size(); ++i) {
    auto& entry = entries_[first + i];
    entry.membership = std::move(tail[i]);
  }
  frontier_ = frontier;
  if (new_keyframe) UpdateNewKeyframe(*new_keyframe);
  for (auto i = first; i < entries_.size(); ++i) {
    if (entries_[i].membership.state == OnlineSubmapState::kSealed)
      entries_[i].builder.reset();
  }
  stats_.last_update_seconds = Seconds(start);
}

void GeometryMapper::UpdateNewKeyframe(const Keyframe& keyframe) {
  const auto id = entries_.size() - 1;
  const auto& entry = entries_[id];
  const auto members = entry.membership.primary.keyframe_ids.size();
  if (entry.data && entry.built_sources.size() == members) return;
  if (entry.builder && entry.data &&
      entry.built_sources.size() + 1 == members) {
    Append(id, keyframe);
  } else {
    Build(id, &keyframe);
  }
}

void GeometryMapper::ReleaseData(std::uint64_t id) {
  auto& entry = entries_[id];
  if (entry.data) {
    resident_ids_.erase(
        std::remove(resident_ids_.begin(), resident_ids_.end(), id),
        resident_ids_.end());
  }
  entry.data.reset();
  entry.builder.reset();
}

void GeometryMapper::EvictFor(std::uint64_t id) {
  if (entries_[id].data) return;
  while (resident_ids_.size() >=
         static_cast<std::size_t>(options_.max_resident_submaps)) {
    std::size_t oldest = entries_.size();
    for (const auto i : resident_ids_) {
      if (entries_[i].membership.state == OnlineSubmapState::kActive) continue;
      if (oldest == entries_.size() ||
          entries_[i].last_access < entries_[oldest].last_access)
        oldest = i;
    }
    if (oldest == entries_.size())
      throw std::runtime_error("No geometry cache slot");
    ReleaseData(oldest);
    ++stats_.evictions;
  }
}

void GeometryMapper::Build(std::uint64_t id, const Keyframe* new_keyframe) {
  auto& entry = entries_.at(id);
  const auto start = Clock::now();
  auto builder = std::make_unique<GeometrySubmapBuilder>(options_);
  for (const auto frame_id : entry.membership.primary.keyframe_ids) {
    PointCloud::ConstPtr cloud;
    if (new_keyframe && new_keyframe->id == frame_id) {
      cloud = new_keyframe->cloud;
    } else {
      const auto read_start = Clock::now();
      ++stats_.cloud_reads;
      try {
        cloud = cloud_reader_(frame_id);
      } catch (...) {
        stats_.last_cloud_read_seconds += Seconds(read_start);
        throw;
      }
      stats_.last_cloud_read_seconds += Seconds(read_start);
    }
    if (!cloud) throw std::runtime_error("Missing geometry body cloud");
    // This local builder is not observable until the complete snapshot is
    // installed below. A failed frame discards the whole partial build.
    builder->AddKeyframeInPlace(
        frame_id, *cloud, SubmapFromBody(entry.membership, frame_id));
  }
  const auto snapshot_start = Clock::now();
  auto data = builder->Snapshot();
  stats_.last_snapshot_seconds += Seconds(snapshot_start);
  auto sources = data->sources;
  if (entry.representation_revision ==
      std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("Geometry representation revision exhausted");
  }
  EvictFor(id);
  if (!entry.data) resident_ids_.push_back(id);
  entry.data = std::move(data);
  entry.built_sources.swap(sources);
  ++entry.representation_revision;
  entry.last_access = ++access_clock_;
  if (entry.membership.state == OnlineSubmapState::kActive)
    entry.builder = std::move(builder);
  ++stats_.full_builds;
  stats_.last_full_build_seconds += Seconds(start);
}

void GeometryMapper::Append(std::uint64_t id, const Keyframe& keyframe) {
  auto& entry = entries_.at(id);
  try {
    entry.builder->AddKeyframe(keyframe.id, *keyframe.cloud,
                               SubmapFromBody(entry.membership, keyframe.id));
    const auto start = Clock::now();
    auto data = entry.builder->Snapshot();
    stats_.last_snapshot_seconds += Seconds(start);
    auto sources = data->sources;
    if (entry.representation_revision ==
        std::numeric_limits<std::uint64_t>::max()) {
      throw std::overflow_error("Geometry representation revision exhausted");
    }
    entry.data = std::move(data);
    entry.built_sources.swap(sources);
    ++entry.representation_revision;
    entry.last_access = ++access_clock_;
    ++stats_.incremental_updates;
  } catch (...) {
    ReleaseData(id);
    throw;
  }
}

GeometrySubmap GeometryMapper::GetSubmap(std::uint64_t id) {
  ResetTimings();
  if (id >= entries_.size())
    throw std::out_of_range("Unknown geometry submap ID");
  auto& entry = entries_[id];
  if (!entry.data) Build(id, nullptr);
  entry.last_access = ++access_clock_;
  return {entry.membership,
          frontier_.version,
          frontier_.frame_count,
          frontier_.optimized_frame_count,
          frontier_.parent_frame,
          PoseAt(entry.membership.primary.anchor_keyframe_id),
          entry.representation_revision,
          entry.data};
}

GeometryCacheStats GeometryMapper::GetStats() const {
  auto result = stats_;
  result.resident_submaps = resident_ids_.size();
  result.resident_voxels = 0;
  for (const auto id : resident_ids_) {
    result.resident_voxels += entries_[id].data->voxels.size();
  }
  return result;
}

}  // namespace livo_atlas
