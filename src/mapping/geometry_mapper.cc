#include "livo_atlas/mapping/geometry_mapper.h"

#include <algorithm>
#include <chrono>
#include <limits>
#include <stdexcept>
#include <utility>

#include "pose_math.h"

namespace livo_atlas {
namespace {

using Clock = std::chrono::steady_clock;

double Seconds(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

bool SameMembership(const OnlineSubmap& a, const OnlineSubmap& b) {
  return a.primary.id == b.primary.id &&
         a.primary.anchor_keyframe_id == b.primary.anchor_keyframe_id &&
         a.primary.keyframe_ids == b.primary.keyframe_ids &&
         a.support_keyframe_ids == b.support_keyframe_ids &&
         a.membership_revision == b.membership_revision && a.state == b.state;
}

void ValidateRevision(const OnlineSubmap& group, const PoseSnapshot& poses) {
  ValidateSubmap(group.primary);
  const auto& members = group.primary.keyframe_ids;
  const bool sealed = group.state == OnlineSubmapState::kSealed;
  if ((group.state != OnlineSubmapState::kActive && !sealed) ||
      members.size() > kMaxGeometryKeyframes ||
      group.membership_revision != members.size() + (sealed ? 1 : 0) ||
      members.back() >= poses.frame_count) {
    throw std::invalid_argument("Invalid geometry submap revision");
  }
}

void ValidateGroups(const std::vector<OnlineSubmap>& groups,
                    const PoseSnapshot& poses) {
  std::uint64_t next_frame = 0;
  for (std::size_t i = 0; i < groups.size(); ++i) {
    const auto& group = groups[i];
    ValidateRevision(group, poses);
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

RigidTransform LocalPose(const OnlineSubmap& group, const PoseSnapshot& poses,
                         std::uint64_t id) {
  return pose_math::Compose(
      pose_math::Inverse(
          poses.poses[group.primary.anchor_keyframe_id].map_from_body),
      poses.poses[id].map_from_body);
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

void GeometryMapper::Update(const std::vector<OnlineSubmap>& revisions,
                            std::shared_ptr<const PoseSnapshot> poses,
                            const Keyframe* new_keyframe) {
  const auto start = Clock::now();
  ResetTimings();
  if (!poses) throw std::invalid_argument("Missing geometry pose snapshot");
  ValidatePoseSnapshot(*poses);
  if (poses_) {
    if (poses->version < poses_->version ||
        poses->frame_count < poses_->frame_count ||
        poses->optimized_frame_count < poses_->optimized_frame_count) {
      throw std::invalid_argument("Geometry pose snapshot moved backwards");
    }
    if (poses->version == poses_->version) {
      if (poses->optimized_frame_count != poses_->optimized_frame_count ||
          poses->parent_frame != poses_->parent_frame ||
          poses->tail_policy != poses_->tail_policy) {
        throw std::invalid_argument("Geometry pose version changed in place");
      }
      for (std::size_t i = 0; i < poses_->poses.size(); ++i) {
        if (!SamePoseExactly(poses_->poses[i].map_from_body,
                             poses->poses[i].map_from_body)) {
          throw std::invalid_argument("Geometry pose version changed in place");
        }
      }
    }
  }

  std::vector<OnlineSubmap> target;
  target.reserve(entries_.size() + revisions.size());
  for (const auto& entry : entries_) target.push_back(entry.membership);
  for (const auto& revision : revisions) {
    ValidateRevision(revision, *poses);
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
    if (changed || (entry.data &&
                    entry.built_sources.size() !=
                        entry.membership.primary.keyframe_ids.size() &&
                    !can_append)) {
      entry.data.reset();
      entry.builder.reset();
    } else if (entry.data && poses_ && poses->version != poses_->version) {
      ++stats_.rigid_updates;
    }
  }
  poses_ = std::move(poses);
  if (new_keyframe) {
    const auto id = entries_.size() - 1;
    auto& entry = entries_[id];
    if (entry.built_sources.size() !=
            entry.membership.primary.keyframe_ids.size() ||
        !entry.data) {
      if (entry.builder && entry.data &&
          entry.built_sources.size() + 1 ==
              entry.membership.primary.keyframe_ids.size()) {
        Append(id, *new_keyframe);
      } else {
        Build(id, new_keyframe);
      }
    }
  }
  for (auto& entry : entries_) {
    if (entry.membership.state == OnlineSubmapState::kSealed)
      entry.builder.reset();
  }
  stats_.last_update_seconds = Seconds(start);
}

void GeometryMapper::EvictFor(std::uint64_t id) {
  if (entries_[id].data) return;
  while (GetStats().resident_submaps >=
         static_cast<std::uint64_t>(options_.max_resident_submaps)) {
    std::size_t oldest = entries_.size();
    for (std::size_t i = 0; i < entries_.size(); ++i) {
      if (i == id || !entries_[i].data ||
          entries_[i].membership.state == OnlineSubmapState::kActive)
        continue;
      if (oldest == entries_.size() ||
          entries_[i].last_access < entries_[oldest].last_access)
        oldest = i;
    }
    if (oldest == entries_.size())
      throw std::runtime_error("No geometry cache slot");
    entries_[oldest].data.reset();
    entries_[oldest].builder.reset();
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
    builder->AddKeyframe(frame_id, *cloud,
                         LocalPose(entry.membership, *poses_, frame_id));
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
    entry.builder->AddKeyframe(
        keyframe.id, *keyframe.cloud,
        LocalPose(entry.membership, *poses_, keyframe.id));
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
    entry.data.reset();
    entry.builder.reset();
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
  return {
      entry.membership,
      poses_->version,
      poses_->frame_count,
      poses_->optimized_frame_count,
      poses_->parent_frame,
      poses_->poses[entry.membership.primary.anchor_keyframe_id].map_from_body,
      entry.representation_revision,
      entry.data};
}

GeometryCacheStats GeometryMapper::GetStats() const {
  auto result = stats_;
  result.resident_submaps = result.resident_voxels = 0;
  for (const auto& entry : entries_) {
    if (!entry.data) continue;
    ++result.resident_submaps;
    result.resident_voxels += entry.data->voxels.size();
  }
  return result;
}

}  // namespace livo_atlas
