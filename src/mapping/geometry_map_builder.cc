#include "livo_atlas/mapping/geometry_map_builder.h"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

#include "geometry_map_sampling.h"
#include "geometry_validation.h"
#include "pose_math.h"

namespace livo_atlas {
namespace {

RigidTransform LocalPose(const OnlineSubmap& group, const PoseSnapshot& poses,
                         std::uint64_t id) {
  return pose_math::Compose(
      pose_math::Inverse(
          poses.poses[group.primary.anchor_keyframe_id].parent_from_body()),
      poses.poses[id].parent_from_body());
}

}  // namespace

GeometryMapBuilder::GeometryMapBuilder(GeometryOptions geometry,
                                       GeometryMapOptions options,
                                       GeometryMapper::CloudReader reader,
                                       std::function<bool()> cancelled)
    : geometry_options_(std::move(geometry)),
      options_(std::move(options)),
      cancelled_(std::move(cancelled)),
      mapper_(geometry_options_, std::move(reader)) {
  ValidateGeometryMapOptions(options_);
}

bool GeometryMapBuilder::Matches(const Fragment& fragment,
                                 const OnlineSubmap& group,
                                 const PoseSnapshot& poses) const {
  const auto& ids = group.primary.keyframe_ids;
  if (fragment.sources.size() != ids.size()) return false;
  for (std::size_t i = 0; i < ids.size(); ++i) {
    const auto& source = fragment.sources[i];
    if (source.keyframe_id != ids[i] ||
        !pose_math::Equal(source.submap_from_body,
                          LocalPose(group, poses, ids[i])))
      return false;
  }
  return true;
}

void GeometryMapBuilder::EraseFragment(std::size_t id) {
  auto& entry = entries_.at(id);
  if (!entry.fragment) return;
  stats_.cached_points -= entry.fragment->points.size();
  entry.fragment.reset();
}

void GeometryMapBuilder::EvictFor(std::size_t points) {
  const auto limit = static_cast<std::uint64_t>(options_.max_cached_points);
  while (stats_.cached_points + points > limit) {
    std::size_t oldest = entries_.size();
    for (std::size_t i = 0; i < entries_.size(); ++i) {
      if (entries_[i].fragment &&
          (oldest == entries_.size() ||
           entries_[i].last_access < entries_[oldest].last_access))
        oldest = i;
    }
    if (oldest == entries_.size())
      throw std::logic_error("No compact geometry cache entry to evict");
    EraseFragment(oldest);
    ++stats_.cache_evictions;
  }
}

void GeometryMapBuilder::Touch(std::size_t id) {
  if (access_clock_ == std::numeric_limits<std::uint64_t>::max())
    throw std::overflow_error("Geometry map cache access counter exhausted");
  entries_.at(id).last_access = ++access_clock_;
}

GeometryMapSnapshot GeometryMapBuilder::Build(
    const std::vector<OnlineSubmap>& groups,
    std::shared_ptr<const PoseSnapshot> poses) {
  const auto check_cancel = [&] {
    if (cancelled_ && cancelled_())
      throw std::runtime_error("Geometry map build cancelled");
  };
  check_cancel();
  if (!poses) throw std::invalid_argument("Missing geometry map poses");
  ValidatePoseSnapshot(*poses);
  if (poses->version &&
      poses->tail_policy != PoseTailPolicy::kProjectedLastOptimizedAnchor)
    throw std::invalid_argument(
        "Live geometry map requires projected pose tail");
  geometry_detail::ValidateGroups(groups, *poses);
  if ((groups.empty() && poses->frame_count != 0) ||
      (!groups.empty() &&
       groups.back().primary.keyframe_ids.back() != poses->frame_count - 1))
    throw std::invalid_argument(
        "Geometry map requires complete primary coverage");

  GeometryMapSnapshot result;
  result.pose_version = poses->version;
  result.frame_count = poses->frame_count;
  result.optimized_frame_count = poses->optimized_frame_count;
  result.submap_count = groups.size();
  result.parent_frame = poses->parent_frame;
  result.geometry_options = geometry_options_;
  result.options = options_;
  result.effective_max_points =
      geometry_map_detail::GeometryMapPointLimit(options_, groups.size());
  if (poses->frame_count && !result.effective_max_points) {
    result.status = GeometryMapStatus::kResourceLimited;
    result.error = "Geometry map exceeds configured message budget";
    return result;
  }

  mapper_.Update(groups, poses);
  entries_.resize(groups.size());
  geometry_map_detail::GeometryMapSampler sampler(result.effective_max_points);
  result.sources.reserve(groups.size());
  for (std::size_t i = 0; i < groups.size(); ++i) {
    check_cancel();
    const auto& group = groups[i];
    auto& entry = entries_[i];
    if (entry.fragment && !Matches(*entry.fragment, group, *poses)) {
      EraseFragment(i);
      ++stats_.cache_invalidations;
    }

    GeometrySubmap geometry;
    if (entry.fragment) {
      ++stats_.cache_hits;
      Touch(i);
    } else {
      geometry = mapper_.GetSubmap(i);
      if (entry.representation_revision != geometry.representation_revision) {
        entry.representation_revision = geometry.representation_revision;
        entry.built_pose_version = poses->version;
      }
      const auto count = geometry.data->voxels.size();
      if (count <= static_cast<std::size_t>(options_.max_cached_points)) {
        EvictFor(count);
        Fragment fragment;
        fragment.sources = geometry.data->sources;
        fragment.representation_revision = geometry.representation_revision;
        fragment.built_pose_version = entry.built_pose_version;
        fragment.input_point_count = geometry.data->input_point_count;
        fragment.points.reserve(count);
        for (const auto& voxel : geometry.data->voxels)
          fragment.points.push_back(
              {voxel.key, voxel.centroid, voxel.mean_intensity});
        entry.fragment = std::move(fragment);
        stats_.cached_points += count;
        stats_.peak_cached_points =
            std::max(stats_.peak_cached_points, stats_.cached_points);
        Touch(i);
      }
    }

    const auto* fragment = entry.fragment ? &*entry.fragment : nullptr;
    const auto count =
        fragment ? fragment->points.size() : geometry.data->voxels.size();
    if (count >
        std::numeric_limits<std::uint64_t>::max() - result.source_voxel_count)
      throw std::overflow_error("Geometry map voxel count overflow");
    result.source_voxel_count += count;
    GeometryMapSourceInfo info;
    info.submap_id = group.primary.id;
    info.anchor_keyframe_id = group.primary.anchor_keyframe_id;
    info.first_keyframe_id = group.primary.keyframe_ids.front();
    info.primary_keyframe_count = group.primary.keyframe_ids.size();
    info.membership_revision = group.membership_revision;
    info.state = group.state;
    info.representation_revision = entry.representation_revision;
    info.built_pose_version = entry.built_pose_version;
    info.source_voxel_count = count;
    info.input_point_count = fragment ? fragment->input_point_count
                                      : geometry.data->input_point_count;
    result.sources.push_back(info);

    const auto& parent =
        poses->poses[group.primary.anchor_keyframe_id].parent_from_body();
    const auto add = [&](const GeometryVoxelKey& key,
                         const Eigen::Vector3d& local, double intensity) {
      sampler.Add({group.primary.id,
                   key,
                   {parent.rotation * local + parent.translation, intensity}});
    };
    std::size_t checked = 0;
    if (fragment) {
      for (const auto& point : fragment->points) {
        if ((checked++ % 1024) == 0) check_cancel();
        add(point.key, point.centroid, point.intensity);
      }
    } else {
      for (const auto& voxel : geometry.data->voxels) {
        if ((checked++ % 1024) == 0) check_cancel();
        add(voxel.key, voxel.centroid, voxel.mean_intensity);
      }
    }
  }

  auto selected = sampler.Take();
  result.sampled = result.source_voxel_count > selected.size();
  result.points.reserve(selected.size());
  std::size_t cursor = 0;
  for (auto& source : result.sources) {
    source.point_offset = result.points.size();
    while (cursor < selected.size() &&
           selected[cursor].submap_id == source.submap_id) {
      result.points.push_back(std::move(selected[cursor++].point));
    }
    source.point_count = result.points.size() - source.point_offset;
  }
  if (cursor != selected.size())
    throw std::logic_error("Sample references an unknown geometry submap");
  return result;
}

GeometryMapBuilderStats GeometryMapBuilder::GetStats() const {
  auto result = stats_;
  result.geometry = mapper_.GetStats();
  return result;
}

}  // namespace livo_atlas
