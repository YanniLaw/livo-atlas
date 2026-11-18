#include "livo_atlas/gaussian/gaussian_render.h"

#include <algorithm>
#include <stdexcept>

#include "gaussian/render_validation.h"
#include "mapping/pose_math.h"

namespace livo_atlas {
namespace {

void Require(bool condition, const char* message) {
  if (!condition) throw std::invalid_argument(message);
}

bool SameGroup(const OnlineSubmap& a, const OnlineSubmap& b) {
  return a.primary.id == b.primary.id &&
         a.primary.anchor_keyframe_id == b.primary.anchor_keyframe_id &&
         a.primary.keyframe_ids == b.primary.keyframe_ids &&
         a.support_keyframe_ids == b.support_keyframe_ids &&
         a.membership_revision == b.membership_revision && a.state == b.state;
}

bool CanonicalCurrent(const GaussianTarget& source, const PoseSnapshot& poses) {
  try {
    ValidateGaussianTarget(source);
  } catch (const std::invalid_argument&) {
    return false;
  }
  if (source.frame_count > poses.frame_count ||
      source.pose_version > poses.version ||
      source.group.primary.anchor_keyframe_id >= poses.poses.size())
    return false;
  const auto inverse = pose_math::Inverse(
      poses.poses[source.group.primary.anchor_keyframe_id].map_from_body);
  for (std::size_t i = 0; i < source.submap_from_body.size(); ++i) {
    const auto id = source.group.primary.keyframe_ids[i];
    if (id >= poses.poses.size() ||
        !pose_math::Equal(
            source.submap_from_body[i],
            pose_math::Compose(inverse, poses.poses[id].map_from_body)))
      return false;
  }
  return true;
}

bool Usable(const GaussianSubmapEntry& entry, const PoseSnapshot& poses) {
  if (!entry.content ||
      entry.availability != GaussianAvailability::kCurrent ||
      entry.frame_ids.empty() || !CanonicalCurrent(entry.content_target, poses))
    return false;
  if (entry.resident) return true;
  return !entry.dirty && entry.asset && entry.asset_current &&
         entry.asset->content == *entry.content &&
         entry.asset_frame_ids == entry.frame_ids &&
         entry.asset_gaussian_count == entry.gaussian_count &&
         CanonicalCurrent(entry.asset_source, poses);
}

GaussianRenderBlock MakeBlock(const GaussianSubmapEntry& entry,
                              const PoseSnapshot& poses) {
  const auto anchor = entry.group.primary.anchor_keyframe_id;
  Require(SameGroup(entry.group, entry.binding.group) &&
              CanonicalCurrent(entry.binding, poses) &&
              pose_math::Equal(entry.binding.parent_from_submap,
                                poses.poses.at(anchor).map_from_body),
          "Render binding differs from the captured poses");
  GaussianRenderBlock result;
  result.target = entry.binding;
  result.content = *entry.content;
  result.resident = entry.resident;
  if (!result.resident) result.asset = entry.asset;
  result.frame_ids = entry.frame_ids;
  result.gaussian_count = entry.gaussian_count;
  return result;
}

bool SameTarget(const GaussianTarget& a, const GaussianTarget& b) {
  if (a.session_id != b.session_id || a.generation != b.generation ||
      a.pose_version != b.pose_version || a.frame_count != b.frame_count ||
      a.optimized_frame_count != b.optimized_frame_count ||
      a.parent_frame != b.parent_frame || !SameGroup(a.group, b.group) ||
      !pose_math::Equal(a.parent_from_submap, b.parent_from_submap) ||
      a.submap_from_body.size() != b.submap_from_body.size())
    return false;
  for (std::size_t i = 0; i < a.submap_from_body.size(); ++i)
    if (!pose_math::Equal(a.submap_from_body[i], b.submap_from_body[i]))
      return false;
  return true;
}

bool SameAsset(const std::optional<GaussianAssetReference>& a,
               const std::optional<GaussianAssetReference>& b) {
  if (a.has_value() != b.has_value()) return false;
  return !a || (a->asset_id == b->asset_id && a->content == b->content &&
                a->manifest_sha256 == b->manifest_sha256 &&
                a->manifest_bytes == b->manifest_bytes &&
                a->gaussian_bytes == b->gaussian_bytes &&
                a->observation_bytes == b->observation_bytes);
}

}  // namespace

void ValidateGaussianRenderRequest(const GaussianRenderRequest& request) {
  Require(request.map_sequence > 0 && request.generation > 0 &&
              request.frame_count <= 512 &&
              request.query_keyframe_id < request.frame_count &&
              !request.blocks.empty() && request.blocks.size() <= 2,
          "Render request exceeds its frontier or block budget");
  ValidateRenderPose(request.parent_from_body);
  const GaussianRenderBlock* owner = nullptr;
  int total = 0;
  int residents = 0;
  std::optional<std::uint64_t> previous;
  for (const auto& block : request.blocks) {
    const auto& target = block.target;
    ValidateGaussianTarget(target);
    ValidateRenderPose(target.parent_from_submap);
    for (const auto& pose : target.submap_from_body) ValidateRenderPose(pose);
    const auto id = target.group.primary.id;
    Require((!previous || id > *previous) &&
                (id == request.query_submap_id ||
                 (request.query_submap_id > 0 &&
                  id == request.query_submap_id - 1)) &&
                target.generation > 0 &&
                target.session_id == request.session_id &&
                target.pose_version == request.pose_version &&
                target.frame_count == request.frame_count &&
                target.optimized_frame_count == request.optimized_frame_count &&
                target.parent_frame == request.parent_frame &&
                block.content.session_id == request.session_id &&
                block.content.submap_id == id &&
                block.content.source_generation > 0 &&
                block.content.content_job_id > 0,
            "Render source identity or target differs");
    const auto& frames = block.frame_ids;
    Require(!frames.empty() && frames.size() <= 10 &&
                std::is_sorted(frames.begin(), frames.end()) &&
                std::adjacent_find(frames.begin(), frames.end()) ==
                    frames.end() &&
                frames.front() >= target.group.primary.anchor_keyframe_id &&
                frames.back() <= target.group.primary.keyframe_ids.back() &&
                block.gaussian_count >= static_cast<int>(frames.size()) &&
                block.gaussian_count <= 10000 &&
                block.gaussian_count <= static_cast<int>(frames.size()) * 1000,
            "Render coverage exceeds its source or Gaussian budget");
    if (block.resident) {
      Require(!block.asset, "Resident render source cannot name an asset");
      ++residents;
    } else {
      Require(block.asset && block.asset->content == block.content,
              "Asset render source differs from its confirmation");
      ValidateGaussianAssetReference(*block.asset);
    }
    if (id == request.query_submap_id) owner = &block;
    previous = id;
    total += block.gaussian_count;
  }
  Require(owner && residents <= 1 && total <= 20000 &&
              owner->frame_ids.back() == request.query_keyframe_id,
          "Query is not the last retained owner camera");
  const auto offset = request.query_keyframe_id -
                      owner->target.group.primary.anchor_keyframe_id;
  Require(pose_math::Equal(request.parent_from_body,
              pose_math::Compose(owner->target.parent_from_submap,
                                 owner->target.submap_from_body.at(offset))),
          "Query body pose differs from the captured target");
  Require(request.neighbor_reason ==
              (request.blocks.size() == 2 ? "" : request.query_submap_id
                  ? "neighbor_unavailable" : "no_previous_group"),
          "Missing or contradictory render coverage reason");
}

std::optional<GaussianRenderRequest> SelectGaussianRender(
    const GaussianMapSnapshot& map, const PoseSnapshot& poses,
    std::uint64_t generation, std::string* reason) {
  if (reason) reason->clear();
  ValidatePoseSnapshot(poses);
  Require(map.entries.size() <= 512 && map.frame_count <= 512 &&
              map.pose_version == poses.version &&
              map.frame_count == poses.frame_count &&
              map.optimized_frame_count == poses.optimized_frame_count &&
              map.parent_frame == poses.parent_frame,
          "Render map and poses must share one frontier");
  const GaussianSubmapEntry* newest = nullptr;
  for (std::size_t i = 0; i < map.entries.size(); ++i) {
    const auto& entry = map.entries[i];
    Require(entry.group.primary.id == i, "Render groups must be ordered");
    if (!entry.content) continue;
    Require(!entry.frame_ids.empty(),
            "Confirmed content has no retained query");
    if (!newest || entry.frame_ids.back() > newest->frame_ids.back())
      newest = &entry;
  }
  if (!newest || !Usable(*newest, poses)) {
    if (reason) *reason = newest ? "query_unavailable" : "no_confirmation";
    return std::nullopt;
  }
  GaussianRenderRequest result;
  result.session_id = map.session_id;
  result.generation = generation;
  result.map_sequence = map.map_sequence;
  result.pose_version = map.pose_version;
  result.frame_count = map.frame_count;
  result.optimized_frame_count = map.optimized_frame_count;
  result.parent_frame = map.parent_frame;
  result.query_submap_id = newest->group.primary.id;
  result.query_keyframe_id = newest->frame_ids.back();
  result.parent_from_body =
      poses.poses.at(result.query_keyframe_id).map_from_body;
  if (result.query_submap_id > 0) {
    const auto& neighbor = map.entries[result.query_submap_id - 1];
    if (Usable(neighbor, poses)) {
      result.blocks.push_back(MakeBlock(neighbor, poses));
    } else {
      result.neighbor_reason = "neighbor_unavailable";
    }
  } else {
    result.neighbor_reason = "no_previous_group";
  }
  result.blocks.push_back(MakeBlock(*newest, poses));
  ValidateGaussianRenderRequest(result);
  return result;
}

bool IsCurrentGaussianRender(const GaussianRenderRequest& request,
                             const GaussianMapSnapshot& map,
                             const PoseSnapshot& poses) {
  if (request.session_id != map.session_id ||
      request.map_sequence != map.map_sequence)
    return false;
  try {
    ValidateGaussianRenderRequest(request);
    const auto current =
        SelectGaussianRender(map, poses, request.generation, nullptr);
    if (!current || current->query_keyframe_id != request.query_keyframe_id ||
        current->query_submap_id != request.query_submap_id ||
        current->neighbor_reason != request.neighbor_reason ||
        !pose_math::Equal(current->parent_from_body,
                          request.parent_from_body) ||
        current->blocks.size() != request.blocks.size())
      return false;
    for (std::size_t i = 0; i < request.blocks.size(); ++i) {
      const auto& a = request.blocks[i];
      const auto& b = current->blocks[i];
      if (a.resident != b.resident || a.content != b.content ||
          a.frame_ids != b.frame_ids || a.gaussian_count != b.gaussian_count ||
          !SameAsset(a.asset, b.asset) || !SameTarget(a.target, b.target))
        return false;
    }
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

}  // namespace livo_atlas
