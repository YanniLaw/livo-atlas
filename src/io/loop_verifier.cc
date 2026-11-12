#include "io/loop_verifier.h"

#include <algorithm>
#include <map>
#include <stdexcept>
#include <string>

#include "io/dataset_metadata.h"
#include "io/loop_diagnostics.h"
#include "io/loop_options.h"
#include "livo_atlas/io/online_submap_store.h"
#include "livo_atlas/mapping/geometry_mapper.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;
using dataset_metadata::Decimal;
using dataset_metadata::WriteYaml;

bool Within(const fs::path& child, const fs::path& parent) {
  auto a = child.begin();
  for (auto b = parent.begin(); b != parent.end(); ++b, ++a) {
    if (a == child.end() || *a != *b) return false;
  }
  return true;
}

}  // namespace

LoopVerifierOptions ReadLoopVerifierOptions(const fs::path& path) {
  const auto yaml = dataset_metadata::ReadYaml(path);
  loop_options::ValidateKeys(yaml.root(), {"search", "registration"});
  LoopVerifierOptions options;
  loop_options::ReadSearch(yaml["search"], options.search);
  loop_options::ReadRegistration(yaml["registration"], options.registration);
  ValidateLoopSearchOptions(options.search);
  ValidateLoopRegistrationOptions(options.registration);
  return options;
}

void VerifyLoopDataset(const fs::path& dataset, const fs::path& output,
                       const LoopVerifierOptions& options) {
  ValidateLoopSearchOptions(options.search);
  ValidateLoopRegistrationOptions(options.registration);
  if (output.empty() ||
      Within(fs::weakly_canonical(output), fs::canonical(dataset))) {
    throw std::invalid_argument(
        "Loop output must be outside the input dataset");
  }
  if (!output.parent_path().empty())
    fs::create_directories(output.parent_path());
  if (!fs::create_directory(output))
    throw std::runtime_error("Loop output already exists");
  std::size_t candidates = 0, verified = 0;
  std::map<std::string, std::size_t> counts;
  std::size_t submap_count = 0, frame_count = 0, max_pinned_voxels = 0;
  GeometryOptions geometry;
  geometry.enabled = true;
  const auto summary = [&](bool complete) {
    WriteYaml(output / "summary.yml", [&](auto& yaml) {
      yaml << "schema_version" << 1 << "complete" << static_cast<int>(complete)
           << "input" << fs::canonical(dataset).string() << "pose_version"
           << "0"
           << "frame_count" << std::to_string(frame_count) << "sealed_submaps"
           << std::to_string(submap_count) << "candidates"
           << std::to_string(candidates) << "verified"
           << std::to_string(verified) << "max_pinned_voxels"
           << std::to_string(max_pinned_voxels) << "geometry_voxel_size_m"
           << Decimal(geometry.voxel_size) << "geometry_min_points"
           << geometry.min_points_per_voxel << "geometry_max_plane_rms"
           << Decimal(geometry.max_plane_rms) << "geometry_max_plane_ratio"
           << Decimal(geometry.max_plane_ratio) << "geometry_cache_submaps"
           << geometry.max_resident_submaps;
      loop_options::Write(yaml, options.search, options.registration);
      yaml << "statuses"
           << "{";
      for (const auto& count : counts)
        yaml << count.first << std::to_string(count.second);
      yaml << "}";
    });
  };
  try {
    summary(false);
    fs::create_directory(output / "matches");
    KeyframeStore frames(dataset);
    OnlineSubmapStore store(dataset / "online_submaps", frames);
    frame_count = frames.ids().size();
    submap_count = store.size();
    if (store.indexed_frame_count() != frame_count) {
      throw std::invalid_argument(
          "Loop verification requires a fully indexed dataset");
    }
    std::vector<double> timestamps;
    for (const auto id : frames.ids()) {
      const auto timestamp = frames.ReadTimestamp(id);
      if (!timestamps.empty() && timestamp < timestamps.back())
        throw std::invalid_argument("Loop dataset timestamps moved backwards");
      timestamps.push_back(timestamp);
    }
    auto poses = std::make_shared<const PoseSnapshot>(
        store.MaterializePoseSnapshot(0, frame_count));
    std::vector<OnlineSubmap> groups;
    std::vector<LoopSubmapInfo> infos;
    for (std::size_t id = 0; id < submap_count; ++id) {
      const auto group = store.Read(id);
      if (group.state != OnlineSubmapState::kSealed)
        throw std::invalid_argument("Loop dataset still has an active submap");
      groups.push_back(group);
      const auto first = group.primary.keyframe_ids.front(),
                 last = group.primary.keyframe_ids.back();
      infos.push_back({group, timestamps.at(first), timestamps.at(last),
                       poses->poses.at(first).map_from_body, 0, frame_count,
                       0});
    }
    GeometryMapper mapper(geometry, [&](std::uint64_t id) {
      return frames.ReadBodyCloud(id, geometry.max_points_per_keyframe);
    });
    mapper.Update(groups, poses);
    std::vector<LoopSubmapInfo> history;
    for (const auto& query : infos) {
      for (const auto& candidate :
           FindLoopCandidates(query, history, options.search)) {
        // References live for this candidate only, including evicted cache
        // data.
        const auto source = mapper.GetSubmap(candidate.source_id);
        const auto target = mapper.GetSubmap(candidate.target_id);
        max_pinned_voxels =
            std::max(max_pinned_voxels,
                     source.data->voxels.size() + target.data->voxels.size());
        const auto result = VerifyLoop(target, source, options.registration);
        WriteYaml(output / "matches" /
                      (std::to_string(candidate.target_id) + "-" +
                       std::to_string(candidate.source_id) + ".yml"),
                  [&](auto& yaml) {
                    loop_diagnostics::WriteMatch(yaml, candidate, result);
                  });
        ++candidates;
        verified += result.status == LoopStatus::kVerified;
        ++counts[LoopStatusName(result.status)];
      }
      history.push_back(query);
    }
    summary(true);
  } catch (const std::exception& error) {
    // Keep already written evidence; never reuse this output for a retry.
    try {
      summary(false);
      WriteYaml(output / "failure.yml",
                [&](auto& yaml) { yaml << "error" << error.what(); });
    } catch (...) {
      // The original I/O failure remains the useful cause if reporting also
      // fails.
    }
    throw;
  }
}

}  // namespace livo_atlas
