#include "io/loop_proposer.h"

#include <algorithm>
#include <chrono>
#include <exception>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "io/dataset_metadata.h"
#include "io/loop_admission_io.h"
#include "livo_atlas/io/online_submap_store.h"
#include "livo_atlas/mapping/geometry_mapper.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;
using dataset_metadata::Decimal;
using dataset_metadata::WriteTransform;
using dataset_metadata::WriteYaml;

bool Within(const fs::path& child, const fs::path& parent) {
  auto a = child.begin();
  for (auto b = parent.begin(); b != parent.end(); ++b, ++a) {
    if (a == child.end() || *a != *b) return false;
  }
  return true;
}

}  // namespace

void ProposeLoopDataset(const fs::path& dataset, const fs::path& output,
                        const LoopAdmissionOptions& options) {
  ValidateLoopAdmissionOptions(options);
  const auto input_path = fs::canonical(dataset);
  if (output.empty() || Within(fs::weakly_canonical(output), input_path)) {
    throw std::invalid_argument(
        "Loop output must be outside the input dataset");
  }
  if (!output.parent_path().empty())
    fs::create_directories(output.parent_path());
  if (!fs::create_directory(output))
    throw std::runtime_error("Loop output already exists");
  LoopAdmissionInput input;
  std::size_t queries = 0, proposals = 0, frame_count = 0, submap_count = 0;
  std::map<std::string, std::size_t> counts;
  GeometryOptions geometry;
  geometry.enabled = true;
  const auto summary = [&](bool complete) {
    // Keep the previous incomplete summary if the final write cannot finish.
    const auto pending = output / ".summary.pending.yml";
    WriteYaml(pending, [&](auto& yaml) {
      yaml << "schema_version" << 1 << "complete" << static_cast<int>(complete)
           << "input" << input_path.string() << "frontier"
           << "closed_dataset"
           << "frame_count" << std::to_string(frame_count) << "sealed_submaps"
           << std::to_string(submap_count) << "query_count"
           << std::to_string(queries) << "proposal_count"
           << std::to_string(proposals) << "pose_version"
           << std::to_string(input.poses.version);
      loop_admission_io::WriteOptions(yaml, options, geometry);
      yaml << "statuses"
           << "{";
      for (const auto& count : counts)
        yaml << count.first << std::to_string(count.second);
      yaml << "}";
    });
    fs::rename(pending, output / "summary.yml");
  };
  try {
    summary(false);
    fs::create_directory(output / "queries");
    fs::create_directory(output / "proposals");
    KeyframeStore frames(dataset);
    OnlineSubmapStore store(dataset / "online_submaps", frames);
    frame_count = frames.ids().size();
    submap_count = store.size();
    if (store.indexed_frame_count() != frame_count) {
      throw std::invalid_argument(
          "Loop admission requires a fully indexed dataset");
    }
    std::vector<double> timestamps;
    for (auto id : frames.ids()) {
      const auto timestamp = frames.ReadTimestamp(id);
      if (!timestamps.empty() && timestamp < timestamps.back())
        throw std::invalid_argument("Loop dataset timestamps moved backwards");
      timestamps.push_back(timestamp);
    }
    input.poses = store.MaterializePoseSnapshot(0, frame_count);
    for (const auto& pose : input.poses.poses)
      input.raw_poses.push_back(pose.map_from_body);
    std::vector<OnlineSubmap> groups;
    for (std::size_t id = 0; id < submap_count; ++id) {
      const auto group = store.Read(id);
      if (group.state != OnlineSubmapState::kSealed)
        throw std::invalid_argument("Loop dataset still has an active submap");
      groups.push_back(group);
      input.submaps.push_back(
          {group, timestamps.at(group.primary.keyframe_ids.front()),
           timestamps.at(group.primary.keyframe_ids.back()),
           input.poses.poses.at(group.primary.anchor_keyframe_id).map_from_body,
           0, frame_count, 0});
    }
    std::unique_ptr<GeometryMapper> mapper;
    std::shared_ptr<const PoseSnapshot> view;
    const LoopGeometryReader reader = [&](auto id, const auto& poses) {
      // Every trial first reads current geometry, then preview geometry. Reset
      // on a view change so rejected previews cannot poison the next query.
      // A single three-submap cache is alive at a time.
      if (!view || view->version != poses.version) {
        mapper.reset();
        view = std::make_shared<const PoseSnapshot>(poses);
        mapper = std::make_unique<GeometryMapper>(
            geometry, [&](std::uint64_t frame_id) {
              return frames.ReadBodyCloud(frame_id,
                                          geometry.max_points_per_keyframe);
            });
        mapper->Update(groups, view);
      }
      return mapper->GetSubmap(id);
    };
    for (const auto& source : input.submaps) {
      input.query_id = source.membership.primary.id;
      const auto start = std::chrono::steady_clock::now();
      const auto result = ProposeLoopUpdate(input, reader, options);
      const double elapsed = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - start)
                                 .count();
      std::optional<PoseSnapshot> next_poses;
      std::vector<AcceptedLoop> next_loops;
      if (result.proposal) {
        next_poses = MakePoseSnapshot(result.proposal->update, input.poses,
                                      input.raw_poses);
        next_loops = input.accepted_loops;
        next_loops.push_back(result.proposal->loop);
        WriteYaml(output / "proposals" /
                      (std::to_string(next_poses->version) + ".yml"),
                  [&](auto& yaml) {
                    loop_admission_io::WriteProposal(yaml, input, result);
                  });
      }
      WriteYaml(output / "queries" / (std::to_string(input.query_id) + ".yml"),
                [&](auto& yaml) {
                  loop_admission_io::WriteQuery(yaml, source, input, result,
                                                elapsed);
                });
      // All allocations and writes succeed before the two no-throw swaps.
      if (next_poses) {
        std::swap(input.poses, *next_poses);
        input.accepted_loops.swap(next_loops);
        ++proposals;
        for (auto& info : input.submaps) {
          info.parent_from_submap =
              input.poses.poses[info.membership.primary.anchor_keyframe_id]
                  .map_from_body;
          info.pose_version = input.poses.version;
          info.optimized_frame_count = input.poses.optimized_frame_count;
        }
      }
      ++queries;
      ++counts[LoopAdmissionStatusName(result.status)];
    }
    summary(true);
  } catch (...) {
    const auto failure = std::current_exception();
    // Reporting may itself fail (for example, a full disk). Preserve the
    // original exception and every record already written.
    try {
      std::string message = "Unknown exception";
      try {
        std::rethrow_exception(failure);
      } catch (const std::exception& error) {
        message = error.what();
      } catch (...) {
      }
      summary(false);
      WriteYaml(output / "failure.yml",
                [&](auto& yaml) { yaml << "error" << message; });
    } catch (...) {
    }
    std::rethrow_exception(failure);
  }
}

}  // namespace livo_atlas
