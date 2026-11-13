#include "livo_atlas/backend/loop_task.h"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include "livo_atlas/mapping/geometry_mapper.h"

namespace livo_atlas {
namespace {

void ValidateTask(const LoopTask& task) {
  const auto count = task.input.poses.frame_count;
  if (task.frames.frame_count() != count ||
      task.input.raw_poses.size() != count || task.timestamps.size() != count)
    throw std::invalid_argument("Loop task has inconsistent frame frontiers");
  for (std::size_t i = 0; i < task.timestamps.size(); ++i) {
    if (!std::isfinite(task.timestamps[i]) ||
        (i && task.timestamps[i] < task.timestamps[i - 1]))
      throw std::invalid_argument("Loop task timestamps are not ordered");
  }
  for (std::size_t i = 0; i < task.input.submaps.size(); ++i) {
    const auto& info = task.input.submaps[i];
    const auto& group = info.membership;
    if (group.primary.id != i || group.primary.keyframe_ids.empty() ||
        group.primary.keyframe_ids.front() >= count ||
        group.primary.keyframe_ids.back() >= count ||
        group.state != OnlineSubmapState::kSealed)
      throw std::invalid_argument("Loop task requires a sealed submap prefix");
    if (info.first_timestamp !=
            task.timestamps[group.primary.keyframe_ids.front()] ||
        info.last_timestamp !=
            task.timestamps[group.primary.keyframe_ids.back()])
      throw std::invalid_argument("Loop task submap timestamps changed");
  }
}

}  // namespace

LoopAdmissionResult RunLoopTask(const LoopTask& task,
                                const LoopAdmissionOptions& options) {
  ValidateLoopAdmissionOptions(options);
  ValidateTask(task);
  GeometryOptions geometry;
  geometry.enabled = true;
  std::unique_ptr<GeometryMapper> mapper;
  std::shared_ptr<const PoseSnapshot> view;
  const LoopGeometryReader reader = [&](auto id, const auto& poses) {
    if (!view || view->version != poses.version) {
      mapper.reset();
      view = std::make_shared<const PoseSnapshot>(poses);
      mapper = std::make_unique<GeometryMapper>(geometry, [&](std::uint64_t
                                                                  frame_id) {
        const auto raw = task.frames.ReadRawBodyPose(frame_id);
        const auto& expected = task.input.raw_poses.at(frame_id);
        if ((raw.rotation.array() != expected.rotation.array()).any() ||
            (raw.translation.array() != expected.translation.array()).any() ||
            task.frames.ReadTimestamp(frame_id) != task.timestamps.at(frame_id))
          throw std::invalid_argument("Committed loop source changed");
        return task.frames.ReadBodyCloud(frame_id,
                                         geometry.max_points_per_keyframe);
      });
      std::vector<OnlineSubmap> groups;
      groups.reserve(task.input.submaps.size());
      for (const auto& info : task.input.submaps)
        groups.push_back(info.membership);
      mapper->Update(groups, view);
    }
    return mapper->GetSubmap(id);
  };
  return ProposeLoopUpdate(task.input, reader, options);
}

}  // namespace livo_atlas
