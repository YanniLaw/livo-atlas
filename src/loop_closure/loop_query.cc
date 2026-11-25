#include "loop_closure/loop_query.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "common/thread_pool.h"
#include "common/timing.h"
#include "livo_atlas/common/logging.h"
#include "livo_atlas/mapping/geometry_mapper.h"
#include "loop_closure/loop_admission.h"
#include "loop_closure/loop_internal.h"

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

LoopGeometryReader MakeGeometryReader(
    const LoopTask& task, GeometryOptions geometry, bool stage_timing = false) {
  // The reader outlives its preparation task and owns the mutable cache.
  struct Cache {
    std::unique_ptr<GeometryMapper> mapper;
    std::shared_ptr<const PoseSnapshot> view;
  };
  auto cache = std::make_shared<Cache>();
  return [&task, geometry, cache, stage_timing](auto id, const auto& poses) {
    const auto start = StartTiming(stage_timing);
    auto& mapper = cache->mapper;
    auto& view = cache->view;
    const bool reader_reset = !view || view->version != poses.version;
    if (reader_reset) {
      mapper.reset();
      view = std::make_shared<const PoseSnapshot>(poses);
      const auto read_cloud = [&task, geometry](std::uint64_t frame_id) {
        const auto raw = task.frames.ReadRawBodyPose(frame_id);
        const auto& expected = task.input.raw_poses.at(frame_id);
        if ((raw.rotation.array() != expected.rotation.array()).any() ||
            (raw.translation.array() != expected.translation.array()).any() ||
            task.frames.ReadTimestamp(frame_id) !=
                task.timestamps.at(frame_id)) {
          throw std::invalid_argument("Committed loop source changed");
        }
        return task.frames.ReadBodyCloud(frame_id,
                                         geometry.max_points_per_keyframe);
      };
      mapper = std::make_unique<GeometryMapper>(geometry, read_cloud);
      std::vector<OnlineSubmap> groups;
      groups.reserve(task.input.submaps.size());
      for (const auto& info : task.input.submaps)
        groups.push_back(info.membership);
      mapper->Update(groups, view);
    }
    GeometryCacheStats before;
    if (stage_timing) before = mapper->GetStats();
    auto result = mapper->GetSubmap(id);
    if (stage_timing) {
      const auto elapsed = SteadyClock::now() - start;
      const auto stats = mapper->GetStats();
      if (stats.last_full_build_seconds > 0) {
        // Full build includes cloud reads and snapshot construction.
        LogStageSeconds("loop_cloud_read", stats.last_cloud_read_seconds);
        LogStageSeconds("loop_geometry_build", stats.last_full_build_seconds);
        LogStageSeconds("loop_geometry_snapshot", stats.last_snapshot_seconds);
      }
      LogStageTiming("loop_geometry_read", elapsed);
      LOG(INFO) << "livo_atlas loop_geometry_cache submap=" << id
                << " pose_version=" << poses.version
                << " reader_reset=" << reader_reset
                << " built=" << (stats.full_builds > before.full_builds)
                << " cloud_reads=" << (stats.cloud_reads - before.cloud_reads)
                << " evictions=" << (stats.evictions - before.evictions)
                << " resident_submaps=" << stats.resident_submaps;
    }
    return result;
  };
}

struct LoopQueryState {
  std::shared_ptr<LoopTaskResult> payload;
  LoopAdmissionOptions options;
  LoopAdmissionResult admission;
  bool stage_timing = false;
  SteadyClock::time_point compute_start, verification_start;
  LoopGeometryReader reader;
  std::optional<GeometrySubmap> source;
  std::vector<std::optional<GeometrySubmap>> targets;
  std::vector<std::exception_ptr> errors;
  // Private to this dependency graph; submission failure is reported through
  // payload->error without racing already submitted preparation/verification.
  std::exception_ptr error;
  std::function<void(std::shared_ptr<LoopTaskResult>)> completion;
};

void PrepareBatch(LoopQueryState& query, std::size_t begin, std::size_t end) {
  if (query.error) return;
  const auto start = StartTiming(query.stage_timing);
  const auto& input = query.payload->task.input;
  const auto& source = input.submaps.at(input.query_id).membership;
  std::size_t index = begin;
  try {
    if (!query.source) {
      query.source = loop_internal::ReadLoopGeometry(
          source, input.poses, query.reader);
    }
    for (; index < end; ++index) {
      // Refresh the pinned source before loading each target. The cache can
      // then evict the previous batch without exceeding its resident budget.
      query.reader(input.query_id, input.poses);
      const auto id = query.admission.candidates[index].candidate.target_id;
      query.targets[index] = loop_internal::ReadLoopGeometry(
          input.submaps.at(id).membership, input.poses, query.reader);
    }
  } catch (...) {
    query.errors[index] = std::current_exception();
  }
  LogStageTiming("loop_prepare_batch", start);
  query.verification_start = StartTiming(query.stage_timing);
}

void VerifyCandidate(LoopQueryState& query, std::size_t index) {
  if (query.error || !query.targets[index]) return;
  const auto start = StartTiming(query.stage_timing);
  try {
    query.admission.candidates[index].verification = loop_internal::VerifyLoop(
        *query.targets[index], *query.source, query.options.registration,
        query.stage_timing);
  } catch (...) {
    query.errors[index] = std::current_exception();
  }
  LogStageTiming("loop_candidate", start);
}

void MergeBatch(LoopQueryState& query, std::size_t begin, std::size_t end) {
  const auto start =
      std::exchange(query.verification_start, SteadyClock::time_point{});
  LogStageTiming("loop_verification_batch", start);
  for (std::size_t index = begin; index < end; ++index) {
    // Completion order cannot change which candidate error reaches the owner.
    if (!query.error && query.errors[index])
      query.error = query.errors[index];
    query.targets[index].reset();
  }
}

void FinishQuery(LoopQueryState& query) {
  query.source.reset();
  if (query.error) {
    query.payload->error = query.error;
  } else {
    const auto start = StartTiming(query.stage_timing);
    try {
      query.payload->admission = loop_internal::FinishLoopAdmission(
          query.payload->task.input, query.reader, query.options,
          std::move(query.admission), query.stage_timing);
    } catch (...) {
      query.payload->error = std::current_exception();
    }
    LogStageTiming("loop_admission", start);
  }
  LogStageTiming("loop_query_compute", query.compute_start);
  query.completion(std::move(query.payload));
}

}  // namespace

LoopAdmissionResult PrepareLoopTask(const LoopTask& task,
                                   const LoopAdmissionOptions& options) {
  ValidateLoopAdmissionOptions(options);
  ValidateTask(task);
  return loop_internal::PrepareLoopAdmission(task.input, options);
}

LoopAdmissionResult RunLoopQuery(const LoopTask& task,
                                const LoopAdmissionOptions& options) {
  ValidateLoopAdmissionOptions(options);
  ValidateTask(task);
  const auto reader = MakeGeometryReader(task, options.geometry);
  return ProposeLoopUpdate(task.input, reader, options);
}

void ScheduleLoopQuery(
    std::shared_ptr<LoopTaskResult> payload, LoopAdmissionOptions options,
    LoopAdmissionResult prepared, ThreadPoolInterface& pool, int num_threads,
    std::function<void(std::shared_ptr<LoopTaskResult>)> completion,
    bool stage_timing) {
  if (!payload || !completion || num_threads <= 0) {
    throw std::invalid_argument("Missing loop query payload, callback or pool");
  }
  const auto batch_size = static_cast<std::size_t>(std::min(
      std::max(1, num_threads - 1), options.geometry.max_resident_submaps - 1));
  auto query = std::make_shared<LoopQueryState>();
  query->stage_timing = stage_timing;
  query->compute_start = StartTiming(stage_timing);
  query->payload = std::move(payload);
  query->options = std::move(options);
  query->admission = std::move(prepared);
  query->reader = MakeGeometryReader(query->payload->task,
                                     query->options.geometry, stage_timing);
  const auto count = query->admission.candidates.size();
  query->targets.resize(count);
  query->errors.resize(count);
  query->completion = std::move(completion);

  std::weak_ptr<Task> previous_batch;
  for (std::size_t begin = 0; begin < count; begin += batch_size) {
    const auto end = std::min(begin + batch_size, count);
    auto preparation = std::make_unique<Task>();
    preparation->AddDependency(previous_batch);
    preparation->SetWorkItem(
        [query, begin, end] { PrepareBatch(*query, begin, end); });
    const auto prepared_batch = pool.Schedule(std::move(preparation));
    auto merge = std::make_unique<Task>();
    for (std::size_t index = begin; index < end; ++index) {
      auto verification = std::make_unique<Task>();
      verification->AddDependency(prepared_batch);
      verification->SetWorkItem(
          [query, index] { VerifyCandidate(*query, index); });
      merge->AddDependency(pool.Schedule(std::move(verification)));
    }
    merge->SetWorkItem([query, begin, end] { MergeBatch(*query, begin, end); });
    previous_batch = pool.Schedule(std::move(merge));
  }
  auto finish = std::make_unique<Task>();
  finish->AddDependency(previous_batch);
  finish->SetWorkItem([query] { FinishQuery(*query); });
  pool.Schedule(std::move(finish));
}

}  // namespace livo_atlas
