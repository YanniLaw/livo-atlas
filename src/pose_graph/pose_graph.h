#ifndef LIVO_ATLAS_SRC_POSE_GRAPH_POSE_GRAPH_H_
#define LIVO_ATLAS_SRC_POSE_GRAPH_POSE_GRAPH_H_

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include "livo_atlas/backend/loop_worker.h"
#include "livo_atlas/io/loop_closure_store.h"
#include "pose_graph/work_queue.h"

namespace livo_atlas {
class ThreadPool;
}  // namespace livo_atlas

namespace livo_atlas::pose_graph {

struct KeyframeObservation {
  std::uint64_t id;
  double timestamp;
  RigidTransform odom_from_body;
  std::optional<RigidTransform> parent_from_body;
};

struct Observation {
  std::optional<KeyframeObservation> keyframe;
  std::vector<OnlineSubmap> revisions;
  PoseFrontier frontier;
  std::shared_ptr<const PoseSnapshot> poses;
};

struct CoalescedQueries {
  std::uint64_t first, last, count;
};

struct ResourceLimit {
  std::uint64_t query_id, frame_count;
};

struct Completion {
  std::shared_ptr<const LoopTaskResult> result;
  LoopDisposition disposition;
};

struct Progress {
  std::optional<CoalescedQueries> coalesced;
  std::optional<ResourceLimit> limit;
  std::optional<Completion> completion;
  std::exception_ptr failure;
};

// The pool's serial execution line owns graph state. The system owner sends
// observations, supplies read views, and acknowledges durable publication.
class PoseGraph {
 public:
  PoseGraph(LoopAdmissionOptions options, LoopTaskRunner runner,
            bool stage_timing);
  ~PoseGraph();

  void CheckInputCapacity() const;
  void AddObservation(Observation observation);
  void SetReadView(KeyframeReadView view);
  Progress TakeProgress();
  void Acknowledge(LoopDisposition disposition,
                   std::shared_ptr<const PoseSnapshot> poses);
  LoopSessionStats GetStats() const;
  bool Drained() const;
  void Wait();
  void Stop();

 private:
  void Enqueue(WorkItem item, bool keyframe);
  void ScheduleDrainLocked();
  void Drain() noexcept;
  void RecordFailure(std::exception_ptr error) noexcept;
  void RecordObservation(const Observation& observation);
  void RecordMembership(const OnlineSubmap& group);
  void QueueQuery(std::uint64_t query_id);
  void StopDispatch();
  LoopAdmissionInput CurrentInput(std::uint64_t query_id) const;
  void Dispatch();
  void RunQuery(std::shared_ptr<LoopTaskResult> payload) noexcept;
  void Complete(std::shared_ptr<const LoopTaskResult> result);
  LoopDisposition Classify(const LoopTaskResult& result);
  void ApplyAcknowledgement(LoopDisposition disposition,
                            const std::shared_ptr<const PoseSnapshot>& poses);

  const LoopAdmissionOptions options_;
  const LoopTaskRunner runner_;
  const bool stage_timing_;

  PoseSnapshot poses_;
  std::vector<RigidTransform> raw_poses_;
  std::vector<double> timestamps_;
  std::vector<OnlineSubmap> submaps_;
  std::vector<AcceptedLoop> accepted_loops_, next_loops_;
  LoopSessionStats stats_;
  std::optional<std::uint64_t> pending_, active_query_;
  std::uint64_t latest_query_ = 0;
  bool budget_stopped_ = false;
  std::chrono::steady_clock::time_point dispatch_time_;

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  WorkQueue queue_;
  std::optional<KeyframeReadView> read_view_;
  Progress progress_;
  LoopSessionStats published_stats_;
  std::size_t queued_keyframes_ = 0;
  std::size_t peak_queued_keyframes_ = 0;
  bool drain_scheduled_ = false;
  bool view_changed_ = false;
  bool idle_ = true;
  bool stopping_ = false;
  std::unique_ptr<ThreadPool> thread_pool_;
};

}  // namespace livo_atlas::pose_graph

#endif  // LIVO_ATLAS_SRC_POSE_GRAPH_POSE_GRAPH_H_
