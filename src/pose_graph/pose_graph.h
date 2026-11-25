#ifndef LIVO_ATLAS_SRC_POSE_GRAPH_POSE_GRAPH_H_
#define LIVO_ATLAS_SRC_POSE_GRAPH_POSE_GRAPH_H_

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include "livo_atlas/backend/loop_worker.h"
#include "livo_atlas/core/online_submap_update.h"
#include "livo_atlas/io/keyframe_store.h"
#include "livo_atlas/io/loop_closure_store.h"
#include "livo_atlas/io/submap_store.h"
#include "livo_atlas/mapping/map_correction.h"
#include "pose_graph/work_queue.h"

namespace livo_atlas {
struct SystemOptions;
class ThreadPool;
}  // namespace livo_atlas

namespace livo_atlas::pose_graph {

class OnlineSubmapArchive;

struct ArchiveCommit {
  KeyframeReadView keyframes;
  std::optional<OnlineSubmapDelta> submaps;
  std::optional<MapCorrection> correction;
};

struct ArchiveProgress {
  std::optional<ArchiveCommit> commit;
  std::exception_ptr failure;
};

class ControlReceipt {
 private:
  friend class PoseGraph;
  bool done_ = false;
  std::exception_ptr rejection_;
};

// One serial execution line owns both the archive and graph. The owner
// delivers each commit outside this queue and acknowledges it before the next
// write. The worker yields whenever the publication slot is occupied.
class PoseGraph {
 public:
  explicit PoseGraph(const SystemOptions& options);
  ~PoseGraph();

  KeyframeReadView InitialReadView() const;
  // An empty runner selects the native dependent candidate tasks.
  void EnableLoops(LoopAdmissionOptions options, LoopTaskRunner runner,
                   bool standalone);
  void PollStandalone();
  void AbandonStandalone() noexcept;
  void AddKeyframe(std::shared_ptr<const Keyframe> keyframe);
  std::shared_ptr<const ControlReceipt> SealInput();
  std::shared_ptr<const ControlReceipt> ApplyPoseUpdate(PoseUpdate update);
  std::shared_ptr<const ControlReceipt> FinishArchive();

  ArchiveProgress TakeProgress();
  void AcknowledgeCommit();
  std::optional<std::exception_ptr> CheckControl(
      const ControlReceipt& receipt) const;
  LoopSessionStats GetStats() const;
  bool Drained() const;
  void Wait(const ControlReceipt* receipt = nullptr,
            bool poll_standalone = false);
  void RethrowFailure() const;
  std::exception_ptr Cancel(std::exception_ptr error) noexcept;
  void Stop();

 private:
  struct QueryReport {
    std::shared_ptr<const LoopTaskResult> result;
    LoopDisposition disposition;
  };
  struct CoalescedQueries {
    std::uint64_t first, last, count;
  };

  void Enqueue(WorkItem item, bool keyframe);
  std::shared_ptr<const ControlReceipt> EnqueueControl(
      std::function<std::exception_ptr()> task);
  void ScheduleDrainLocked();
  void Drain() noexcept;
  void Publish(ArchiveCommit commit, bool appended_keyframe = false,
               std::chrono::steady_clock::time_point loop_dispatch_time = {});
  void Append(std::shared_ptr<const Keyframe> keyframe);
  void RecordMembership(const std::vector<OnlineSubmap>& revisions);
  void QueueQuery(std::uint64_t query_id);
  void StopDispatch();
  void WriteLoopEvents();
  LoopAdmissionInput CurrentInput(std::uint64_t query_id) const;
  bool CanDispatchLocked() const;
  void Dispatch();
  void RunQuery(std::shared_ptr<LoopTaskResult> payload) noexcept;
  void DeliverQuery(std::shared_ptr<LoopTaskResult> payload) noexcept;
  WorkItem CompletionWork(std::shared_ptr<const LoopTaskResult> result,
                          std::chrono::steady_clock::time_point ready);
  void Complete(std::shared_ptr<const LoopTaskResult> result);
  LoopDisposition Classify(const LoopTaskResult& result);
  void RecordDisposition(LoopDisposition disposition);
  void WriteQueryReport();

  KeyframeStore keyframes_;
  SubmapBuilder groups_;
  std::unique_ptr<SubmapStore> group_store_;
  std::unique_ptr<OnlineSubmapArchive> online_;
  std::unique_ptr<LoopClosureStore> journal_;
  LoopAdmissionOptions options_;
  LoopTaskRunner runner_;
  const bool stage_timing_;
  const int num_background_threads_;
  bool standalone_ = false;
  bool failure_written_ = false;

  std::vector<double> timestamps_;
  std::vector<OnlineSubmap> submaps_;
  std::vector<AcceptedLoop> accepted_loops_, next_loops_;
  LoopSessionStats stats_;
  std::optional<std::uint64_t> pending_, active_query_;
  std::optional<QueryReport> query_report_;
  std::optional<CoalescedQueries> coalesced_;
  std::optional<std::uint64_t> resource_limit_query_;
  std::uint64_t latest_query_ = 0;
  double latest_timestamp_ = 0;
  bool budget_stopped_ = false;
  std::chrono::steady_clock::time_point dispatch_time_;

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  WorkQueue queue_;
  std::optional<ArchiveCommit> publication_;
  std::shared_ptr<const LoopTaskResult> standalone_result_;
  std::exception_ptr failure_;
  LoopSessionStats published_stats_;
  std::size_t queued_keyframes_ = 0;
  std::size_t peak_queued_keyframes_ = 0;
  std::chrono::steady_clock::time_point standalone_ready_, publication_time_;
  // Only an accepted loop publication carries a dispatch time to the owner.
  std::chrono::steady_clock::time_point published_loop_dispatch_time_;
  bool standalone_polling_ = false;
  bool standalone_abandoned_ = false;
  bool awaiting_delivery_ = false;
  bool drain_scheduled_ = false;
  bool idle_ = true;
  bool stopping_ = false;
  std::unique_ptr<ThreadPool> thread_pool_;
};

}  // namespace livo_atlas::pose_graph

#endif  // LIVO_ATLAS_SRC_POSE_GRAPH_POSE_GRAPH_H_
