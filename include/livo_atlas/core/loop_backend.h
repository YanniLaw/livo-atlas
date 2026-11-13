#ifndef LIVO_ATLAS_CORE_LOOP_BACKEND_H_
#define LIVO_ATLAS_CORE_LOOP_BACKEND_H_

#include <exception>
#include <memory>
#include <optional>
#include <vector>

#include "livo_atlas/backend/loop_worker.h"
#include "livo_atlas/core/online_submap_update.h"
#include "livo_atlas/io/loop_closure_store.h"

namespace livo_atlas {
class LivoSystem;
class LoopBackendTestPeer;

// All methods run on the system owner. Only the worker's owned task values
// cross threads; Observe collects metadata without I/O or reentering the
// system.
class LoopBackend {
 public:
  LoopBackend(LivoSystem& system, LoopAdmissionOptions options = {});
  ~LoopBackend();
  LoopBackend(const LoopBackend&) = delete;
  LoopBackend& operator=(const LoopBackend&) = delete;
  void Observe(const OnlineSubmapUpdate& update) noexcept;
  void Poll();
  void Finish();

  LoopSessionStats GetStats() const { return stats_; }

 private:
  friend class LoopBackendTestPeer;
  LoopBackend(LivoSystem& system, LoopAdmissionOptions options,
              LoopTaskRunner runner);
  static LoopAdmissionOptions ValidateSession(LivoSystem& system,
                                              LoopAdmissionOptions options);
  LoopAdmissionInput CurrentInput(std::uint64_t query_id) const;
  void RecordMembership(const OnlineSubmap& group);
  void QueueQuery(std::uint64_t query_id);
  void StopDispatch();
  void FlushEvents();
  void Complete(LoopTaskResult result);
  [[noreturn]] void Fail(std::exception_ptr error);

  struct Coalesced {
    std::uint64_t first = 0, last = 0, count = 0;
  };

  LivoSystem& system_;
  const LoopAdmissionOptions options_;
  LoopClosureStore store_;
  LoopWorker worker_;
  std::vector<RigidTransform> raw_poses_;
  std::vector<double> timestamps_;
  std::vector<OnlineSubmap> submaps_;
  std::vector<AcceptedLoop> accepted_loops_;
  std::shared_ptr<const PoseSnapshot> poses_;
  std::optional<std::uint64_t> pending_;
  std::optional<Coalesced> coalesced_;
  LoopSessionStats stats_;
  std::exception_ptr failure_;
  std::uint64_t latest_query_ = 0;
  double latest_timestamp_ = 0;
  bool budget_stopped_ = false;
  bool limit_event_pending_ = false;
  bool finished_ = false;
};
}  // namespace livo_atlas

#endif  // LIVO_ATLAS_CORE_LOOP_BACKEND_H_
