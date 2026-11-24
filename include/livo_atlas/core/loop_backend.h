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
class OnlineSubmapManager;
namespace pose_graph {
class PoseGraph;
}  // namespace pose_graph

// The system owner publishes graph results and writes the loop journal.
// Observe hands owned metadata to the serial graph without reentering the
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

  LoopSessionStats GetStats() const;

 private:
  friend class LivoSystem;
  friend class LoopBackendTestPeer;
  friend class OnlineSubmapManager;
  LoopBackend(LivoSystem& system, LoopAdmissionOptions options,
              LoopTaskRunner runner);
  static LoopAdmissionOptions ValidateSession(LivoSystem& system,
                                              LoopAdmissionOptions options);
  void ObserveDelta(const OnlineSubmapDelta& update) noexcept;
  void ObserveFrontier(const Keyframe* keyframe,
                       const std::vector<OnlineSubmap>& revisions,
                       const PoseFrontier& frontier,
                       const std::shared_ptr<const PoseSnapshot>& poses,
                       const std::optional<RigidTransform>& appended_pose);
  void CheckInputCapacity() const;
  void Complete(std::shared_ptr<const LoopTaskResult> result,
                LoopDisposition disposition);
  void Drain();
  void FinishJournal();
  void StopOnFailure(std::exception_ptr error) noexcept;
  [[noreturn]] void Fail(std::exception_ptr error);

  LivoSystem& system_;
  const LoopAdmissionOptions options_;
  LoopClosureStore store_;
  std::unique_ptr<pose_graph::PoseGraph> graph_;
  PoseFrontier observed_;
  std::shared_ptr<const PoseSnapshot> published_poses_;
  std::exception_ptr failure_;
  std::uint64_t published_loops_ = 0;
  double latest_timestamp_ = 0;
  bool publishing_ = false;
  bool finished_ = false;
};
}  // namespace livo_atlas

#endif  // LIVO_ATLAS_CORE_LOOP_BACKEND_H_
