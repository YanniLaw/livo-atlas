#ifndef LIVO_ATLAS_CORE_LOOP_BACKEND_H_
#define LIVO_ATLAS_CORE_LOOP_BACKEND_H_

#include <exception>

#include "livo_atlas/backend/loop_worker.h"
#include "livo_atlas/core/online_submap_update.h"
#include "livo_atlas/io/loop_closure_store.h"

namespace livo_atlas {
class LivoSystem;
class LoopBackendTestPeer;

// Compatibility entry for SDK callers that explicitly observe, poll and
// finish the loop backend. All writes use the system's pose graph queue.
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
  friend class LoopBackendTestPeer;
  LoopBackend(LivoSystem& system, LoopAdmissionOptions options,
              LoopTaskRunner runner);

  LivoSystem& system_;
  PoseFrontier observed_;
  std::exception_ptr failure_;
  double latest_timestamp_ = 0;
  bool finished_ = false;
};
}  // namespace livo_atlas

#endif  // LIVO_ATLAS_CORE_LOOP_BACKEND_H_
