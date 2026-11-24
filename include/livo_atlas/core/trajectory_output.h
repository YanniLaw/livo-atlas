#ifndef LIVO_ATLAS_CORE_TRAJECTORY_OUTPUT_H_
#define LIVO_ATLAS_CORE_TRAJECTORY_OUTPUT_H_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "livo_atlas/core/online_submap_update.h"
#include "livo_atlas/core/trajectory_output_options.h"
#include "livo_atlas/io/keyframe_read_view.h"
#include "livo_atlas/mapping/trajectory_snapshot.h"

namespace livo_atlas {
class LivoSystem;

struct TrajectoryOutputStats {
  std::uint64_t observed = 0, submitted = 0, coalesced = 0, delivered = 0;
  std::uint64_t resource_limited = 0, failed = 0, timestamp_reads = 0;
  std::uint64_t peak_work_payloads = 0, peak_pending = 0;
  std::uint64_t last_revision = 0, last_pose_version = 0, last_frame_count = 0;
  double max_observe_seconds = 0, max_poll_seconds = 0;
  double max_worker_seconds = 0, last_latency_seconds = 0;
  bool finished = false;
  std::string error;
};

// Observe/Poll/Finish belong to the system owner. Only owned snapshots and a
// committed read view cross to the worker; it never calls the live system.
// Consumer borrows its argument on that worker and must not reenter the system.
// Its captured publisher handles/assets must outlive this object's join.
class TrajectoryOutput {
 public:
  using Consumer = std::function<void(const TrajectorySnapshot&)>;

  TrajectoryOutput(LivoSystem& system, TrajectoryOutputOptions options,
                   Consumer consumer);
  ~TrajectoryOutput();
  TrajectoryOutput(const TrajectoryOutput&) = delete;
  TrajectoryOutput& operator=(const TrajectoryOutput&) = delete;

  void Observe(const OnlineSubmapUpdate& update) noexcept;
  void ObserveDelta(const OnlineSubmapDelta& update) noexcept;
  void Poll();
  void Finish();
  TrajectoryOutputStats GetStats() const;

 private:
  using Clock = std::chrono::steady_clock;

  struct Observed {
    TrajectorySnapshot header;
    std::shared_ptr<const PoseSnapshot> poses;
    Clock::time_point observed_at;
  };

  struct Task {
    Observed source;
    std::optional<KeyframeReadView> view;
  };

  void Poll(bool force);
  void ObserveFrontier(
      std::optional<PoseFrontier> frontier,
      const std::shared_ptr<const PoseSnapshot>& poses) noexcept;
  void Run();
  void Stop();
  void RecordFailure(const std::string& error);

  LivoSystem& system_;
  const TrajectoryOutputOptions options_;
  const Consumer consumer_;
  std::string producer_id_;
  TrajectorySnapshot latest_header_;
  std::optional<Observed> pending_;
  std::exception_ptr observe_error_;
  std::uint64_t revision_ = 0;
  bool have_source_ = false, terminal_pending_ = false, finished_ = false;

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::optional<Task> task_;
  TrajectoryOutputStats stats_;
  Clock::time_point last_delivery_;
  bool busy_ = false, failed_ = false, have_delivery_ = false;
  std::atomic<bool> stopping_{false};
  std::thread thread_;
  // Worker-only metadata cache. It never retains clouds, images or raw poses.
  std::vector<double> timestamps_;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_CORE_TRAJECTORY_OUTPUT_H_
