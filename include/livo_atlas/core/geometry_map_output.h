#ifndef LIVO_ATLAS_CORE_GEOMETRY_MAP_OUTPUT_H_
#define LIVO_ATLAS_CORE_GEOMETRY_MAP_OUTPUT_H_

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
#include <utility>
#include <vector>

#include "livo_atlas/core/geometry_map_output_options.h"
#include "livo_atlas/core/online_submap_update.h"
#include "livo_atlas/io/keyframe_read_view.h"
#include "livo_atlas/mapping/geometry_map_builder.h"

namespace livo_atlas {
class LivoSystem;

struct GeometryMapOutputStats {
  std::uint64_t observed = 0, submitted = 0, coalesced = 0, delivered = 0;
  std::uint64_t resource_limited = 0, failed = 0, timestamp_reads = 0;
  std::uint64_t peak_work_payloads = 0, peak_pending = 0;
  std::uint64_t last_revision = 0, last_pose_version = 0, last_frame_count = 0;
  std::uint64_t last_optimized_frame_count = 0, last_submap_count = 0;
  std::uint64_t last_source_voxel_count = 0, last_point_count = 0;
  double max_observe_seconds = 0, max_poll_seconds = 0;
  double max_worker_seconds = 0, last_latency_seconds = 0;
  GeometryMapBuilderStats geometry;
  bool finished = false;
  std::string error;
};

// Observe/Poll/Finish are called by the system owner. The worker only reads
// owned membership and pose snapshots plus a fixed committed file view.
// A consumer borrows its result and its captured assets must outlive the join.
class GeometryMapOutput {
 public:
  using Consumer = std::function<void(const GeometryMapSnapshot&)>;

  GeometryMapOutput(LivoSystem& system, GeometryMapOutputOptions options,
                    GeometryOptions geometry, Consumer consumer);
  ~GeometryMapOutput();
  GeometryMapOutput(const GeometryMapOutput&) = delete;
  GeometryMapOutput& operator=(const GeometryMapOutput&) = delete;

  void Observe(const OnlineSubmapUpdate& update) noexcept;
  void Poll();
  void Finish();
  GeometryMapOutputStats GetStats() const;

 private:
  using Clock = std::chrono::steady_clock;

  struct Observed {
    GeometryMapSnapshot header;
    std::shared_ptr<const PoseSnapshot> poses;
    Clock::time_point observed_at;
  };

  struct Task {
    Observed source;
    std::vector<OnlineSubmap> groups;
    std::optional<KeyframeReadView> view;
  };

  bool ApplyRevisions(const OnlineSubmapUpdate& update);
  void Poll(bool force);
  void Run();
  void Stop();
  void RecordFailure(const std::string& error);

  LivoSystem& system_;
  const GeometryMapOutputOptions options_;
  const GeometryOptions geometry_options_;
  const Consumer consumer_;
  std::string producer_id_;
  GeometryMapSnapshot latest_header_;
  std::shared_ptr<const PoseSnapshot> latest_poses_;
  std::vector<OnlineSubmap> groups_;
  std::optional<Observed> pending_;
  std::exception_ptr observe_error_;
  std::uint64_t revision_ = 0, submap_count_ = 0;
  std::uint64_t last_membership_revision_ = 0;
  OnlineSubmapState last_membership_state_ = OnlineSubmapState::kActive;
  bool have_source_ = false, limited_ = false, terminal_pending_ = false;
  bool finished_ = false;

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::optional<Task> task_;
  GeometryMapOutputStats stats_;
  Clock::time_point last_delivery_;
  bool busy_ = false, failed_ = false, have_delivery_ = false;
  bool nonready_delivered_ = false;
  std::atomic<bool> stopping_{false};
  std::thread thread_;
  // These members belong only to the worker after construction.
  std::optional<KeyframeReadView> view_;
  std::unique_ptr<GeometryMapBuilder> builder_;
  std::optional<std::pair<std::uint64_t, double>> timestamp_;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_CORE_GEOMETRY_MAP_OUTPUT_H_
