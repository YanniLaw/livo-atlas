#ifndef LIVO_ATLAS_GAUSSIAN_ONLINE_GAUSSIAN_MAPPER_H_
#define LIVO_ATLAS_GAUSSIAN_ONLINE_GAUSSIAN_MAPPER_H_

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "livo_atlas/camera/camera_config.h"
#include "livo_atlas/core/online_gaussian_options.h"
#include "livo_atlas/core/online_submap_update.h"

namespace livo_atlas {

class LivoSystem;

// A confirmed, owned RGB image and the exact pose frontier used to train it.
struct GaussianPreview {
  std::string session_id;
  std::uint64_t job_id = 0, submap_id = 0, pose_version = 0, generation = 0;
  std::uint64_t membership_revision = 0, frame_count = 0;
  std::uint64_t optimized_frame_count = 0;
  std::vector<std::uint64_t> frame_ids;
  RigidTransform parent_from_submap;
  double capture_time = 0, rgb_l1 = 0, depth_l1_m = 0;
  double latency_seconds = 0, worker_seconds = 0, confirmed_at_unix_seconds = 0;
  int width = 0, height = 0, steps = 0, gaussian_count = 0;
  int supervised_depth_pixels = 0;
  bool nonzero_gradient = false, parameter_updated = false;
  std::uint64_t gpu_peak_allocated_bytes = 0, gpu_peak_reserved_bytes = 0;
  std::uint64_t worker_peak_rss_bytes = 0;
  std::vector<std::uint8_t> rgb;
};

struct OnlineGaussianStats {
  std::uint64_t observed = 0, submitted = 0, coalesced = 0;
  std::uint64_t skipped_no_image = 0, skipped_no_depth = 0,
                resource_limited = 0;
  std::uint64_t completed = 0, accepted = 0, stale = 0, resets = 0, failed = 0;
  std::uint64_t pending = 0, in_flight = 0, peak_pending = 0,
                peak_in_flight = 0;
  std::uint64_t pose_version = 0, generation = 0, submap_id = 0;
  bool has_submap = false, finished = false;
  double max_observe_seconds = 0, max_poll_seconds = 0,
         last_latency_seconds = 0;
  double max_latency_seconds = 0, total_latency_seconds = 0;
  // Confirmation latency buckets: <= 0.1, 0.25, 0.5, 1, 2, 5, 10, > 10 s.
  std::array<std::uint64_t, 8> latency_buckets{};
  std::string error;
};

// Observe, Poll, Finish and queries belong to the LivoSystem owner thread.
// Socket, reconstruction and filesystem operations belong to one worker thread.
class OnlineGaussianMapper {
 public:
  OnlineGaussianMapper(LivoSystem& system, CameraConfig camera,
                       OnlineGaussianOptions options);
  ~OnlineGaussianMapper();
  OnlineGaussianMapper(const OnlineGaussianMapper&) = delete;
  OnlineGaussianMapper& operator=(const OnlineGaussianMapper&) = delete;

  void Observe(const OnlineSubmapUpdate& update) noexcept;
  void Poll() noexcept;
  void Finish() noexcept;
  std::shared_ptr<const GaussianPreview> GetLatestPreview() const;
  OnlineGaussianStats GetStats() const;

 private:
  using Clock = std::chrono::steady_clock;

  struct Source {
    std::shared_ptr<const Keyframe> keyframe;
    std::shared_ptr<const PoseSnapshot> poses;
    OnlineSubmap submap;
    std::uint64_t generation = 0, job_id = 0;
    Clock::time_point observed_at;
  };

  enum class CommandKind { kReset, kView, kDecision, kSave, kStatus };

  struct Command {
    CommandKind kind = CommandKind::kReset;
    Source source;
    std::shared_ptr<const GaussianPreview> preview;
    std::optional<std::uint64_t> submap_id;
    std::uint64_t generation = 0, pose_version = 0, state_sequence = 0;
    bool accept = false, ready = false;
    std::string preview_name;
  };

  enum class ReplyKind {
    kConnected,
    kReset,
    kCandidate,
    kConfirmed,
    kSkipped,
    kSaved,
    kStatus,
    kError
  };

  struct Reply {
    ReplyKind kind = ReplyKind::kError;
    Command command;
    std::shared_ptr<GaussianPreview> preview;
    std::string detail;
  };

  void Run() noexcept;
  void HandleReply(Reply reply);
  void Dispatch();
  void Fail(const std::string& message) noexcept;
  void Invalidate();
  bool IsCurrent(std::uint64_t generation, std::uint64_t submap_id,
                 std::uint64_t pose_version) const;

  const CameraConfig camera_;
  const OnlineGaussianOptions options_;
  const std::string session_id_;
  std::optional<OnlineSubmap> submap_;
  std::shared_ptr<const PoseSnapshot> poses_;
  std::shared_ptr<const Keyframe> latest_keyframe_;
  std::optional<Source> pending_, in_flight_;
  std::shared_ptr<GaussianPreview> candidate_;
  std::shared_ptr<const GaussianPreview> ready_, unsaved_;
  std::vector<std::uint64_t> remote_frame_ids_;
  std::optional<bool> decision_;
  std::uint64_t generation_ = 0, next_job_ = 1, remote_generation_ = 0;
  std::uint64_t state_sequence_ = 0, saved_status_sequence_ = 0;
  bool connected_ = false, remote_known_ = false, busy_ = false;
  bool awaiting_decision_ = false, failed_ = false, finished_ = false;
  std::string saved_preview_name_;
  OnlineGaussianStats stats_;

  std::mutex mutex_;
  std::condition_variable condition_;
  std::optional<Command> command_;
  std::optional<Reply> reply_;
  std::atomic<bool> stopping_{false};
  std::atomic<int> socket_fd_{-1};
  std::thread thread_;
  // Only the worker owns and prunes these paths after creating the directory.
  std::deque<std::string> written_previews_;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_GAUSSIAN_ONLINE_GAUSSIAN_MAPPER_H_
