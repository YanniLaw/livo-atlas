#ifndef LIVO_ATLAS_GAUSSIAN_GAUSSIAN_RENDER_OUTPUT_H_
#define LIVO_ATLAS_GAUSSIAN_GAUSSIAN_RENDER_OUTPUT_H_

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "livo_atlas/core/gaussian_render_output_options.h"
#include "livo_atlas/gaussian/gaussian_render_snapshot.h"

namespace livo_atlas {

struct GaussianRenderOutputStats {
  std::uint64_t observed = 0, submitted = 0, coalesced = 0;
  std::uint64_t published = 0, unavailable = 0, failed = 0;
  std::uint64_t pending = 0, in_flight = 0;
  std::uint64_t peak_pending = 0, peak_in_flight = 0, last_revision = 0;
  double max_worker_seconds = 0;
  bool finished = false;
  std::string error;
};

// Observe/Finish have a single owner. The consumer runs serially outside the
// mutex; it must eventually return. true means native calls completed, never
// remote delivery. Finish joins before the publisher may be destroyed.
class GaussianRenderOutput {
 public:
  using Consumer = std::function<bool(const GaussianRenderSnapshot&)>;

  GaussianRenderOutput(GaussianRenderOutputOptions options, Consumer consumer);
  ~GaussianRenderOutput();
  GaussianRenderOutput(const GaussianRenderOutput&) = delete;
  GaussianRenderOutput& operator=(const GaussianRenderOutput&) = delete;

  void Observe(const GaussianRenderSnapshot& source) noexcept;
  void Finish() noexcept;
  GaussianRenderOutputStats GetStats() const;

 private:
  void Run() noexcept;
  void Fail(const char* reason);
  void Queue(GaussianRenderSnapshot source, bool urgent);

  const GaussianRenderOutputOptions options_;
  const Consumer consumer_;
  const std::string producer_id_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::optional<GaussianRenderSnapshot> latest_, pending_;
  std::weak_ptr<const GaussianRender> latest_render_;
  GaussianRenderOutputStats stats_;
  bool urgent_ = false, closing_ = false;
  std::thread thread_;
};

}  // namespace livo_atlas
#endif  // LIVO_ATLAS_GAUSSIAN_GAUSSIAN_RENDER_OUTPUT_H_
