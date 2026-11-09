#ifndef LIVO_ATLAS_CORE_LIVO_SYSTEM_H_
#define LIVO_ATLAS_CORE_LIVO_SYSTEM_H_
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <memory>
#include <optional>

#include "livo_atlas/core/online_submap_update.h"
#include "livo_atlas/core/system_options.h"
#include "livo_atlas/frontend/frontend.h"
#include "livo_atlas/sensor/lidar_preprocessor.h"
#include "livo_atlas/sensor/lidar_scan.h"
#include "livo_atlas/sensor/measurements.h"
#include "livo_atlas/sensor/synchronizer.h"
namespace livo_atlas {
class KeyframeStore;
class OnlineSubmapManager;
class SubmapStore;

struct PropagationResult {
  double timestamp = 0;
  FilterState state;
};
// All calls belong to one executor. Views
// passed to the callback must be consumed before the next ProcessNext call.
class LivoSystem {
 public:
  using ResultCallback = std::function<void(
      const FrontendResult&, const ColoredCloud::ConstPtr&, double)>;
  using OnlineSubmapCallback = std::function<void(const OnlineSubmapUpdate&)>;
  // Register before the first frame; callbacks only enqueue, never reenter or
  // throw.
  void SetOnlineSubmapCallback(OnlineSubmapCallback callback);
  std::shared_ptr<const PoseSnapshot> GetPoseSnapshot() const;
  void ApplyPoseUpdate(const PoseUpdate& update);
  explicit LivoSystem(SystemOptions options);
  ~LivoSystem();
  LivoSystem(const LivoSystem&) = delete;
  LivoSystem& operator=(const LivoSystem&) = delete;
  // The caller applies LiDAR/image time offsets. Nonfinite timestamps are
  // rejected before queues, cursors or sampling counters change.
  bool AddLidar(const LidarScan& scan);
  // IMU timestamps precede the configured IMU offset and driver correction.
  bool AddImu(Imu measurement);
  bool AddImage(TimedImage measurement);
  SyncStatus ProcessNext(const ResultCallback& callback);
  void ProcessAvailable(const ResultCallback& callback);
  std::optional<PropagationResult> PropagateLatest();
  // A submap persistence failure is terminal: subsequent processing and Finish
  // rethrow the original error. Already committed data remains readable.
  void Finish();

 private:
  void AppendSubmap(const Submap& submap);
  OnlineSubmapManager& Online() const;
  void Propagate(const Imu& measurement);

  SystemOptions options_;
  LidarPreprocessor preprocess_;
  SensorBuffers buffers_;
  SensorSynchronizer synchronizer_;
  SynchronizedMeasurements measurements_;
  Frontend frontend_;
  CloudColorizer colorizer_;
  KeyframeSelector keyframe_selector_;
  std::unique_ptr<KeyframeStore> keyframe_store_;
  SubmapBuilder submap_builder_;
  std::unique_ptr<SubmapStore> submap_store_;
  // A committed-data failure stops later online mutations.
  std::exception_ptr submap_error_;
  std::unique_ptr<OnlineSubmapManager> online_;
  LivoExporter writer_;
  std::uint64_t next_keyframe_id_ = 0;
  double last_lidar_ = -1, last_imu_ = -1, last_image_ = -1;
  std::uint64_t image_count_ = 0;
  std::deque<Imu> propagation_buffer_;
  Imu newest_imu_;
  bool new_imu_ = false, state_updated_ = false, have_estimate_ = false;
  FilterState latest_state_, propagated_;
  double latest_time_ = 0, last_from_end_ = 0;
};
}  // namespace livo_atlas
#endif  // LIVO_ATLAS_CORE_LIVO_SYSTEM_H_
