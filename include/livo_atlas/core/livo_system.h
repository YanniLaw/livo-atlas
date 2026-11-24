#ifndef LIVO_ATLAS_CORE_LIVO_SYSTEM_H_
#define LIVO_ATLAS_CORE_LIVO_SYSTEM_H_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include "livo_atlas/common/filter_state.h"
#include "livo_atlas/core/loop_backend.h"
#include "livo_atlas/core/online_submap_update.h"
#include "livo_atlas/core/system_options.h"
#include "livo_atlas/frontend/frontend.h"
#include "livo_atlas/frontend/synchronized_measurements.h"
#include "livo_atlas/io/keyframe_read_view.h"
#include "livo_atlas/io/livo_exporter.h"
#include "livo_atlas/mapping/cloud_colorizer.h"
#include "livo_atlas/mapping/geometry_submap.h"
#include "livo_atlas/mapping/keyframe_selector.h"
#include "livo_atlas/mapping/map_correction.h"
#include "livo_atlas/mapping/pose_snapshot.h"
#include "livo_atlas/mapping/submap.h"
#include "livo_atlas/sensor/lidar_preprocessor.h"
#include "livo_atlas/sensor/lidar_scan.h"
#include "livo_atlas/sensor/measurements.h"
#include "livo_atlas/sensor/synchronizer.h"

namespace livo_atlas {

class GeometryMapper;
class LoopBackendTestPeer;
namespace pose_graph {
struct ArchiveCommit;
class ControlReceipt;
class PoseGraph;
}  // namespace pose_graph

struct PropagationResult {
  double timestamp = 0;
  FilterState state;
};

struct StartupBufferStats {
  std::size_t pending_imu = 0;
  std::size_t pending_images = 0;
  std::uint64_t buffered_imu = 0;
  std::uint64_t buffered_images = 0;
  std::uint64_t evicted_imu = 0;
  std::uint64_t evicted_images = 0;
  std::uint64_t rejected_imu = 0;
};

struct SensorBufferCounts {
  std::size_t lidar = 0;
  std::size_t imu = 0;
  std::size_t image = 0;
  std::size_t propagation_imu = 0;
};

// Callers serialize access to the system. Consume callback views before the
// next ProcessNext call.
class LivoSystem {
 public:
  using ResultCallback = std::function<void(
      const FrontendResult&, const ColoredCloud::ConstPtr&, double)>;
  using OnlineSubmapCallback = std::function<void(const OnlineSubmapUpdate&)>;
  using OnlineSubmapDeltaCallback =
      std::function<void(const OnlineSubmapDelta&)>;
  // Register before the first queued frame. The system owner calls consumers
  // in commit order. Callbacks may read committed views and must not reenter
  // processing. An exception stops the session after retaining the commit.
  void SetOnlineSubmapCallback(OnlineSubmapCallback callback);
  void SetOnlineSubmapDeltaCallback(OnlineSubmapDeltaCallback callback);
  // Read the committed metadata without materializing the pose history.
  PoseFrontier GetPoseFrontier() const;
  std::shared_ptr<const PoseSnapshot> GetPoseSnapshot() const;
  // Constant-size derived view; safe for read-only diagnostics after a
  // committed publication failure, without materializing the full trajectory.
  MapCorrection GetMapCorrection() const;
  KeyframeReadView GetKeyframeReadView() const;
  std::optional<LoopSessionStats> GetLoopStats() const;
  void ApplyPoseUpdate(const PoseUpdate& update);
  GeometrySubmap GetGeometrySubmap(std::uint64_t id);
  GeometryCacheStats GetGeometryStats() const;
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
  // Called by the same owner as input methods; counts include startup only.
  StartupBufferStats GetStartupBufferStats() const noexcept;
  SensorBufferCounts GetSensorBufferCounts() const noexcept;
  // Let a batched caller try synchronization before submitting input that
  // could exceed the current count or corrected timestamp span.
  bool WouldExceedInputLimit(const LidarScan& scan) const;
  bool WouldExceedInputLimit(const Imu& imu) const;
  bool WouldExceedInputLimit(const TimedImage& image) const;
  // Reports front-end progress. Read views reflect delivered archive commits.
  SyncStatus ProcessNext(const ResultCallback& callback);
  // Drain the current sensor batch and deliver its keyframe writes.
  void ProcessAvailable(const ResultCallback& callback);
  std::optional<PropagationResult> PropagateLatest();
  // Close input and seal the tail while keeping pose updates available.
  void SealInput();
  // Record a terminal failure from the owner before an unsuccessful shutdown.
  void RecordFailure(std::exception_ptr error) noexcept;
  // Input overflow and persistence failures stop the session. Subsequent input,
  // processing and Finish rethrow the original error. Committed views remain
  // readable.
  void Finish();
  // Input sealing still permits pose updates; only successful Finish freezes
  // the source. A later terminal backend failure revokes completion.
  bool IsFinished() const noexcept;

 private:
  struct BufferPeak {
    std::size_t count = 0;
    double span_s = 0;
  };

  friend class LoopBackend;
  friend class LoopBackendTestPeer;
  void EnableLoops(LoopAdmissionOptions options, LoopTaskRunner runner,
                   LoopBackend* backend);
  void ClearLoopBackend(const LoopBackend& backend) noexcept;
  void FinishSession(LoopBackend* backend);
  LoopSessionStats LoopStats() const;
  void CheckInputOpen();
  SyncStatus ProcessMeasurement(const ResultCallback& callback);
  template <typename Sample>
  void CheckBufferLimit(const char* name, const std::deque<Sample>& buffer,
                        int max_count, double timestamp);
  template <typename Sample>
  void RecordBufferPeak(const std::deque<Sample>& buffer, BufferPeak* peak,
                        bool ordered = true);
  void ReportBufferPeaks() const;
  void AcceptLidarAnchor(TimedPointCloud measurement);
  std::optional<double> CorrectImuTimestamp(double lidar_time,
                                            double previous_imu_time,
                                            Imu* measurement) const;
  void BufferImu(const Imu& measurement, bool propagate, std::deque<Imu>* imu,
                 std::deque<Imu>* propagation);
  void AppendKeyframe(std::shared_ptr<const Keyframe> keyframe);
  void CheckOnlineMapping() const;
  void CheckCallbackRegistration();
  void PumpArchive();
  void DeliverArchiveCommit(pose_graph::ArchiveCommit commit);
  void UpdateGeometry(const OnlineSubmapDelta& update);
  void WaitForKeyframes();
  std::exception_ptr WaitForControl(const pose_graph::ControlReceipt& receipt);
  void Propagate(const Imu& measurement);

  SystemOptions options_;
  LidarPreprocessor preprocess_;
  SensorBuffers buffers_;
  SensorSynchronizer synchronizer_;
  SynchronizedMeasurements measurements_;
  Frontend frontend_;
  CloudColorizer colorizer_;
  KeyframeSelector keyframe_selector_;
  std::exception_ptr session_error_;
  std::optional<KeyframeReadView> committed_keyframes_;
  PoseFrontier committed_frontier_;
  std::shared_ptr<const PoseSnapshot> frozen_poses_;
  std::vector<KeyframePose> appended_poses_;
  std::optional<MapCorrection> map_correction_{MapCorrection{}};
  std::unique_ptr<GeometryMapper> geometry_;
  std::uint64_t submap_count_ = 0;
  OnlineSubmapCallback submap_callback_;
  OnlineSubmapDeltaCallback submap_delta_callback_;
  std::unique_ptr<pose_graph::PoseGraph> pose_graph_;
  LivoExporter writer_;
  std::uint64_t next_keyframe_id_ = 0;
  bool have_first_lidar_ = false;
  bool input_sealed_ = false;
  bool finished_ = false;
  bool loop_pose_updates_claimed_ = false;
  LoopBackend* active_loop_backend_ = nullptr;
  double last_lidar_ = -1, last_imu_ = -1, last_image_ = -1;
  std::uint64_t image_count_ = 0;
  std::deque<Imu> startup_imu_;
  StartupBufferStats startup_stats_;
  BufferPeak lidar_peak_, imu_peak_, image_peak_, propagation_peak_;
  BufferPeak startup_imu_peak_, startup_image_peak_;
  std::deque<Imu> propagation_buffer_;
  Imu newest_imu_;
  bool new_imu_ = false, state_updated_ = false, have_estimate_ = false;
  FilterState latest_state_, propagated_;
  double latest_time_ = 0, last_from_end_ = 0;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_CORE_LIVO_SYSTEM_H_
