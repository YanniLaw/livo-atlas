#ifndef LIVO_ATLAS_RUNTIME_MAPPING_SESSION_H_
#define LIVO_ATLAS_RUNTIME_MAPPING_SESSION_H_
#include <functional>
#include <memory>
#include <optional>

#include "livo_atlas/frontend/frontend.h"
#include "livo_atlas/runtime/options.h"
#include "livo_atlas/sensor/lidar_scan.h"
#include "livo_atlas/sensor/measurements.h"
#include "livo_atlas/sensor/synchronizer.h"
namespace livo_atlas {
struct PropagationResult {
  double timestamp = 0;
  State state;
};
// All calls belong to one executor. Views
// passed to the callback must be consumed before the next ProcessNext call.
class MappingSession {
 public:
  using ResultCallback =
      std::function<void(const FrontendResult&, const PreparedOutput&, double)>;
  explicit MappingSession(SessionOptions options);
  ~MappingSession();
  MappingSession(const MappingSession&) = delete;
  MappingSession& operator=(const MappingSession&) = delete;
  // The caller applies LiDAR/image time offsets. Nonfinite timestamps are
  // rejected before queues, cursors or sampling counters change.
  bool AddLidar(const LidarScan& scan);
  // IMU timestamps precede the configured IMU offset and driver correction.
  bool AddImu(ImuMeasurement measurement);
  bool AddImage(ImageMeasurement measurement);
  SynchronizationResult ProcessNext(const ResultCallback& callback);
  void ProcessAvailable(const ResultCallback& callback);
  std::optional<PropagationResult> PropagateLatest();
  void Finish();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace livo_atlas
#endif  // LIVO_ATLAS_RUNTIME_MAPPING_SESSION_H_
