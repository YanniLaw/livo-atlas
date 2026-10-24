#ifndef LIVO_ATLAS_ROS2_MAPPING_SESSION_H_
#define LIVO_ATLAS_ROS2_MAPPING_SESSION_H_

#include <functional>
#include <memory>
#include <optional>

#include "livo_atlas/frontend/frontend.h"
#include "livo_atlas/io/result_writer.h"
#include "livo_atlas/sensor/lidar_scan.h"
#include "livo_atlas/sensor/measurements.h"

namespace livo_atlas::ros2 {
struct PreprocessOptions {
  int lidar_type = kLivox;
  int scan_lines = 6;
  int point_filter = 3;
  double blind = 0.01;
  bool features = false;
};
struct SessionOptions {
  FrontendOptions frontend;
  OutputOptions output;
  OutputProcessorOptions processor;
  PreprocessOptions preprocess;
  bool lidar_enabled = true;
  bool image_enabled = true;
  bool hilti = false;
  bool driver_fix = false;
  bool propagate_imu = false;
  double exposure_time = 0;
  double imu_time_offset = 0;
};
struct PropagationResult {
  double timestamp = 0;
  State state;
};
// Private adapter implementation. All calls belong to one executor. Views
// passed to the callback must be consumed before the next ProcessAvailable
// iteration.
class MappingSession {
 public:
  using ResultCallback =
      std::function<void(const FrontendResult&, const PreparedOutput&, double)>;
  explicit MappingSession(SessionOptions options);
  ~MappingSession();
  MappingSession(const MappingSession&) = delete;
  MappingSession& operator=(const MappingSession&) = delete;
  bool AddLidar(const LidarScan& scan);
  bool AddImu(ImuMeasurement measurement);
  bool AddImage(ImageMeasurement measurement);
  void ProcessAvailable(const ResultCallback& callback);
  std::optional<PropagationResult> PropagateLatest();
  void Finish();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace livo_atlas::ros2
#endif  // LIVO_ATLAS_ROS2_MAPPING_SESSION_H_
