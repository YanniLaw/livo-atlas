#ifndef LIVO_ATLAS_FRONTEND_FRONTEND_H_
#define LIVO_ATLAS_FRONTEND_FRONTEND_H_

#include <pcl/filters/voxel_grid.h>

#include <memory>
#include <vector>

#include "livo_atlas/common/filter_state.h"
#include "livo_atlas/frontend/frontend_options.h"
#include "livo_atlas/frontend/frontend_result.h"
#include "livo_atlas/frontend/synchronized_measurements.h"
#include "livo_atlas/mapping/point_with_covariance.h"

namespace livo_atlas {

class Camera;
class ImuProcessor;
class VIOManager;
class VoxelMapManager;

class Frontend {
 public:
  // Options must satisfy the existing calibration/image/grid constraints.
  explicit Frontend(const FrontendOptions& options);
  ~Frontend();
  Frontend(const Frontend&) = delete;
  Frontend& operator=(const Frontend&) = delete;

  // One executor calls this once after each successful synchronization.
  // measurements must be non-null; the IMU advances its last update time.
  FrontendResult Process(SynchronizedMeasurements* measurements);
  const FilterState& state() const;
  bool imu_needs_initialization() const;
  // Returns zero when IMU is disabled or initialization has not completed.
  double imu_mean_acceleration_norm() const;

 private:
  void CaptureOutput(FrontendResult* result);
  void AlignGravity();
  void TransformCloud(const PointCloud::Ptr& input, PointCloud* output);
  bool ProcessLidar();
  bool ProcessVisual(SynchronizedMeasurements& measurements,
                     double relative_time);

  FrontendOptions options_;
  double cloud_timestamp_ = 0;
  FilterState state_;
  FilterState propagated_;
  std::unique_ptr<ImuProcessor> imu_;
  std::unique_ptr<VoxelMapManager> map_;
  // The visual manager borrows the camera; member order keeps it alive longer.
  std::unique_ptr<Camera> camera_;
  std::unique_ptr<VIOManager> visual_;
  PointCloud::Ptr undistorted_{new PointCloud};
  PointCloud::Ptr down_body_{new PointCloud};
  PointCloud::Ptr down_world_{new PointCloud};
  PointCloud::Ptr world_{new PointCloud};
  std::vector<PointWithCovariance> points_;
  pcl::VoxelGrid<Point> filter_;
  bool started_ = false;
  bool map_initialized_ = false;
  bool gravity_aligned_ = false;
  double first_lidar_time_ = 0.0;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_FRONTEND_FRONTEND_H_
