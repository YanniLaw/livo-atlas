#ifndef LIVO_ATLAS_FRONTEND_FRONTEND_RESULT_H_
#define LIVO_ATLAS_FRONTEND_FRONTEND_RESULT_H_

#include <vector>

#include <Eigen/Geometry>

#include "livo_atlas/core/state.h"
#include "livo_atlas/frontend/diagnostics.h"
#include "livo_atlas/frontend/synchronized_measurements.h"

namespace livo_atlas {

struct FrontendResult {
  UpdateKind kind = UpdateKind::kWaiting;
  bool first_frame = false;
  bool updated = false;
  double timestamp = 0.0;
  double relative_time = 0.0;
  State prediction;
  State estimate;
  // Read-only views valid until the next Process call. States are values.
  PointCloud::ConstPtr world_cloud;
  PointCloud::ConstPtr undistorted_cloud;
  PointCloud::ConstPtr effective_cloud;
  std::vector<PlaneVisualization> planes;
  bool has_plane_visualization = false;
  bool imu_initialized = false;
  std::vector<ImuDiagnostic> imu_diagnostics;
  cv::Mat color_image;
  cv::Mat tracking_image;
  Eigen::Quaterniond camera_from_world_rotation =
      Eigen::Quaterniond::Identity();
  Eigen::Vector3d camera_from_world_translation = Eigen::Vector3d::Zero();
  std::vector<DebugImage> debug_images;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_FRONTEND_FRONTEND_RESULT_H_
