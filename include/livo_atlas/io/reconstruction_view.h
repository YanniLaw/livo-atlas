#ifndef LIVO_ATLAS_IO_RECONSTRUCTION_VIEW_H_
#define LIVO_ATLAS_IO_RECONSTRUCTION_VIEW_H_

#include <cstddef>
#include <cstdint>

#include <opencv2/core/mat.hpp>

#include "livo_atlas/camera/camera_config.h"
#include "livo_atlas/mapping/keyframe.h"
#include "livo_atlas/mapping/pose_snapshot.h"
#include "livo_atlas/sensor/extrinsics.h"

namespace livo_atlas {

struct ReconstructionViewLimits {
  std::size_t max_pixels = 0;
  std::size_t max_cloud_points = 0;
};

// Camera intrinsics use integer pixel centers. Masks contain 0 or 1; depth is
// camera z in meters and is meaningful only where depth_mask is 1.
struct ReconstructionView {
  std::uint64_t keyframe_id = 0;
  std::uint64_t calibration_id = 0;
  std::uint64_t pose_version = 0;
  std::uint64_t frame_count = 0;
  std::uint64_t optimized_frame_count = 0;
  PoseFrame parent_frame = PoseFrame::kOdom;
  double state_timestamp = 0;
  double cloud_timestamp = 0;
  double image_timestamp = 0;
  double inverse_exposure_time = 1;
  CameraConfig camera;
  RigidTransform camera_from_parent;
  cv::Mat image_bgr;
  cv::Mat image_mask;
  cv::Mat depth_m;
  cv::Mat depth_mask;
  std::size_t valid_depth_pixels = 0;
};

// The caller supplies the calibration for keyframe.calibration_id. Returned
// matrices own their storage; copied cv::Mat values must be treated as
// read-only.
ReconstructionView MakeReconstructionView(const CameraConfig& camera,
                                          const Keyframe& keyframe,
                                          const PoseSnapshot& poses,
                                          ReconstructionViewLimits limits);

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_IO_RECONSTRUCTION_VIEW_H_
