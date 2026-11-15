#include "livo_atlas/io/reconstruction_view.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

#include "io/colmap_camera.h"
#include "mapping/pose_math.h"

namespace livo_atlas {

ReconstructionView MakeReconstructionView(const CameraConfig& camera,
                                          const Keyframe& keyframe,
                                          const PoseSnapshot& poses,
                                          ReconstructionViewLimits limits) {
  if (!limits.max_pixels || !limits.max_cloud_points) {
    throw std::invalid_argument("Reconstruction view limits must be positive");
  }
  const auto camera_error = ValidateCameraConfig(camera);
  if (!camera_error.empty()) throw std::invalid_argument(camera_error);
  const int width = static_cast<int>(camera.width * camera.scale);
  const int height = static_cast<int>(camera.height * camera.scale);
  const auto pixels = static_cast<std::size_t>(width) * height;
  if (!keyframe.cloud) throw std::invalid_argument("Missing keyframe cloud");
  if (pixels > limits.max_pixels ||
      keyframe.cloud->size() > limits.max_cloud_points) {
    throw std::invalid_argument("Reconstruction view exceeds input limits");
  }
  ValidateKeyframe(keyframe);
  if (!keyframe.image_timestamp || keyframe.image.cols != width ||
      keyframe.image.rows != height) {
    throw std::invalid_argument(
        "Reconstruction view requires a calibrated image");
  }
  ValidatePoseSnapshot(poses);
  if (keyframe.id >= poses.frame_count) {
    throw std::invalid_argument("Pose snapshot does not cover keyframe");
  }
  const auto& parent_from_body = poses.poses[keyframe.id].map_from_body;
  if (poses.version == 0 &&
      !pose_math::Equal(parent_from_body, keyframe.odom_from_body)) {
    throw std::invalid_argument("Version zero must use the raw keyframe pose");
  }

  // One nearest sample per pixel makes the existing point limit lossless.
  const ColmapCamera rectifier(camera);
  auto projected = rectifier.MakeView(keyframe, parent_from_body,
                                      static_cast<std::uint32_t>(pixels));
  ReconstructionView result;
  result.keyframe_id = keyframe.id;
  result.calibration_id = keyframe.calibration_id;
  result.pose_version = poses.version;
  result.frame_count = poses.frame_count;
  result.optimized_frame_count = poses.optimized_frame_count;
  result.parent_frame = poses.parent_frame;
  result.state_timestamp = keyframe.timestamp;
  result.cloud_timestamp = keyframe.cloud_timestamp;
  result.image_timestamp = *keyframe.image_timestamp;
  result.inverse_exposure_time = keyframe.inverse_exposure_time;
  result.camera = rectifier.pinhole();
  result.camera_from_parent = projected.camera_from_parent;
  result.image_bgr = std::move(projected.image);
  result.image_mask = rectifier.valid_mask().clone();
  result.depth_m = cv::Mat::zeros(height, width, CV_32FC1);
  result.depth_mask = cv::Mat::zeros(height, width, CV_8UC1);
  for (const auto& sample : projected.samples) {
    const double z = (result.camera_from_parent.rotation * sample.position +
                      result.camera_from_parent.translation)
                         .z();
    if (!std::isfinite(z) || z <= 0 || z > std::numeric_limits<float>::max()) {
      throw std::overflow_error("Reconstruction depth is not representable");
    }
    const auto depth = static_cast<float>(z);
    if (!std::isfinite(depth) || depth <= 0) {
      throw std::overflow_error("Reconstruction depth is not representable");
    }
    const int x = static_cast<int>(sample.pixel.x());
    const int y = static_cast<int>(sample.pixel.y());
    result.depth_m.at<float>(y, x) = depth;
    result.depth_mask.at<unsigned char>(y, x) = 1;
  }
  result.valid_depth_pixels = projected.samples.size();
  return result;
}

}  // namespace livo_atlas
