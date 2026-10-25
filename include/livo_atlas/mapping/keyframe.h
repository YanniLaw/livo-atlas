#ifndef LIVO_ATLAS_MAPPING_KEYFRAME_H_
#define LIVO_ATLAS_MAPPING_KEYFRAME_H_

#include <cstdint>
#include <opencv2/core/mat.hpp>
#include <optional>

#include "livo_atlas/sensor/extrinsics.h"
#include "livo_atlas/sensor/point_types.h"

namespace livo_atlas {
struct FrontendResult;

// An owned snapshot. Cloud XYZ is in the anchor's IMU/body frame; intensity
// is retained. Consumers share these assets as read-only data.
struct Keyframe {
  std::uint64_t id = 0;
  std::uint64_t calibration_id = 0;
  double timestamp = 0;
  double cloud_timestamp = 0;
  std::optional<double> image_timestamp;
  RigidTransform odom_from_body;
  std::optional<RigidTransform> camera_from_odom;
  double inverse_exposure_time = 1;
  cv::Mat image;
  PointCloud::ConstPtr cloud;
};

// Throws before returning a snapshot for malformed data. Only selected frames
// should be captured; no source image or cloud buffer is retained.
Keyframe CaptureKeyframe(std::uint64_t id, const FrontendResult& result);
void ValidateKeyframe(const Keyframe& keyframe);

// Rebuild geometry under a chosen pose without changing the original snapshot.
PointCloud TransformKeyframeCloud(const Keyframe& keyframe,
                                  const RigidTransform& parent_from_body);
}  // namespace livo_atlas

#endif  // LIVO_ATLAS_MAPPING_KEYFRAME_H_
