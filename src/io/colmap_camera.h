#ifndef LIVO_ATLAS_SRC_IO_COLMAP_CAMERA_H_
#define LIVO_ATLAS_SRC_IO_COLMAP_CAMERA_H_

#include <cstdint>
#include <vector>

#include "livo_atlas/camera/camera_config.h"
#include "livo_atlas/mapping/keyframe.h"

namespace livo_atlas {
struct ColmapSample {
  Eigen::Vector3d position;
  Eigen::Vector2d pixel;
  cv::Vec3b rgb;  // RGB order, unlike the image's OpenCV BGR storage.
};
struct ColmapView {
  RigidTransform camera_from_parent;
  cv::Mat image;
  std::vector<ColmapSample> samples;
};

// Private offline rectification/projection; never changes frontend images.
class ColmapCamera {
 public:
  explicit ColmapCamera(const CameraConfig& config);
  // Already scaled; principal point and observations use COLMAP corner pixels.
  const CameraConfig& pinhole() const { return pinhole_; }
  ColmapView MakeView(const Keyframe& frame,
                      const RigidTransform& parent_from_body,
                      std::uint32_t max_points) const;

 private:
  CameraConfig pinhole_;
  cv::Mat map_x_, map_y_, valid_;
};
}  // namespace livo_atlas

#endif  // LIVO_ATLAS_SRC_IO_COLMAP_CAMERA_H_
