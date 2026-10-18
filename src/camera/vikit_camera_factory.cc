#include "livo_atlas/camera/vikit_camera_factory.h"

#include <vikit/equidistant_camera.h>
#include <vikit/pinhole_camera.h>

namespace livo_atlas {
namespace {

// vikit may be compiled with a different Eigen alignment than this package.
// Keep allocation and the virtual deleting destructor in the same target.
// This adds no data and retains the original camera implementation.
template <typename Camera>
class OwnedVikitCamera final : public Camera {
 public:
  using Camera::Camera;
};

}  // namespace

std::unique_ptr<vk::AbstractCamera> CreateVikitCamera(
    const CameraConfig& config) {
  if (!ValidateCameraConfig(config).empty()) {
    return nullptr;
  }
  const auto& distortion = config.distortion;
  if (config.model == "Pinhole") {
    return std::make_unique<OwnedVikitCamera<vk::PinholeCamera>>(
        config.width, config.height, config.scale, config.fx, config.fy,
        config.cx, config.cy, distortion[0], distortion[1], distortion[2],
        distortion[3]);
  }
  return std::make_unique<OwnedVikitCamera<vk::EquidistantCamera>>(
      config.width, config.height, config.scale, config.fx, config.fy,
      config.cx, config.cy, distortion[0], distortion[1], distortion[2],
      distortion[3]);
}

}  // namespace livo_atlas
