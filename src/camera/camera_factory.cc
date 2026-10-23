#include "livo_atlas/camera/camera_factory.h"

#include "livo_atlas/camera/equidistant_camera.h"
#include "livo_atlas/camera/pinhole_camera.h"

namespace livo_atlas {

std::unique_ptr<Camera> CreateCamera(const CameraConfig& config) {
  if (!ValidateCameraConfig(config).empty()) return nullptr;
  if (config.model == "Pinhole") {
    return std::make_unique<PinholeCamera>(config);
  }
  return std::make_unique<EquidistantCamera>(config);
}

}  // namespace livo_atlas
