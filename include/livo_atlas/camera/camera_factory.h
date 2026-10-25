#ifndef LIVO_ATLAS_CAMERA_CAMERA_FACTORY_H_
#define LIVO_ATLAS_CAMERA_CAMERA_FACTORY_H_

#include <memory>

#include "livo_atlas/camera/camera.h"

namespace livo_atlas {

// Invalid calibration returns nullptr. Allocation or OpenCV failures may throw.
std::unique_ptr<Camera> CreateCamera(const CameraConfig& config);

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_CAMERA_CAMERA_FACTORY_H_
