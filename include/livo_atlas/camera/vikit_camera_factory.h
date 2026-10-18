#ifndef LIVO_ATLAS_CAMERA_VIKIT_CAMERA_FACTORY_H_
#define LIVO_ATLAS_CAMERA_VIKIT_CAMERA_FACTORY_H_

#include <memory>

#include <vikit/abstract_camera.h>

#include "livo_atlas/camera/camera_config.h"

namespace livo_atlas {

// Temporary bridge until the camera implementation is migrated in P3.
// Returns nullptr for invalid calibration; vikit construction may throw.
std::unique_ptr<vk::AbstractCamera> CreateVikitCamera(
    const CameraConfig& config);

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_CAMERA_VIKIT_CAMERA_FACTORY_H_
