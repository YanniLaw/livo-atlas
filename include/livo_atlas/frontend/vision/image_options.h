#ifndef LIVO_ATLAS_FRONTEND_IMAGE_OPTIONS_H_
#define LIVO_ATLAS_FRONTEND_IMAGE_OPTIONS_H_

#include <string>

#include "livo_atlas/camera/camera_config.h"
#include "livo_atlas/frontend/frontend_options.h"

namespace livo_atlas {
// Empty on success. Validate before constructing the visual frontend.
std::string ValidateImageOptions(const CameraConfig& camera,
                                 const VisualOptions& visual);
}  // namespace livo_atlas

#endif  // LIVO_ATLAS_FRONTEND_IMAGE_OPTIONS_H_
