#ifndef LIVO_ATLAS_SRC_GAUSSIAN_RENDER_VALIDATION_H_
#define LIVO_ATLAS_SRC_GAUSSIAN_RENDER_VALIDATION_H_

#include "livo_atlas/gaussian/gaussian_render.h"

namespace livo_atlas {

// Render DTOs, wire messages and files share this stricter boundary. Frontend
// calibration keeps its existing tolerance.
void ValidateRenderPose(const RigidTransform& pose);
void ValidateRenderCamera(const GaussianCamera& camera);
void ValidateRenderSelection(const GaussianRenderRequest& request);
void ValidateRenderMetrics(const GaussianRenderMetrics& metrics, int pixels);
void ValidateRenderImage(const GaussianRender& render);

}  // namespace livo_atlas
#endif  // LIVO_ATLAS_SRC_GAUSSIAN_RENDER_VALIDATION_H_
