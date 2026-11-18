#ifndef LIVO_ATLAS_SRC_GAUSSIAN_RENDER_PROTOCOL_H_
#define LIVO_ATLAS_SRC_GAUSSIAN_RENDER_PROTOCOL_H_

#include <opencv2/core.hpp>

#include "gaussian/wire_protocol.h"
#include "livo_atlas/gaussian/gaussian_render.h"

namespace livo_atlas {

GaussianMessage EncodeGaussianRenderRequest(
    const GaussianRenderRequest& request);
GaussianRenderRequest DecodeGaussianRenderRequest(
    const GaussianMessage& message);
GaussianRender DecodeGaussianRenderReply(
    const GaussianMessage& message, const GaussianRenderRequest& request);
std::string DecodeGaussianRenderError(
    const GaussianMessage& message, const GaussianRenderRequest& request);
void ValidateGaussianRenderMessage(const cv::FileNode& header,
                                   const std::vector<std::uint8_t>& body);

}  // namespace livo_atlas
#endif  // LIVO_ATLAS_SRC_GAUSSIAN_RENDER_PROTOCOL_H_
