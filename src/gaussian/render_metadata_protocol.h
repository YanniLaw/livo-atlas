#ifndef LIVO_ATLAS_SRC_GAUSSIAN_RENDER_METADATA_PROTOCOL_H_
#define LIVO_ATLAS_SRC_GAUSSIAN_RENDER_METADATA_PROTOCOL_H_

#include <opencv2/core.hpp>

#include "livo_atlas/gaussian/gaussian_map.h"

namespace livo_atlas {

void WriteGaussianRenderMetadata(cv::FileStorage& out,
                                 const GaussianRenderMetadata& metadata);
GaussianRenderMetadata DecodeGaussianRenderMetadata(
    const cv::FileNode& node, const GaussianContentId& content,
    const std::vector<std::uint64_t>& frames);

}  // namespace livo_atlas
#endif  // LIVO_ATLAS_SRC_GAUSSIAN_RENDER_METADATA_PROTOCOL_H_
