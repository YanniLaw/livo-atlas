#ifndef LIVO_ATLAS_SRC_GAUSSIAN_SUBMAP_PROTOCOL_H_
#define LIVO_ATLAS_SRC_GAUSSIAN_SUBMAP_PROTOCOL_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include "livo_atlas/gaussian/gaussian_map.h"

namespace livo_atlas {

std::vector<std::uint8_t> EncodeGaussianPrimaryPoses(
    const GaussianTarget& target);
void WriteGaussianTargetHeader(cv::FileStorage& json,
                               const GaussianTarget& target);
GaussianTarget DecodeGaussianTarget(const cv::FileNode& header,
                                    const std::vector<std::uint8_t>& body,
                                    std::size_t offset = 0);
void ValidateGaussianSubmapMessage(const cv::FileNode& header,
                                   const std::vector<std::uint8_t>& body);

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_SRC_GAUSSIAN_SUBMAP_PROTOCOL_H_
