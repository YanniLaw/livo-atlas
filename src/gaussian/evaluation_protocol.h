#ifndef LIVO_ATLAS_SRC_GAUSSIAN_EVALUATION_PROTOCOL_H_
#define LIVO_ATLAS_SRC_GAUSSIAN_EVALUATION_PROTOCOL_H_

#include "gaussian/wire_protocol.h"
#include "livo_atlas/gaussian/gaussian_evaluation.h"
#include "livo_atlas/io/reconstruction_view.h"

namespace livo_atlas {

GaussianMessage EncodeGaussianEvaluation(
    const GaussianEvaluationRequest& request, const ReconstructionView& view,
    const RigidTransform& camera_from_body);
void ValidateGaussianEvaluationMessage(const cv::FileNode& root,
                                       const std::vector<std::uint8_t>& body);
std::string CheckGaussianEvaluationReply(
    const GaussianMessage& message, const GaussianEvaluationRequest& request);

}  // namespace livo_atlas
#endif  // LIVO_ATLAS_SRC_GAUSSIAN_EVALUATION_PROTOCOL_H_
