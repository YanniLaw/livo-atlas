#include <chrono>
#include <exception>
#include <string>

#include "gaussian/evaluation_protocol.h"
#include "gaussian/submap_protocol.h"
#include "gaussian/wire_protocol.h"
#include "livo_atlas/common/logging.h"
#include "render_test_helpers.h"

int main(int argc, char** argv) {
  livo_atlas::InitializeLogging(argv[0]);
  if (argc != 2 && argc != 3) return 2;
  try {
    const int fd = std::stoi(argv[1]);
    if (argc == 3 && std::string(argv[2]) == "--evaluation") {
      livo_atlas::GaussianEvaluationRequest request;
      request.sources = livo_atlas::test::RenderRequest();
      request.sources.query_keyframe_id = 4;
      request.sources.parent_from_body.translation.x() = 4;
      request.sources.blocks.back().frame_ids = {5};
      request.sources.blocks.back().gaussian_count = 1;
      request.camera.keyframe_id = 4;
      request.split_id = std::string(64, 'a');
      request.sealed_at_unix_seconds = 1;
      request.dispatched_at_unix_seconds = 2;
      livo_atlas::ReconstructionView view;
      view.keyframe_id = 4;
      view.camera = {"Pinhole", 8, 6, 1, 4, 4, 3.5, 2.5, {}};
      view.image_bgr = cv::Mat(6, 8, CV_8UC3, cv::Scalar(40, 40, 40));
      view.image_mask = cv::Mat::ones(6, 8, CV_8UC1);
      view.depth_mask = cv::Mat::zeros(6, 8, CV_8UC1);
      view.depth_m = cv::Mat::zeros(6, 8, CV_32FC1);
      livo_atlas::WriteGaussianMessage(
          fd, livo_atlas::EncodeGaussianEvaluation(request, view, {}),
          std::chrono::seconds(2));
      const auto response =
          livo_atlas::ReadGaussianMessage(fd, std::chrono::seconds(2));
      return livo_atlas::CheckGaussianEvaluationReply(response, request).empty()
                 ? 0
                 : 1;
    }
    if (argc == 3 && (std::string(argv[2]) == "--render" ||
                      std::string(argv[2]) == "--spatial-render")) {
      const auto request = std::string(argv[2]) == "--spatial-render"
                               ? livo_atlas::test::SpatialRenderRequest()
                               : livo_atlas::test::RenderRequest();
      livo_atlas::WriteGaussianMessage(
          fd, livo_atlas::EncodeGaussianRenderRequest(request),
          std::chrono::seconds(2));
      const auto response =
          livo_atlas::ReadGaussianMessage(fd, std::chrono::seconds(2));
      const auto result =
          livo_atlas::DecodeGaussianRenderReply(response, request);
      return result.width == 8 && result.request.job_id == request.job_id ? 0
                                                                          : 1;
    }
    if (argc == 3) {
      if (std::string(argv[2]) != "--target") return 2;
      livo_atlas::GaussianTarget target;
      target.session_id = std::string(32, 'a');
      target.generation = 1;
      target.frame_count = 512;
      target.group.membership_revision = 512;
      target.parent_from_submap.rotation << 0, -1, 0, 1, 0, 0, 0, 0, 1;
      target.parent_from_submap.translation = {4, -2, 1};
      for (int i = 0; i < 512; ++i) {
        target.group.primary.keyframe_ids.push_back(i);
        livo_atlas::RigidTransform pose;
        pose.translation.x() = i;
        target.submap_from_body.push_back(pose);
      }
      cv::FileStorage json("", cv::FileStorage::WRITE |
                                   cv::FileStorage::MEMORY |
                                   cv::FileStorage::FORMAT_JSON);
      json << "schema" << 2 << "type"
           << "activate"
           << "session_id" << target.session_id << "job_id"
           << "9007199254740993"
           << "action"
           << "empty";
      livo_atlas::WriteGaussianTargetHeader(json, target);
      const livo_atlas::GaussianMessage outgoing{
          json.releaseAndGetString(),
          livo_atlas::EncodeGaussianPrimaryPoses(target)};
      livo_atlas::WriteGaussianMessage(fd, outgoing, std::chrono::seconds(2));
      const auto incoming =
          livo_atlas::ReadGaussianMessage(fd, std::chrono::seconds(2));
      return incoming.body == outgoing.body ? 0 : 1;
    }
    const auto message =
        livo_atlas::ReadGaussianMessage(fd, std::chrono::seconds(2));
    livo_atlas::WriteGaussianMessage(fd, message, std::chrono::seconds(2));
  } catch (const std::exception& error) {
    LOG(ERROR) << error.what();
    return 1;
  }
  return 0;
}
