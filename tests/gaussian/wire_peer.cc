#include "gaussian/wire_protocol.h"
#include "gaussian/submap_protocol.h"
#include "render_test_helpers.h"

#include <chrono>
#include <exception>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
  if (argc != 2 && argc != 3) return 2;
  try {
    const int fd = std::stoi(argv[1]);
    if (argc == 3 && std::string(argv[2]) == "--render") {
      const auto request = livo_atlas::test::RenderRequest();
      livo_atlas::WriteGaussianMessage(fd,
          livo_atlas::EncodeGaussianRenderRequest(request),
          std::chrono::seconds(2));
      const auto response =
          livo_atlas::ReadGaussianMessage(fd, std::chrono::seconds(2));
      const auto result = livo_atlas::DecodeGaussianRenderReply(response, request);
      return result.width == 8 && result.request.job_id == request.job_id ? 0 : 1;
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
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
