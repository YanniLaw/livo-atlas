#ifndef LIVO_ATLAS_TESTS_GAUSSIAN_RENDER_TEST_HELPERS_H_
#define LIVO_ATLAS_TESTS_GAUSSIAN_RENDER_TEST_HELPERS_H_

#include <string>

#include "gaussian/mapper_io.h"
#include "gaussian/render_protocol.h"
#include "gaussian/render_metadata_test_helpers.h"
#include "mapping/pose_math.h"

namespace livo_atlas {
namespace test {

inline GaussianRenderRequest RenderRequest() {
  GaussianRenderRequest request;
  request.session_id = std::string(32, 'a');
  request.job_id = 9007199254740993ULL;
  request.generation = 4;
  request.map_sequence = 9;
  request.frame_count = 6;
  request.query_keyframe_id = 5;
  request.query_submap_id = 2;
  request.parent_from_body.translation.x() = 5;
  for (std::uint64_t id : {1, 2}) {
    GaussianRenderBlock block;
    auto& target = block.target;
    target.session_id = request.session_id;
    target.generation = id + 1;
    target.frame_count = 6;
    target.group.primary = {id, 2 * id, {2 * id, 2 * id + 1}};
    target.group.membership_revision = 3;
    target.group.state = OnlineSubmapState::kSealed;
    target.parent_from_submap.translation.x() = 2 * id;
    target.submap_from_body.resize(2);
    target.submap_from_body[1].translation.x() = 1;
    block.content = {request.session_id, id, id + 1, id + 10};
    block.asset = GaussianAssetReference{id + 1, block.content,
                                        std::string(64, 'b'), 2048, 1024, 4096};
    block.frame_ids = target.group.primary.keyframe_ids;
    block.gaussian_count = 2;
    request.blocks.push_back(block);
  }
  return request;
}

inline GaussianRenderRequest SpatialRenderRequest() {
  auto request = RenderRequest();
  request.policy = GaussianRenderPolicy::kSpatial;
  request.candidate_count = 1;
  request.candidate_pixels = 48;
  request.selection_seconds = .001;
  auto& earlier = request.blocks.front();
  earlier.target.group.primary = {0, 0, {0, 1}};
  earlier.target.generation = 1;
  earlier.target.parent_from_submap.translation.x() = 0;
  earlier.content = {request.session_id, 0, 1, 10};
  earlier.asset->asset_id = 1;
  earlier.asset->content = earlier.content;
  earlier.frame_ids = {0, 1};
  for (auto& block : request.blocks)
    block.render_metadata = RenderMetadata(block.content, block.frame_ids);
  return request;
}

inline void CopyJson(cv::FileStorage& out, const cv::FileNode& node) {
  if (node.isMap()) {
    out << "{";
    for (const auto& child : node) {
      out << child.name();
      CopyJson(out, child);
    }
    out << "}";
  } else if (node.isSeq()) {
    out << "[";
    for (const auto& child : node) CopyJson(out, child);
    out << "]";
  } else if (node.isString()) {
    out << static_cast<std::string>(node);
  } else if (node.isInt()) {
    out << static_cast<int>(node);
  } else {
    out << static_cast<double>(node);
  }
}

inline GaussianMessage RenderReply(const GaussianMessage& message,
                                   bool oom = false) {
  using namespace gaussian_mapper_io;
  const auto request = DecodeGaussianRenderRequest(message);
  const auto camera_from_body =
      request.policy == GaussianRenderPolicy::kSpatial
          ? request.blocks.back().render_metadata->camera.camera_from_body
          : RigidTransform{};
  const double capture_time = request.policy == GaussianRenderPolicy::kSpatial
      ? request.blocks.back().render_metadata->camera.capture_time : 100.01;
  cv::FileStorage input(message.header_json,
                        cv::FileStorage::READ | cv::FileStorage::MEMORY);
  GaussianMessage reply;
  reply.header_json = Json([&](auto& out) {
    for (const auto& child : input.root()) {
      if (child.name() == "type") continue;
      out << child.name();
      CopyJson(out, child);
    }
    out << "type" << (oom ? "joint_render_error" : "joint_render_ok");
    if (oom) {
      out << "code" << "oom" << "error" << "Controlled allocation failure";
      return;
    }
    out << "training_view" << 1 << "width" << 8 << "height" << 6;
    Id(out, "calibration_id", 0);
    out << "fx" << 4. << "fy" << 4. << "cx" << 3.5 << "cy" << 2.5;
    Transform(out, "parent_from_body", request.parent_from_body);
    Transform(out, "camera_from_body", camera_from_body);
    Transform(out, "camera_from_parent",
              pose_math::Compose(camera_from_body,
                  pose_math::Inverse(request.parent_from_body)));
    out << "image_timestamp" << capture_time << "state_timestamp" << 100.
        << "cloud_timestamp" << 99.99 << "inverse_exposure_time" << 1.;
    for (const auto* key : {"single", "joint"})
      out << key << "{" << "image_pixels" << 48 << "covered_pixels" << 24
          << "depth_pixels_available" << 4 << "depth_pixels_used" << 2
          << "image_valid" << 1 << "depth_valid" << 1 << "psnr_valid" << 1
          << "rgb_perfect" << 0 << "rgb_l1" << .2 << "rgb_mse" << .04
          << "rgb_psnr_db" << 13.979400086720375 << "alpha_coverage" << .5
          << "depth_l1_m" << .1 << "depth_rmse_m" << .2 << "}";
    int count = 0;
    for (const auto& block : request.blocks) count += block.gaussian_count;
    out << "changed_pixels" << (request.blocks.size() == 2 ? 2 : 0)
        << "single_gaussian_count" << request.blocks.back().gaussian_count
        << "gaussian_count" << count << "worker_seconds" << .01;
    for (const auto* key : {"gpu_peak_allocated_bytes",
                            "gpu_peak_reserved_bytes", "gpu_allocated_bytes",
                            "gpu_reserved_bytes", "worker_peak_rss_bytes"})
      Id(out, key, 1024);
  });
  if (!oom) reply.body.assign(8 * 6 * 3, 40);
  return reply;
}

}  // namespace test
}  // namespace livo_atlas
#endif  // LIVO_ATLAS_TESTS_GAUSSIAN_RENDER_TEST_HELPERS_H_
