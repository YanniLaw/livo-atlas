#ifndef LIVO_ATLAS_SRC_ROS_GAUSSIAN_RENDER_MESSAGES_H_
#define LIVO_ATLAS_SRC_ROS_GAUSSIAN_RENDER_MESSAGES_H_

#include <Eigen/Geometry>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "livo_atlas/core/gaussian_render_output_options.h"
#include "livo_atlas/gaussian/gaussian_render_snapshot.h"
#include "mapping/pose_math.h"

namespace livo_atlas::ros_detail {

// Generated ROS1/ROS2 types have the same fields. Only time construction and
// native publication differ. Bad READY data with a usable header becomes a
// cleared ERROR; the native publisher reports that failure after publishing it.
template <typename Message, typename Stamp>
Message MakeGaussianRenderMessage(const GaussianRenderSnapshot& source,
                                  const std::string& optical_frame,
                                  const std::string& map_frame, Stamp stamp) {
  GaussianRenderOutputOptions options;
  options.enabled = true;
  options.optical_frame = optical_frame;
  ValidateGaussianRenderOutputOptions(options, map_frame);
  if (source.producer_id.size() != 32 || source.session_id.size() != 32 ||
      source.reason.size() > 64)
    throw std::invalid_argument("Invalid Gaussian publication identity");
  auto context = source;
  context.render.reset();
  if (source.status == GaussianRenderStatus::kReady) {
    context.status = GaussianRenderStatus::kError;
    context.reason = "invalid_snapshot";
  }
  ValidateGaussianRenderSnapshotHeader(context, true);
  const auto cleared = [&] {
    Message result;
    result.header.frame_id = optical_frame;
    result.header.stamp = stamp(0);
    result.producer_id = source.producer_id;
    result.revision = source.revision;
    result.session_id = source.session_id;
    result.status = static_cast<std::uint8_t>(source.status);
    result.reason =
        source.status == GaussianRenderStatus::kReady ? "" : source.reason;
    result.generation = source.generation;
    result.map_sequence = source.map_sequence;
    result.pose_version = source.pose_version;
    result.frame_count = source.frame_count;
    result.optimized_frame_count = source.optimized_frame_count;
    result.parent_frame =
        source.parent_frame == PoseFrame::kOdom ? "camera_init" : map_frame;
    result.policy = source.policy == GaussianRenderPolicy::kSpatial
                        ? "spatial"
                        : "temporal";
    std::fill(result.k.begin(), result.k.end(), 0);
    result.parent_from_camera.orientation.w = 1;
    result.image.header = result.header;
    return result;
  };
  Message message = cleared();
  try {
    ValidateGaussianRenderSnapshot(source);
    if (source.status != GaussianRenderStatus::kReady) return message;
    const auto& render = *source.render;
    const auto& request = render.request;
    const auto ns = GaussianRenderNanoseconds(render.capture_time);
    message.header.stamp = stamp(ns);
    message.image.header = message.header;
    message.job_id = request.job_id;
    message.query_keyframe_id = request.query_keyframe_id;
    message.query_submap_id = request.query_submap_id;
    message.neighbor_reason = request.neighbor_reason;
    message.sources.resize(request.blocks.size());
    for (std::size_t i = 0; i < request.blocks.size(); ++i) {
      const auto& block = request.blocks[i];
      auto& output = message.sources[i];
      output.submap_id = block.content.submap_id;
      output.binding_generation = block.target.generation;
      output.source_generation = block.content.source_generation;
      output.content_job_id = block.content.content_job_id;
      output.membership_revision = block.target.group.membership_revision;
      output.resident = block.resident;
      if (block.asset) {
        output.asset_id = block.asset->asset_id;
        output.manifest_sha256 = block.asset->manifest_sha256;
      }
      output.frame_ids = block.frame_ids;
      output.gaussian_count = block.gaussian_count;
    }
    message.calibration_id = render.calibration_id;
    message.capture_time = render.capture_time;
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 3; ++col)
        message.k[3 * row + col] = render.K(row, col);
    const auto pose = pose_math::Inverse(render.camera_from_parent);
    const auto rotation = Eigen::Quaterniond(pose.rotation).normalized();
    if (!pose.translation.allFinite() || !rotation.coeffs().allFinite())
      throw std::invalid_argument("Nonfinite displayed Gaussian pose");
    auto& output = message.parent_from_camera;
    output.position.x = pose.translation.x();
    output.position.y = pose.translation.y();
    output.position.z = pose.translation.z();
    output.orientation.w = rotation.w();
    output.orientation.x = rotation.x();
    output.orientation.y = rotation.y();
    output.orientation.z = rotation.z();
    message.changed_pixels = render.changed_pixels;
    message.candidate_count = request.candidate_count;
    message.candidate_pixels = request.candidate_pixels;
    message.projection_uncertain = request.projection_uncertain;
    message.image.width = render.width;
    message.image.height = render.height;
    message.image.encoding = "rgb8";
    message.image.is_bigendian = false;
    message.image.step = 3 * render.width;
    message.image.data = render.rgb;
  } catch (...) {
    message = cleared();
    message.status = Message::ERROR;
    message.reason = "invalid_snapshot";
  }
  return message;
}

}  // namespace livo_atlas::ros_detail
#endif  // LIVO_ATLAS_SRC_ROS_GAUSSIAN_RENDER_MESSAGES_H_
