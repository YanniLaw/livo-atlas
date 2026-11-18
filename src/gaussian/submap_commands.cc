#include "livo_atlas/gaussian/online_gaussian_mapper.h"

#include <algorithm>
#include <stdexcept>

#include "gaussian/mapper_io.h"
#include "gaussian/render_metadata_protocol.h"
#include "gaussian/render_protocol.h"
#include "gaussian/submap_protocol.h"
#include "livo_atlas/io/gaussian_map_writer.h"
#include "livo_atlas/io/reconstruction_view.h"
#include "mapping/pose_math.h"

namespace livo_atlas {
namespace {
using gaussian_mapper_io::Base;
using gaussian_mapper_io::Id;
using gaussian_mapper_io::Json;
using gaussian_mapper_io::Parse;
using gaussian_mapper_io::ReadId;
using gaussian_mapper_io::Transform;
using gaussian_mapper_io::ViewBody;

const char* Operation(GaussianOperation operation) {
  switch (operation) {
    case GaussianOperation::kView:
      return "view";
    case GaussianOperation::kReuse:
      return "reuse";
    case GaussianOperation::kRebuild:
      return "rebuild";
  }
  throw std::invalid_argument("Unknown Gaussian operation");
}

std::vector<std::uint64_t> Frames(const cv::FileNode& values) {
  std::vector<std::uint64_t> result;
  for (const auto& value : values) result.push_back(ReadId(value));
  return result;
}

void CheckContent(const cv::FileStorage& json,
                  const GaussianContentId& content) {
  if (ReadId(json["source_generation"]) != content.source_generation ||
      ReadId(json["content_job_id"]) != content.content_job_id)
    throw std::runtime_error("Gaussian confirmation content identity mismatch");
}

}  // namespace

OnlineGaussianMapper::Reply OnlineGaussianMapper::RunSubmapCommand(
    int fd, const Command& command) {
  Reply reply;
  reply.command = command;
  if (command.kind == CommandKind::kRender) {
    const auto& request = *command.render_request;
    WriteGaussianMessage(fd, EncodeGaussianRenderRequest(request),
                         options_.transfer_timeout);
    const auto response =
        ReadGaussianMessage(fd, options_.joint_render_timeout);
    cv::FileStorage json(response.header_json,
                         cv::FileStorage::READ | cv::FileStorage::MEMORY);
    if (static_cast<std::string>(json["type"]) == "joint_render_error") {
      reply.detail = DecodeGaussianRenderError(response, request);
      reply.kind = ReplyKind::kRenderError;
    } else {
      reply.render = std::make_shared<GaussianRender>(
          DecodeGaussianRenderReply(response, request));
      reply.kind = ReplyKind::kRendered;
    }
    return reply;
  }
  if (command.kind == CommandKind::kWriteMap) {
    WriteGaussianMapIndex(options_.output_directory, *command.map);
    reply.kind = ReplyKind::kMapWritten;
    return reply;
  }
  const auto started = Clock::now();
  GaussianMessage request;
  std::string type, response_type;
  switch (command.kind) {
    case CommandKind::kActivate:
      type = "activate";
      response_type = command.content ? "candidate" : "activated";
      request.body = EncodeGaussianPrimaryPoses(command.target);
      break;
    case CommandKind::kView:
      type = "view";
      response_type = "candidate";
      break;
    case CommandKind::kDecision:
      type = "decision";
      response_type = command.accept ? "commit_ok" : "discard_ok";
      break;
    case CommandKind::kCheckpoint:
      type = "checkpoint";
      response_type = "checkpoint_ok";
      break;
    case CommandKind::kMapCommitted:
      type = "map_committed";
      response_type = "map_committed_ok";
      break;
    case CommandKind::kPrune:
      type = "prune";
      response_type = "prune_ok";
      break;
    default:
      throw std::runtime_error("Unexpected Gaussian submap command");
  }

  std::optional<ReconstructionView> view;
  if (command.kind == CommandKind::kView) {
    const auto& frame = *command.source.keyframe;
    reply.kind = ReplyKind::kSkipped;
    if (frame.image.empty() || !frame.image_timestamp ||
        !frame.camera_from_odom) {
      reply.detail = "no_image";
      return reply;
    }
    if (!frame.cloud || frame.cloud->size() > 100000 ||
        frame.image.total() > 1000000) {
      reply.detail = "limit";
      return reply;
    }
    view = MakeReconstructionView(camera_, frame, *command.source.poses,
                                  {1000000, 100000});
    if (!view->valid_depth_pixels) {
      reply.detail = "no_depth";
      return reply;
    }
    request.body = ViewBody(*view);
    const auto primary = EncodeGaussianPrimaryPoses(command.target);
    request.body.insert(request.body.end(), primary.begin(), primary.end());
  }
  const bool scoped = command.kind == CommandKind::kActivate ||
                      command.kind == CommandKind::kView ||
                      command.kind == CommandKind::kDecision ||
                      command.kind == CommandKind::kCheckpoint;
  const auto generation =
      scoped ? command.target.generation : command.generation;
  const auto version =
      command.map ? command.map->pose_version
                  : scoped ? command.target.pose_version : command.pose_version;
  request.header_json = Json([&](auto& out) {
    Base(out, type, session_id_, 2);
    Id(out, "job_id", command.job_id);
    if (command.kind == CommandKind::kActivate ||
        command.kind == CommandKind::kView) {
      WriteGaussianTargetHeader(out, command.target);
    } else {
      Id(out, "generation", generation);
      Id(out, "pose_version", version);
      if (scoped) Id(out, "submap_id", command.target.group.primary.id);
    }
    if (command.kind == CommandKind::kActivate) {
      out << "action"
          << (command.content ? Operation(command.operation) : "empty");
      if (command.content) {
        Id(out, "source_generation", command.content->source_generation);
        Id(out, "content_job_id", command.content->content_job_id);
      }
      if (command.asset) {
        Id(out, "asset_id", command.asset->asset_id);
        out << "manifest_sha256" << command.asset->manifest_sha256;
      }
    } else if (command.kind == CommandKind::kView) {
      const auto& frame = *command.source.keyframe;
      Id(out, "keyframe_id", view->keyframe_id);
      Id(out, "calibration_id", view->calibration_id);
      out << "state_timestamp" << view->state_timestamp << "cloud_timestamp"
          << view->cloud_timestamp << "image_timestamp" << view->image_timestamp
          << "inverse_exposure_time" << view->inverse_exposure_time << "width"
          << view->image_bgr.cols << "height" << view->image_bgr.rows << "fx"
          << view->camera.fx << "fy" << view->camera.fy << "cx"
          << view->camera.cx << "cy" << view->camera.cy;
      // C_B is measured independently; deriving it from target S_B would make
      // the camera/pose consistency check circular. Match ColmapCamera's
      // temporary rotation projection for rounded calibration, leaving raw
      // poses intact.
      Transform(
          out, "camera_from_body",
          pose_math::Compose(pose_math::ProjectPose(*frame.camera_from_odom),
                             pose_math::ProjectPose(frame.odom_from_body)));
      Transform(out, "camera_from_submap",
                pose_math::Compose(view->camera_from_parent,
                                   command.target.parent_from_submap));
    } else if (command.kind == CommandKind::kDecision) {
      out << "operation" << Operation(command.operation) << "action"
          << (command.accept ? "accept" : "discard");
    } else if (command.kind == CommandKind::kCheckpoint) {
      Id(out, "asset_id", command.asset_id);
      Id(out, "source_generation", command.content->source_generation);
      Id(out, "content_job_id", command.content->content_job_id);
    } else if (command.kind == CommandKind::kMapCommitted) {
      Id(out, "map_sequence", command.map->map_sequence);
      out << "asset_ids"
          << "[";
      for (const auto id : command.asset_ids) out << std::to_string(id);
      out << "]";
    } else if (command.kind == CommandKind::kPrune) {
      Id(out, "map_sequence", command.state_sequence);
      Id(out, "asset_id", command.asset_id);
    }
  });
  WriteGaussianMessage(fd, request, options_.transfer_timeout);
  const auto response = ReadGaussianMessage(fd, options_.job_timeout);
  const auto json = Parse(response, response_type, session_id_, 2);
  if (static_cast<int>(json["schema"]) != 2 ||
      ReadId(json["job_id"]) != command.job_id ||
      ReadId(json["generation"]) != generation ||
      ReadId(json["pose_version"]) != version ||
      (scoped && ReadId(json["submap_id"]) != command.target.group.primary.id))
    throw std::runtime_error("Gaussian submap reply identity mismatch");
  if (response_type == "candidate" || command.kind == CommandKind::kDecision) {
    if (static_cast<std::string>(json["operation"]) !=
        Operation(command.operation))
      throw std::runtime_error("Gaussian operation mismatch");
  }
  if (response_type == "candidate") {
    auto preview = std::make_shared<GaussianPreview>();
    preview->session_id = session_id_;
    preview->job_id = command.job_id;
    preview->generation = command.target.generation;
    preview->submap_id = command.target.group.primary.id;
    preview->pose_version = command.target.pose_version;
    preview->membership_revision = command.target.group.membership_revision;
    preview->frame_count = command.target.frame_count;
    preview->optimized_frame_count = command.target.optimized_frame_count;
    preview->parent_from_submap = command.target.parent_from_submap;
    preview->operation = command.operation;
    preview->frame_ids = Frames(json["frame_ids"]);
    if (preview->frame_ids != command.frame_ids ||
        ReadId(json["membership_revision"]) != preview->membership_revision ||
        ReadId(json["frame_count"]) != preview->frame_count ||
        ReadId(json["optimized_frame_count"]) != preview->optimized_frame_count)
      throw std::runtime_error(
          "Gaussian candidate coverage or frontier mismatch");
    preview->width = static_cast<int>(json["width"]);
    preview->height = static_cast<int>(json["height"]);
    if (view && (preview->width != view->image_bgr.cols ||
                 preview->height != view->image_bgr.rows))
      throw std::runtime_error("Gaussian candidate image dimensions mismatch");
    preview->capture_time = static_cast<double>(json["capture_time"]);
    const bool spatial =
        options_.joint_render_policy == GaussianRenderPolicy::kSpatial;
    if (spatial != !json["render_metadata"].empty())
      throw std::runtime_error("Gaussian candidate spatial metadata mismatch");
    if (spatial) {
      const auto content = command.operation == GaussianOperation::kReuse
          ? *command.content
          : GaussianContentId{session_id_, command.target.group.primary.id,
                              command.target.generation, command.job_id};
      const auto metadata = DecodeGaussianRenderMetadata(
          json["render_metadata"], content, preview->frame_ids);
      if (command.operation == GaussianOperation::kReuse &&
          command.render_metadata && *command.render_metadata != metadata)
        throw std::runtime_error("Gaussian reuse changed spatial metadata");
      reply.command.render_metadata = metadata;
    }
    preview->steps = static_cast<int>(json["steps"]);
    preview->gaussian_count = static_cast<int>(json["gaussian_count"]);
    if (command.operation == GaussianOperation::kReuse &&
        preview->gaussian_count != command.gaussian_count)
      throw std::runtime_error("Gaussian reuse changed parameter count");
    preview->supervised_depth_pixels =
        static_cast<int>(json["supervised_depth_pixels"]);
    preview->rgb_l1 = static_cast<double>(json["rgb_l1"]);
    preview->depth_l1_m = static_cast<double>(json["depth_l1_m"]);
    preview->nonzero_gradient = static_cast<int>(json["nonzero_gradient"]) != 0;
    preview->parameter_updated =
        static_cast<int>(json["parameter_updated"]) != 0;
    for (auto value :
         {std::pair<const char*, std::uint64_t*>{
              "gpu_peak_allocated_bytes", &preview->gpu_peak_allocated_bytes},
          {"gpu_peak_reserved_bytes", &preview->gpu_peak_reserved_bytes},
          {"worker_peak_rss_bytes", &preview->worker_peak_rss_bytes}})
      if (!json[value.first].empty()) *value.second = ReadId(json[value.first]);
    preview->worker_seconds =
        std::chrono::duration<double>(Clock::now() - started).count();
    preview->rgb = response.body;
    reply.preview = std::move(preview);
    reply.kind = ReplyKind::kCandidate;
  } else if (command.kind == CommandKind::kDecision) {
    if (command.accept) {
      const auto content =
          command.operation == GaussianOperation::kReuse
              ? *command.content
              : GaussianContentId{session_id_, command.target.group.primary.id,
                                  command.target.generation, command.job_id};
      CheckContent(json, content);
      if (Frames(json["frame_ids"]) != command.preview->frame_ids ||
          static_cast<int>(json["gaussian_count"]) !=
              command.preview->gaussian_count)
        throw std::runtime_error("Gaussian commit changed confirmed coverage");
    }
    reply.kind = ReplyKind::kConfirmed;
  } else if (command.kind == CommandKind::kCheckpoint) {
    CheckContent(json, *command.content);
    if (ReadId(json["asset_id"]) != command.asset_id ||
        Frames(json["frame_ids"]) != command.frame_ids ||
        static_cast<int>(json["gaussian_count"]) != command.gaussian_count)
      throw std::runtime_error(
          "Gaussian checkpoint differs from confirmed content");
    reply.asset = GaussianAssetReference{
        command.asset_id,
        *command.content,
        static_cast<std::string>(json["manifest_sha256"]),
        ReadId(json["manifest_bytes"]),
        ReadId(json["gaussian_bytes"]),
        ReadId(json["observation_bytes"])};
    reply.kind = ReplyKind::kCheckpoint;
  } else if (command.kind == CommandKind::kMapCommitted) {
    if (ReadId(json["map_sequence"]) != command.map->map_sequence ||
        Frames(json["asset_ids"]) != command.asset_ids)
      throw std::runtime_error(
          "Gaussian map acknowledgement differs from durable references");
    reply.kind = ReplyKind::kMapCommitted;
  } else if (command.kind == CommandKind::kPrune) {
    if (ReadId(json["map_sequence"]) != command.state_sequence ||
        ReadId(json["asset_id"]) != command.asset_id)
      throw std::runtime_error("Gaussian prune acknowledgement mismatch");
    reply.kind = ReplyKind::kPruned;
  } else {
    reply.kind = ReplyKind::kActivated;
  }
  return reply;
}

}  // namespace livo_atlas
