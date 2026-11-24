#include "livo_atlas/gaussian/online_gaussian_mapper.h"

#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <opencv2/core/persistence.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <random>
#include <stdexcept>
#include <system_error>
#include <utility>

#include "gaussian/mapper_io.h"
#include "gaussian/wire_protocol.h"
#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/io/reconstruction_view.h"
#include "livo_atlas/io/gaussian_render_writer.h"
#include "mapping/pose_math.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

constexpr std::uint64_t kMaxFrames = 512;
constexpr std::size_t kMaxPixels = 1000000;
constexpr std::size_t kMaxPoints = 100000;
constexpr std::size_t kMaxPreviews = 8;
constexpr std::array<double, 7> kLatencyBounds = {0.1, 0.25, 0.5, 1, 2, 5, 10};

std::string NewSessionId() {
  std::random_device random;
  constexpr char kHex[] = "0123456789abcdef";
  std::string result;
  result.reserve(32);
  for (int i = 0; i < 4; ++i) {
    const std::uint32_t value = random();
    for (int shift = 28; shift >= 0; shift -= 4)
      result.push_back(kHex[(value >> shift) & 15]);
  }
  return result;
}

using gaussian_mapper_io::Base;
using gaussian_mapper_io::CheckIdentity;
using gaussian_mapper_io::Id;
using gaussian_mapper_io::Json;
using gaussian_mapper_io::Parse;
using gaussian_mapper_io::PreviewIdentity;
using gaussian_mapper_io::ReadId;
using gaussian_mapper_io::Transform;
using gaussian_mapper_io::ViewBody;
using gaussian_mapper_io::WriteFile;

double Seconds(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

}  // namespace

OnlineGaussianMapper::OnlineGaussianMapper(LivoSystem& system,
                                           CameraConfig camera,
                                           OnlineGaussianOptions options)
    : system_(system),
      camera_(std::move(camera)),
      options_(std::move(options)),
      session_id_(NewSessionId()),
      map_index_(session_id_) {
  ValidateOnlineGaussianOptions(options_);
  if (!options_.enabled) return;
  const auto camera_error = ValidateCameraConfig(camera_);
  if (!camera_error.empty()) throw std::invalid_argument(camera_error);
  OnlineSubmapDelta initial;
  initial.frontier = system.GetPoseFrontier();
  const auto view = system.GetKeyframeReadView();
  if (initial.frontier.frame_count || view.frame_count())
    throw std::invalid_argument(
        "Gaussian mapper requires a new keyframe session");
  ValidateOnlineGaussianPaths(options_, view.directory());
  map_index_.ObserveDelta(initial);
  if (!fs::create_directory(options_.output_directory))
    throw std::runtime_error("Could not create Gaussian output directory");
  state_sequence_ = 1;
  thread_ = std::thread(&OnlineGaussianMapper::Run, this);
}

OnlineGaussianMapper::~OnlineGaussianMapper() { Finish(); }

void OnlineGaussianMapper::Invalidate() {
  ready_.reset();
  saved_preview_name_.clear();
  ++state_sequence_;
}

void OnlineGaussianMapper::Fail(const std::string& message) noexcept {
  if (failed_) return;
  failed_ = true;
  if (render_stats_.in_flight) ++render_stats_.failed;
  StopRendering();
  render_stats_.in_flight = 0;
  render_stats_.pending_storage = 0;
  ++stats_.failed;
  try {
    stats_.error = message.substr(0, 1024);
  } catch (...) {
    // The frontend must continue even if allocating the diagnostic fails.
  }
  submap_work_.reset();
  candidate_command_.reset();
  pending_.reset();
  in_flight_.reset();
  candidate_.reset();
  unsaved_.reset();
  unsaved_render_.reset();
  decision_.reset();
  awaiting_decision_ = false;
  stats_.pending = 0;
  stats_.in_flight = 0;
  ready_.reset();
  stopping_ = true;
  // Never wait for the worker on the frontend thread. The mutex also protects
  // the descriptor from being closed and reused while shutdown is called.
  {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (lock && socket_fd_ >= 0) shutdown(socket_fd_, SHUT_RDWR);
  }
  condition_.notify_all();
}

bool OnlineGaussianMapper::IsCurrent(std::uint64_t generation,
                                     std::uint64_t submap_id,
                                     std::uint64_t pose_version) const {
  return submap_ && generation == generation_ &&
         submap_id == submap_->primary.id &&
         pose_version == map_index_.frontier().version;
}

void OnlineGaussianMapper::Observe(const OnlineSubmapUpdate& update) noexcept {
  ObserveUpdate(&update, nullptr);
}

void OnlineGaussianMapper::ObserveDelta(
    const OnlineSubmapDelta& update) noexcept {
  ObserveUpdate(nullptr, &update);
}

void OnlineGaussianMapper::ObserveUpdate(
    const OnlineSubmapUpdate* full, const OnlineSubmapDelta* delta) noexcept {
  if (!options_.enabled || finished_ || failed_) return;
  if (finalization_result_) {
    Fail("Observation after Gaussian finalization admission");
    return;
  }
  const auto start = Clock::now();
  try {
    if (full && !full->poses)
      throw std::invalid_argument("Missing Gaussian pose snapshot");
    const auto& keyframe = full ? full->keyframe : delta->keyframe;
    const auto& poses = full ? full->poses : delta->poses;
    const auto next = full ? poses->frontier() : delta->frontier;
    if (next.frame_count > kMaxFrames && !options_.persist_submaps) {
      ++stats_.resource_limited;
      Invalidate();
      Fail("Gaussian pose frontier exceeds 512 frames");
      return;
    }
    const bool changed_pose = next.version != map_index_.frontier().version;
    if (full)
      map_index_.Observe(*full);
    else
      map_index_.ObserveDelta(*delta);
    const auto* entry = map_index_.LastEntry();
    const bool changed_group =
        entry && (!submap_ || entry->group.primary.id != submap_->primary.id);
    const bool reset =
        !options_.persist_submaps && (changed_group || changed_pose);
    if (reset) {
      if (generation_ == std::numeric_limits<std::uint64_t>::max())
        throw std::overflow_error("Gaussian generation overflow");
      ++generation_;
      Invalidate();
      if (changed_group) latest_keyframe_.reset();
    }
    if (entry) submap_ = entry->group;
    if (keyframe) {
      ++stats_.observed;
      if (!options_.persist_submaps) latest_keyframe_ = keyframe;
    }
    const auto& source = keyframe ? keyframe : latest_keyframe_;
    if ((keyframe || reset) && source && submap_) {
      if (pending_) ++stats_.coalesced;
      pending_ = Source{source, poses, *submap_, generation_, 0, Clock::now(),
                        next};
      stats_.pending = stats_.peak_pending = 1;
    }
    if (options_.persist_submaps && ready_ &&
        (!entry || ready_->submap_id != entry->group.primary.id ||
         ready_->pose_version != next.version ||
         entry->availability != GaussianAvailability::kCurrent))
      Invalidate();
    stats_.has_submap = submap_.has_value();
    stats_.submap_id = submap_ ? submap_->primary.id : 0;
    stats_.generation = generation_;
    stats_.pose_version = next.version;
    ++state_sequence_;
    RefreshRenderDemand();
  } catch (const std::exception& error) {
    Fail(error.what());
  } catch (...) {
    Fail("Unknown Gaussian observation failure");
  }
  stats_.max_observe_seconds =
      std::max(stats_.max_observe_seconds, Seconds(start));
}

void OnlineGaussianMapper::HandleReply(Reply reply) {
  if (options_.persist_submaps) {
    HandleSubmapReply(std::move(reply));
    return;
  }
  busy_ = false;
  switch (reply.kind) {
    case ReplyKind::kConnected:
      connected_ = true;
      break;
    case ReplyKind::kReset:
      remote_known_ = true;
      remote_generation_ = reply.command.generation;
      remote_frame_ids_.clear();
      ++stats_.resets;
      break;
    case ReplyKind::kCandidate:
      ++stats_.completed;
      candidate_ = std::move(reply.preview);
      {
        auto expected = remote_frame_ids_;
        expected.push_back(in_flight_->keyframe->id);
        if (expected.size() > 10) expected.erase(expected.begin());
        if (candidate_->frame_ids != expected)
          throw std::runtime_error(
              "Gaussian candidate coverage differs from accepted views");
      }
      awaiting_decision_ = true;
      decision_ = IsCurrent(candidate_->generation, candidate_->submap_id,
                            candidate_->pose_version);
      if (!*decision_) ++stats_.stale;
      break;
    case ReplyKind::kConfirmed:
      awaiting_decision_ = false;
      if (reply.command.accept)
        remote_frame_ids_ = candidate_->frame_ids;
      else
        remote_frame_ids_.clear();
      if (reply.command.accept &&
          IsCurrent(candidate_->generation, candidate_->submap_id,
                    candidate_->pose_version)) {
        candidate_->latency_seconds = Seconds(in_flight_->observed_at);
        candidate_->confirmed_at_unix_seconds =
            std::chrono::duration<double>(
                std::chrono::system_clock::now().time_since_epoch())
                .count();
        ready_ = candidate_;
        unsaved_ = ready_;
        ++stats_.accepted;
        stats_.last_latency_seconds = candidate_->latency_seconds;
        stats_.total_latency_seconds += candidate_->latency_seconds;
        stats_.max_latency_seconds =
            std::max(stats_.max_latency_seconds, candidate_->latency_seconds);
        const auto bucket =
            std::lower_bound(kLatencyBounds.begin(), kLatencyBounds.end(),
                             candidate_->latency_seconds);
        ++stats_.latency_buckets[std::distance(kLatencyBounds.begin(), bucket)];
        ++state_sequence_;
      } else if (reply.command.accept) {
        ++stats_.stale;
      }
      candidate_.reset();
      in_flight_.reset();
      stats_.in_flight = 0;
      break;
    case ReplyKind::kSkipped:
      if (reply.detail == "no_image")
        ++stats_.skipped_no_image;
      else if (reply.detail == "no_depth")
        ++stats_.skipped_no_depth;
      else
        ++stats_.resource_limited;
      in_flight_.reset();
      stats_.in_flight = 0;
      break;
    case ReplyKind::kSaved:
      if (ready_ && ready_->job_id == reply.command.preview->job_id)
        saved_preview_name_ = reply.detail;
      if (unsaved_ && unsaved_->job_id == reply.command.preview->job_id)
        unsaved_.reset();
      ++state_sequence_;
      break;
    case ReplyKind::kStatus:
      saved_status_sequence_ = reply.command.state_sequence;
      break;
    case ReplyKind::kActivated:
    case ReplyKind::kCheckpoint:
    case ReplyKind::kMapWritten:
    case ReplyKind::kMapCommitted:
    case ReplyKind::kPruned:
    case ReplyKind::kRendered:
    case ReplyKind::kRenderError:
    case ReplyKind::kRenderSaved:
      throw std::runtime_error("Unexpected persistent Gaussian reply");
    case ReplyKind::kError:
      Fail(reply.detail);
      break;
  }
}

void OnlineGaussianMapper::Dispatch() {
  if (!connected_ || busy_ || failed_) return;
  if (options_.persist_submaps) {
    DispatchSubmaps();
    return;
  }
  Command command;
  if (awaiting_decision_) {
    if (!decision_) return;
    if (*decision_ && !IsCurrent(candidate_->generation, candidate_->submap_id,
                                 candidate_->pose_version)) {
      *decision_ = false;
      ++stats_.stale;
    }
    command.kind = CommandKind::kDecision;
    command.preview = candidate_;
    command.accept = *decision_;
  } else if (!remote_known_ || remote_generation_ != generation_) {
    command.kind = CommandKind::kReset;
    command.generation = generation_;
    command.pose_version = map_index_.frontier().version;
    if (submap_) command.submap_id = submap_->primary.id;
  } else if (unsaved_) {
    command.kind = CommandKind::kSave;
    command.preview = unsaved_;
  } else if (state_sequence_ > saved_status_sequence_) {
    command.kind = CommandKind::kStatus;
    command.generation = generation_;
    command.pose_version = map_index_.frontier().version;
    command.state_sequence = state_sequence_;
    if (submap_) command.submap_id = submap_->primary.id;
    command.ready = ready_ && !saved_preview_name_.empty();
    command.preview_name = command.ready ? saved_preview_name_ : "";
  } else if (pending_) {
    command.kind = CommandKind::kView;
    command.source = *pending_;
    if (!command.source.poses) {
      command.source.poses = map_index_.GetPoseSnapshot();
      if (!(command.source.poses->frontier() == command.source.frontier))
        throw std::invalid_argument(
            "Gaussian view frontier changed before dispatch");
    }
    command.source.job_id = next_job_;
  } else {
    return;
  }
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock || command_ || reply_) return;
  command_ = command;
  busy_ = true;
  if (command.kind == CommandKind::kDecision) decision_.reset();
  if (command.kind == CommandKind::kView) {
    in_flight_ = command.source;
    pending_.reset();
    ++next_job_;
    ++stats_.submitted;
    stats_.pending = 0;
    stats_.in_flight = stats_.peak_in_flight = 1;
  }
  lock.unlock();
  condition_.notify_one();
}

void OnlineGaussianMapper::Poll() noexcept {
  if (!options_.enabled || finished_ || failed_) return;
  const auto start = Clock::now();
  try {
    std::optional<Reply> reply;
    {
      std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
      if (lock && reply_) {
        reply = std::move(reply_);
        reply_.reset();
      }
    }
    if (reply) {
      condition_.notify_one();
      HandleReply(std::move(*reply));
    }
    Dispatch();
  } catch (const std::exception& error) {
    Fail(error.what());
  } catch (...) {
    Fail("Unknown Gaussian polling failure");
  }
  stats_.max_poll_seconds = std::max(stats_.max_poll_seconds, Seconds(start));
}

std::shared_ptr<const GaussianPreview> OnlineGaussianMapper::GetLatestPreview()
    const {
  return ready_;
}

OnlineGaussianStats OnlineGaussianMapper::GetStats() const {
  auto result = stats_;
  if (options_.persist_submaps) {
    const auto map = map_index_.Snapshot();
    result.map_sequence = acknowledged_map_;
    result.indexed = map.entries.size();
    for (const auto& entry : map.entries) {
      result.dirty += entry.dirty;
      result.missing += entry.availability == GaussianAvailability::kMissing;
      result.needs_rebuild +=
          entry.availability == GaussianAvailability::kNeedsRebuild;
      if (entry.asset)
        result.asset_bytes += entry.asset->manifest_bytes +
                              entry.asset->gaussian_bytes +
                              entry.asset->observation_bytes;
    }
  }
  return result;
}

std::shared_ptr<const GaussianMapSnapshot>
OnlineGaussianMapper::GetMapSnapshot() const {
  if (!options_.enabled || !options_.persist_submaps) return {};
  return std::make_shared<GaussianMapSnapshot>(map_index_.Snapshot());
}

void OnlineGaussianMapper::Finish() noexcept {
  if (finished_) return;
  StopRendering();
  if (options_.persist_submaps) {
    finishing_ = true;
    pending_.reset();
    stats_.pending = 0;
  }
  if (options_.enabled && !failed_) {
    const auto deadline = Clock::now() + options_.shutdown_timeout;
    while (Clock::now() < deadline) {
      Poll();
      if (options_.persist_submaps && SubmapsDrained()) break;
      if (!options_.persist_submaps && connected_ && !busy_ && !pending_ &&
          !in_flight_ && !unsaved_ && state_sequence_ == saved_status_sequence_)
        break;
      std::this_thread::sleep_for(1ms);
    }
  }
  if (options_.persist_submaps)
    stats_.incomplete_shutdown = failed_ || !SubmapsDrained();
  const bool graceful = connected_ && !busy_ && !failed_;
  stopping_ = true;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!graceful && socket_fd_ >= 0) shutdown(socket_fd_, SHUT_RDWR);
  }
  condition_.notify_all();
  if (thread_.joinable()) thread_.join();
  // StopRendering only withdraws demand. A running render/publication remains
  // in flight until definitive failure or this join releases its command.
  unsaved_render_.reset();
  render_stats_.in_flight = 0;
  render_stats_.pending_storage = 0;
  if (options_.enabled && options_.persist_submaps && !stop_acknowledged_)
    stats_.incomplete_shutdown = true;
  // After the join no background task or mailbox still owns a live job.
  command_.reset();
  reply_.reset();
  pending_.reset();
  in_flight_.reset();
  candidate_.reset();
  unsaved_.reset();
  decision_.reset();
  stats_.pending = 0;
  stats_.in_flight = 0;
  const int fd = socket_fd_.exchange(-1);
  if (fd >= 0) close(fd);
  finished_ = stats_.finished = true;
}

void OnlineGaussianMapper::CloseSocket() noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  const int fd = socket_fd_.exchange(-1);
  if (fd >= 0) close(fd);
}

void OnlineGaussianMapper::Run() noexcept {
  const auto post = [this](Reply reply) {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this] { return !reply_ || stopping_; });
    if (!stopping_) reply_ = std::move(reply);
  };
  try {
    const auto deadline = Clock::now() + options_.connect_timeout;
    const auto name = fs::absolute(options_.socket_path).string();
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    std::copy(name.begin(), name.end(), address.sun_path);
    int fd = -1;
    while (!stopping_ && Clock::now() < deadline) {
      fd = socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
      if (fd < 0)
        throw std::system_error(errno, std::generic_category(), "socket");
      {
        std::lock_guard<std::mutex> lock(mutex_);
        socket_fd_ = fd;
      }
      int result =
          connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address));
      if (result < 0 && errno == EINPROGRESS) {
        while (!stopping_ && Clock::now() < deadline) {
          pollfd event{fd, POLLOUT, 0};
          result = poll(&event, 1, 25);
          if (result < 0 && errno == EINTR) continue;
          if (result < 0)
            throw std::system_error(errno, std::generic_category(), "poll");
          if (result > 0) {
            int socket_error = 0;
            socklen_t size = sizeof(socket_error);
            if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &socket_error, &size))
              throw std::system_error(errno, std::generic_category(),
                                      "getsockopt");
            result = socket_error ? -1 : 0;
            break;
          }
        }
      }
      if (result == 0 && !stopping_) break;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        socket_fd_ = -1;
        close(fd);
      }
      fd = -1;
      if (!stopping_) std::this_thread::sleep_for(25ms);
    }
    if (stopping_) {
      CloseSocket();
      return;
    }
    if (fd < 0)
      throw std::runtime_error("Gaussian worker connection timed out");
    const bool spatial =
        options_.joint_render_policy == GaussianRenderPolicy::kSpatial;
    WriteGaussianMessage(
        fd,
        {Json([&](auto& out) {
           Base(out, "hello", session_id_, options_.persist_submaps ? 2 : 1);
           if (options_.joint_render_enabled) {
             out << "capabilities" << "[" << "joint_render_v1";
             if (spatial) out << "joint_render_spatial_v1";
             out << "]";
           }
           if (options_.persist_submaps)
             out << "state_directory"
                 << (fs::canonical(options_.output_directory) / "states")
                        .string();
         }),
         {}},
        options_.transfer_timeout);
    const auto hello = ReadGaussianMessage(fd, options_.connect_timeout);
    const auto greeting =
        Parse(hello, "hello_ok", session_id_, options_.persist_submaps ? 2 : 1);
    if (static_cast<int>(greeting["schema"]) !=
        (options_.persist_submaps ? 2 : 1))
      throw std::runtime_error("Gaussian worker protocol mode mismatch");
    if (options_.joint_render_enabled &&
        (!greeting["capabilities"].isSeq() ||
         greeting["capabilities"].size() < 1 ||
         static_cast<std::string>(greeting["capabilities"][0]) !=
             "joint_render_v1"))
      throw std::runtime_error("Worker lacks joint_render_v1 capability");
    if (options_.joint_render_enabled &&
        greeting["capabilities"].size() != (spatial ? 2u : 1u))
      throw std::runtime_error(spatial
          ? "Worker lacks joint_render_spatial_v1 capability"
          : "Unexpected Gaussian render capability");
    post({ReplyKind::kConnected});

    while (!stopping_) {
      Command command;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock,
                        [this] { return stopping_ || command_.has_value(); });
        if (stopping_) break;
        command = std::move(*command_);
        command_.reset();
      }
      if (stopping_) break;
      Reply reply;
      reply.command = command;
      if (command.kind == CommandKind::kSaveRender) {
        try {
          if (written_renders_.size() == 4) {
            const auto oldest = written_renders_.front();
            if (!fs::remove(options_.output_directory / (oldest + ".json")) ||
                !fs::remove(options_.output_directory / (oldest + ".png")))
              throw std::runtime_error("Could not remove oldest joint render");
            written_renders_.pop_front();
          }
          WriteGaussianRender(options_.output_directory, *command.render);
          written_renders_.push_back("joint-render-" +
              std::to_string(command.render->request.job_id));
          reply.kind = ReplyKind::kRenderSaved;
        } catch (const std::exception& error) {
          reply.kind = ReplyKind::kRenderError;
          reply.detail = std::string("Joint preview write failed: ") +
                         error.what();
        }
      } else if (options_.persist_submaps &&
                 command.kind != CommandKind::kSave &&
          command.kind != CommandKind::kStatus) {
        reply = RunSubmapCommand(fd, command);
      } else if (command.kind == CommandKind::kReset) {
        GaussianMessage message;
        message.header_json = Json([&](auto& out) {
          Base(out, "reset", session_id_);
          Id(out, "generation", command.generation);
          Id(out, "pose_version", command.pose_version);
          if (command.submap_id) Id(out, "submap_id", *command.submap_id);
        });
        WriteGaussianMessage(fd, message, options_.transfer_timeout);
        const auto response =
            ReadGaussianMessage(fd, options_.transfer_timeout);
        const auto json = Parse(response, "reset_ok", session_id_);
        if (ReadId(json["generation"]) != command.generation ||
            ReadId(json["pose_version"]) != command.pose_version ||
            (command.submap_id
                 ? json["submap_id"].empty() ||
                       ReadId(json["submap_id"]) != *command.submap_id
                 : !json["submap_id"].empty()))
          throw std::runtime_error("Gaussian reset confirmation mismatch");
        reply.kind = ReplyKind::kReset;
      } else if (command.kind == CommandKind::kView) {
        const auto worker_started = Clock::now();
        const auto& source = command.source;
        const auto& frame = *source.keyframe;
        reply.kind = ReplyKind::kSkipped;
        if (frame.image.empty() || !frame.image_timestamp ||
            !frame.camera_from_odom) {
          reply.detail = "no_image";
        } else if (!frame.cloud || frame.cloud->size() > kMaxPoints ||
                   frame.image.total() > kMaxPixels) {
          reply.detail = "limit";
        } else {
          const auto view = MakeReconstructionView(
              camera_, frame, *source.poses, {kMaxPixels, kMaxPoints});
          if (!view.valid_depth_pixels) {
            reply.detail = "no_depth";
          } else {
            const auto anchor = source.submap.primary.anchor_keyframe_id;
            const auto parent_from_submap =
                source.poses->poses.at(anchor).parent_from_body();
            const auto camera_from_submap =
                pose_math::Compose(view.camera_from_parent, parent_from_submap);
            GaussianMessage message;
            message.header_json = Json([&](auto& out) {
              Base(out, "view", session_id_);
              Id(out, "job_id", source.job_id);
              Id(out, "generation", source.generation);
              Id(out, "submap_id", source.submap.primary.id);
              Id(out, "pose_version", view.pose_version);
              Id(out, "membership_revision", source.submap.membership_revision);
              Id(out, "frame_count", view.frame_count);
              Id(out, "optimized_frame_count", view.optimized_frame_count);
              Id(out, "keyframe_id", view.keyframe_id);
              Id(out, "calibration_id", view.calibration_id);
              out << "parent_frame"
                  << (view.parent_frame == PoseFrame::kOdom ? "odom" : "map")
                  << "state_timestamp" << view.state_timestamp
                  << "cloud_timestamp" << view.cloud_timestamp
                  << "image_timestamp" << view.image_timestamp
                  << "inverse_exposure_time" << view.inverse_exposure_time
                  << "width" << view.image_bgr.cols << "height"
                  << view.image_bgr.rows << "fx" << view.camera.fx << "fy"
                  << view.camera.fy << "cx" << view.camera.cx << "cy"
                  << view.camera.cy;
              Transform(out, "camera_from_submap", camera_from_submap);
              Transform(out, "parent_from_submap", parent_from_submap);
            });
            message.body = ViewBody(view);
            WriteGaussianMessage(fd, message, options_.transfer_timeout);
            const auto response = ReadGaussianMessage(fd, options_.job_timeout);
            const auto json = Parse(response, "candidate", session_id_);
            auto preview = std::make_shared<GaussianPreview>();
            preview->session_id = session_id_;
            preview->job_id = source.job_id;
            preview->generation = source.generation;
            preview->submap_id = source.submap.primary.id;
            preview->pose_version = view.pose_version;
            preview->membership_revision = source.submap.membership_revision;
            preview->frame_count = view.frame_count;
            preview->optimized_frame_count = view.optimized_frame_count;
            preview->parent_from_submap = parent_from_submap;
            preview->capture_time = view.image_timestamp;
            preview->worker_seconds = Seconds(worker_started);
            CheckIdentity(json, *preview);
            if (ReadId(json["membership_revision"]) !=
                    preview->membership_revision ||
                ReadId(json["frame_count"]) != preview->frame_count ||
                ReadId(json["optimized_frame_count"]) !=
                    preview->optimized_frame_count ||
                static_cast<int>(json["width"]) != view.image_bgr.cols ||
                static_cast<int>(json["height"]) != view.image_bgr.rows)
              throw std::runtime_error(
                  "Gaussian candidate frontier or dimensions mismatch");
            for (const auto& node : json["frame_ids"]) {
              const auto id = ReadId(node);
              if (std::find(source.submap.primary.keyframe_ids.begin(),
                            source.submap.primary.keyframe_ids.end(),
                            id) == source.submap.primary.keyframe_ids.end())
                throw std::runtime_error(
                    "Gaussian candidate references a foreign frame");
              preview->frame_ids.push_back(id);
            }
            if (preview->frame_ids.empty() ||
                preview->frame_ids.back() != frame.id)
              throw std::runtime_error(
                  "Gaussian candidate omits its source frame");
            preview->width = view.image_bgr.cols;
            preview->height = view.image_bgr.rows;
            preview->steps = static_cast<int>(json["steps"]);
            preview->gaussian_count = static_cast<int>(json["gaussian_count"]);
            preview->supervised_depth_pixels =
                static_cast<int>(json["supervised_depth_pixels"]);
            preview->rgb_l1 = static_cast<double>(json["rgb_l1"]);
            preview->depth_l1_m = static_cast<double>(json["depth_l1_m"]);
            if (!json["nonzero_gradient"].empty())
              preview->nonzero_gradient =
                  static_cast<int>(json["nonzero_gradient"]) != 0;
            if (!json["parameter_updated"].empty())
              preview->parameter_updated =
                  static_cast<int>(json["parameter_updated"]) != 0;
            if (!json["gpu_peak_allocated_bytes"].empty())
              preview->gpu_peak_allocated_bytes =
                  ReadId(json["gpu_peak_allocated_bytes"]);
            if (!json["gpu_peak_reserved_bytes"].empty())
              preview->gpu_peak_reserved_bytes =
                  ReadId(json["gpu_peak_reserved_bytes"]);
            if (!json["worker_peak_rss_bytes"].empty())
              preview->worker_peak_rss_bytes =
                  ReadId(json["worker_peak_rss_bytes"]);
            preview->rgb = response.body;
            reply.kind = ReplyKind::kCandidate;
            reply.preview = std::move(preview);
          }
        }
      } else if (command.kind == CommandKind::kDecision) {
        GaussianMessage message;
        message.header_json = Json([&](auto& out) {
          Base(out, "decision", session_id_);
          PreviewIdentity(out, *command.preview);
          out << "action" << (command.accept ? "accept" : "discard");
        });
        WriteGaussianMessage(fd, message, options_.transfer_timeout);
        const auto response =
            ReadGaussianMessage(fd, options_.transfer_timeout);
        const auto json = Parse(
            response, command.accept ? "commit_ok" : "discard_ok", session_id_);
        CheckIdentity(json, *command.preview);
        reply.kind = ReplyKind::kConfirmed;
      } else if (command.kind == CommandKind::kSave) {
        if (written_previews_.size() == kMaxPreviews) {
          const auto oldest = written_previews_.front();
          if (!fs::remove(options_.output_directory / (oldest + ".json")) ||
              !fs::remove(options_.output_directory / (oldest + ".png")))
            throw std::runtime_error(
                "Could not remove oldest Gaussian preview");
          written_previews_.pop_front();
        }
        const auto& preview = *command.preview;
        const std::string stem = "preview-" + std::to_string(preview.job_id);
        const auto temporary_png =
            options_.output_directory / (stem + ".tmp.png");
        cv::Mat rgb(preview.height, preview.width, CV_8UC3,
                    const_cast<std::uint8_t*>(preview.rgb.data()));
        cv::Mat bgr;
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
        if (!cv::imwrite(temporary_png.string(), bgr))
          throw std::runtime_error("Could not write Gaussian preview image");
        fs::rename(temporary_png, options_.output_directory / (stem + ".png"));
        const auto metadata = Json([&](auto& out) {
          Base(out, "preview", session_id_);
          PreviewIdentity(out, preview);
          if (options_.persist_submaps)
            out << "operation"
                << (preview.operation == GaussianOperation::kView
                        ? "view"
                        : preview.operation == GaussianOperation::kReuse
                              ? "reuse"
                              : "rebuild");
          Id(out, "membership_revision", preview.membership_revision);
          Id(out, "frame_count", preview.frame_count);
          Id(out, "optimized_frame_count", preview.optimized_frame_count);
          out << "frame_ids"
              << "[";
          for (const auto id : preview.frame_ids) out << std::to_string(id);
          out << "]"
              << "capture_time" << preview.capture_time << "latency_seconds"
              << preview.latency_seconds << "worker_seconds"
              << preview.worker_seconds << "confirmed_at_unix_seconds"
              << preview.confirmed_at_unix_seconds << "width" << preview.width
              << "height" << preview.height << "steps" << preview.steps
              << "rgb_l1" << preview.rgb_l1 << "depth_l1_m"
              << preview.depth_l1_m << "gaussian_count"
              << preview.gaussian_count << "supervised_depth_pixels"
              << preview.supervised_depth_pixels << "nonzero_gradient"
              << static_cast<int>(preview.nonzero_gradient)
              << "parameter_updated"
              << static_cast<int>(preview.parameter_updated);
          Id(out, "gpu_peak_allocated_bytes", preview.gpu_peak_allocated_bytes);
          Id(out, "gpu_peak_reserved_bytes", preview.gpu_peak_reserved_bytes);
          Id(out, "worker_peak_rss_bytes", preview.worker_peak_rss_bytes);
          Transform(out, "parent_from_submap", preview.parent_from_submap);
        });
        const auto temporary_json =
            options_.output_directory / (stem + ".tmp.json");
        WriteFile(temporary_json, metadata);
        fs::rename(temporary_json,
                   options_.output_directory / (stem + ".json"));
        written_previews_.push_back(stem);
        reply.kind = ReplyKind::kSaved;
        reply.detail = stem + ".png";
      } else {
        const auto now =
            std::chrono::duration<double>(
                std::chrono::system_clock::now().time_since_epoch())
                .count();
        const auto status = Json([&](auto& out) {
          out << "session_id" << session_id_;
          Id(out, "state_sequence", command.state_sequence);
          Id(out, "generation", command.generation);
          Id(out, "pose_version", command.pose_version);
          if (command.submap_id) Id(out, "submap_id", *command.submap_id);
          out << "written_at_unix_seconds" << now << "ready"
              << static_cast<int>(command.ready) << "preview"
              << command.preview_name;
        });
        const auto temporary = options_.output_directory / "status.tmp.json";
        WriteFile(temporary, status);
        fs::rename(temporary, options_.output_directory / "status.json");
        reply.kind = ReplyKind::kStatus;
      }
      post(std::move(reply));
    }
    if (fd >= 0) {
      try {
        WriteGaussianMessage(
            fd,
            {Json([&](auto& out) {
               Base(out, "stop", session_id_, options_.persist_submaps ? 2 : 1);
               if (options_.persist_submaps) {
                 Id(out, "job_id", next_job_);
                 Id(out, "generation", generation_);
                 Id(out, "pose_version", stats_.pose_version);
                 Id(out, "map_sequence", acknowledged_map_);
                 out << "incomplete"
                     << static_cast<int>(stats_.incomplete_shutdown);
               }
             }),
             {}},
            options_.shutdown_timeout);
        if (options_.persist_submaps) {
          const auto stopped =
              ReadGaussianMessage(fd, options_.shutdown_timeout);
          const auto confirmation = Parse(stopped, "stopped", session_id_, 2);
          if (static_cast<int>(confirmation["schema"]) != 2 ||
              ReadId(confirmation["job_id"]) != next_job_ ||
              ReadId(confirmation["generation"]) != generation_ ||
              ReadId(confirmation["pose_version"]) != stats_.pose_version ||
              ReadId(confirmation["map_sequence"]) != acknowledged_map_ ||
              static_cast<int>(confirmation["incomplete"]) !=
                  static_cast<int>(stats_.incomplete_shutdown))
            throw std::runtime_error("Gaussian stop confirmation mismatch");
          stop_acknowledged_ = true;
        }
      } catch (...) {
      }
    }
  } catch (const std::exception& error) {
    CloseSocket();
    post({ReplyKind::kError, {}, {}, error.what()});
  } catch (...) {
    CloseSocket();
    post({ReplyKind::kError, {}, {}, "Unknown Gaussian worker failure"});
  }
  CloseSocket();
}

}  // namespace livo_atlas
