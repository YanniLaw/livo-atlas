#include "livo_atlas/core/geometry_map_output.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <utility>

#include "livo_atlas/common/logging.h"
#include "livo_atlas/core/livo_system.h"
#include "mapping/geometry_map_sampling.h"
#include "mapping/geometry_validation.h"

namespace livo_atlas {
namespace {

std::string ProducerId() {
  std::random_device random;
  std::string id;
  id.reserve(32);
  constexpr char kHex[] = "0123456789abcdef";
  for (int i = 0; i < 4; ++i) {
    const std::uint32_t value = random();
    for (int shift = 28; shift >= 0; shift -= 4)
      id.push_back(kHex[(value >> shift) & 15]);
  }
  return id;
}

std::string ErrorText(std::exception_ptr error) {
  try {
    std::rethrow_exception(error);
  } catch (const std::exception& exception) {
    return std::string(exception.what()).substr(0, 1024);
  } catch (...) {
    return "Unknown geometry map output failure";
  }
}

void CheckHeader(const PoseSnapshot& poses) {
  const bool projected =
      poses.tail_policy == PoseTailPolicy::kProjectedLastOptimizedAnchor;
  if ((poses.tail_policy != PoseTailPolicy::kLegacyLastOptimizedAnchor &&
       !projected) ||
      poses.frame_count != poses.poses.size() ||
      (poses.version == 0
           ? poses.base_version != 0 || poses.optimized_frame_count != 0 ||
                 poses.parent_frame != PoseFrame::kOdom || projected
           : poses.base_version != poses.version - 1 ||
                 !poses.optimized_frame_count ||
                 poses.optimized_frame_count > poses.frame_count ||
                 poses.parent_frame != PoseFrame::kMap || !projected))
    throw std::invalid_argument("Invalid live geometry map source header");
}

bool SamePose(const KeyframePose& a, const KeyframePose& b) {
  return a.id == b.id &&
         (a.map_from_body.rotation.array() == b.map_from_body.rotation.array())
             .all() &&
         (a.map_from_body.translation.array() ==
          b.map_from_body.translation.array())
             .all();
}

bool SameGroup(const OnlineSubmap& a, const OnlineSubmap& b) {
  return a.primary.id == b.primary.id &&
         a.primary.anchor_keyframe_id == b.primary.anchor_keyframe_id &&
         a.primary.keyframe_ids == b.primary.keyframe_ids &&
         a.support_keyframe_ids == b.support_keyframe_ids &&
         a.membership_revision == b.membership_revision && a.state == b.state;
}

double Seconds(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
      .count();
}

}  // namespace

void ValidateGeometryMapOutputOptions(const GeometryMapOutputOptions& options) {
  if (!options.enabled) return;
  if (!std::isfinite(options.max_frequency_hz) ||
      options.max_frequency_hz <= 0 || options.max_frequency_hz > 2 ||
      options.max_keyframes < 1 || options.max_keyframes > 100000)
    throw std::invalid_argument(
        "Geometry map requires frequency in (0,2] Hz and "
        "max_keyframes in [1,100000]");
  ValidateGeometryMapOptions(options.map);
}

GeometryMapOutput::GeometryMapOutput(LivoSystem& system,
                                     GeometryMapOutputOptions options,
                                     GeometryOptions geometry,
                                     Consumer consumer)
    : system_(system),
      options_(options),
      geometry_options_(geometry),
      consumer_(std::move(consumer)) {
  ValidateGeometryMapOutputOptions(options_);
  if (!options_.enabled) return;
  if (!consumer_) throw std::invalid_argument("Missing geometry map consumer");
  ValidateGeometryOptions(geometry_options_);
  const auto initial_poses = system_.GetPoseSnapshot();
  const auto initial_view = system_.GetKeyframeReadView();
  if (!initial_poses || initial_poses->frame_count != 0 ||
      initial_view.frame_count() != 0)
    throw std::invalid_argument("Geometry map output requires a new session");
  producer_id_ = ProducerId();
  builder_ = std::make_unique<GeometryMapBuilder>(
      geometry_options_, options_.map,
      [this](std::uint64_t id) {
        if (stopping_) throw std::runtime_error("Geometry map output stopped");
        if (!view_ || id >= view_->frame_count())
          throw std::out_of_range("Geometry cloud exceeds fixed read view");
        return view_->ReadBodyCloud(id,
                                    geometry_options_.max_points_per_keyframe);
      },
      [this] { return stopping_.load(); });
  OnlineSubmapUpdate initial;
  initial.poses = initial_poses;
  Observe(initial);
  if (observe_error_) std::rethrow_exception(observe_error_);
  thread_ = std::thread(&GeometryMapOutput::Run, this);
}

GeometryMapOutput::~GeometryMapOutput() { Stop(); }

bool GeometryMapOutput::ApplyRevisions(const OnlineSubmapUpdate& update) {
  if (update.revisions.size() > 2)
    throw std::invalid_argument("Too many geometry membership revisions");
  bool changed = false;
  for (const auto& group : update.revisions) {
    geometry_detail::ValidateRevision(group, *update.poses);
    const auto id = group.primary.id;
    const auto& members = group.primary.keyframe_ids;
    if (id > groups_.size() ||
        (id < groups_.size() && id + 1 != groups_.size()))
      throw std::invalid_argument("Invalid geometry membership ID");
    if (id == groups_.size()) {
      if (group.state != OnlineSubmapState::kActive || members.size() != 1 ||
          group.membership_revision != 1)
        throw std::invalid_argument("Invalid new geometry membership");
      if (groups_.empty()) {
        if (members.front() != 0 || !group.support_keyframe_ids.empty())
          throw std::invalid_argument("Invalid first geometry membership");
      } else {
        const auto& previous = groups_.back();
        const auto& prior = previous.primary.keyframe_ids;
        const auto& support = group.support_keyframe_ids;
        if (previous.state != OnlineSubmapState::kSealed ||
            members.front() != prior.back() + 1 ||
            support.size() > prior.size() ||
            !std::equal(support.begin(), support.end(),
                        prior.end() - support.size()))
          throw std::invalid_argument("Invalid geometry membership boundary");
      }
      groups_.push_back(group);
      changed = true;
      continue;
    }
    auto& old = groups_.back();
    if (SameGroup(old, group)) continue;
    const auto& before = old.primary.keyframe_ids;
    const bool appended = members.size() == before.size() + 1 &&
                          group.state == OnlineSubmapState::kActive;
    const bool sealed = members.size() == before.size() &&
                        group.state == OnlineSubmapState::kSealed;
    if (old.state != OnlineSubmapState::kActive ||
        old.membership_revision == std::numeric_limits<std::uint64_t>::max() ||
        group.membership_revision != old.membership_revision + 1 ||
        group.primary.anchor_keyframe_id != old.primary.anchor_keyframe_id ||
        group.support_keyframe_ids != old.support_keyframe_ids ||
        (!appended && !sealed) ||
        !std::equal(before.begin(), before.end(), members.begin()))
      throw std::invalid_argument("Invalid geometry membership revision");
    old = group;
    changed = true;
  }
  if ((groups_.empty() && update.poses->frame_count != 0) ||
      (!groups_.empty() && groups_.back().primary.keyframe_ids.back() !=
                               update.poses->frame_count - 1))
    throw std::invalid_argument("Geometry membership does not cover poses");
  if (update.keyframe && (!update.poses->frame_count ||
                          update.keyframe->id != update.poses->frame_count - 1))
    throw std::invalid_argument("Geometry keyframe is not the committed tail");
  submap_count_ = groups_.size();
  if (!groups_.empty()) {
    last_membership_revision_ = groups_.back().membership_revision;
    last_membership_state_ = groups_.back().state;
  }
  return changed;
}

void GeometryMapOutput::Observe(const OnlineSubmapUpdate& update) noexcept {
  if (!options_.enabled || finished_ || terminal_pending_) return;
  const auto start = Clock::now();
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    if (failed_) return;
    if (!update.poses)
      throw std::invalid_argument("Missing geometry map poses");
    const auto& poses = *update.poses;
    CheckHeader(poses);
    if (have_source_ &&
        (poses.version < latest_header_.pose_version ||
         poses.frame_count < latest_header_.frame_count ||
         poses.optimized_frame_count < latest_header_.optimized_frame_count ||
         (poses.version == latest_header_.pose_version &&
          poses.optimized_frame_count != latest_header_.optimized_frame_count)))
      throw std::invalid_argument("Geometry map source moved backwards");
    const bool frame_limit =
        poses.frame_count > static_cast<std::uint64_t>(options_.max_keyframes);
    if (!limited_ && !frame_limit && latest_poses_ &&
        poses.version == latest_poses_->version &&
        !std::equal(latest_poses_->poses.begin(), latest_poses_->poses.end(),
                    poses.poses.begin(), SamePose))
      throw std::invalid_argument("Geometry map pose version changed in place");

    bool membership_changed = false;
    if (!limited_ && !frame_limit) {
      membership_changed = ApplyRevisions(update);
    } else {
      if (update.revisions.size() > 2)
        throw std::invalid_argument("Too many geometry membership revisions");
      for (const auto& group : update.revisions) {
        geometry_detail::ValidateRevision(group, poses);
        if (group.primary.id > submap_count_ ||
            (group.primary.id < submap_count_ &&
             group.primary.id + 1 != submap_count_))
          throw std::invalid_argument("Geometry membership ID gap");
        if (group.primary.id == submap_count_) {
          if (group.membership_revision != 1 ||
              group.state != OnlineSubmapState::kActive ||
              (submap_count_ &&
               last_membership_state_ != OnlineSubmapState::kSealed))
            throw std::invalid_argument("Invalid new geometry membership");
          ++submap_count_;
          last_membership_revision_ = 0;
          last_membership_state_ = OnlineSubmapState::kActive;
        }
        if (group.membership_revision == last_membership_revision_) {
          if (group.state != last_membership_state_)
            throw std::invalid_argument("Geometry membership changed in place");
          continue;
        }
        if (last_membership_state_ == OnlineSubmapState::kSealed ||
            last_membership_revision_ ==
                std::numeric_limits<std::uint64_t>::max() ||
            group.membership_revision != last_membership_revision_ + 1)
          throw std::invalid_argument("Invalid geometry membership revision");
        last_membership_revision_ = group.membership_revision;
        last_membership_state_ = group.state;
        membership_changed = true;
      }
    }
    const bool byte_limit =
        poses.frame_count && !geometry_map_detail::GeometryMapPointLimit(
                                 options_.map, submap_count_);
    const bool was_limited = limited_;
    limited_ = limited_ || frame_limit || byte_limit;
    if (have_source_ && !membership_changed && limited_ == was_limited &&
        poses.version == latest_header_.pose_version &&
        poses.frame_count == latest_header_.frame_count &&
        submap_count_ == latest_header_.submap_count)
      return;
    if (revision_ == std::numeric_limits<std::uint64_t>::max())
      throw std::overflow_error("Geometry map revision exhausted");
    GeometryMapSnapshot header;
    header.producer_id = producer_id_;
    header.revision = ++revision_;
    header.pose_version = poses.version;
    header.frame_count = poses.frame_count;
    header.optimized_frame_count = poses.optimized_frame_count;
    header.submap_count = submap_count_;
    header.parent_frame = poses.parent_frame;
    header.geometry_options = geometry_options_;
    header.options = options_.map;
    header.max_keyframes = options_.max_keyframes;
    if (limited_) {
      header.status = GeometryMapStatus::kResourceLimited;
      header.error = frame_limit
                         ? "Geometry map exceeds configured max_keyframes"
                         : "Geometry map exceeds configured message budget";
      std::vector<OnlineSubmap>().swap(groups_);
      latest_poses_.reset();
    } else {
      latest_poses_ = update.poses;
    }
    if (pending_) ++stats_.coalesced;
    pending_ =
        Observed{header, limited_ ? nullptr : update.poses, Clock::now()};
    latest_header_ = std::move(header);
    have_source_ = true;
    ++stats_.observed;
    stats_.peak_pending = 1;
    stats_.max_observe_seconds =
        std::max(stats_.max_observe_seconds, Seconds(start));
  } catch (...) {
    observe_error_ = std::current_exception();
    terminal_pending_ = true;
    latest_poses_.reset();
    std::vector<OnlineSubmap>().swap(groups_);
    if (pending_) pending_->poses.reset();
  }
}

void GeometryMapOutput::Poll() { Poll(false); }

void GeometryMapOutput::Poll(bool force) {
  if (!options_.enabled || finished_) return;
  const auto start = Clock::now();
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    if (failed_) {
      pending_.reset();
      latest_poses_.reset();
      std::vector<OnlineSubmap>().swap(groups_);
      return;
    }
    if (busy_) return;
    if (observe_error_) {
      if (revision_ == std::numeric_limits<std::uint64_t>::max())
        throw std::overflow_error("Geometry map revision exhausted");
      auto header = latest_header_;
      header.producer_id = producer_id_;
      header.revision = ++revision_;
      header.status = GeometryMapStatus::kError;
      header.error = ErrorText(observe_error_);
      pending_ = Observed{std::move(header), {}, Clock::now()};
      observe_error_ = {};
    }
    if (!pending_) return;
    const bool urgent = pending_->header.status != GeometryMapStatus::kReady &&
                        !nonready_delivered_;
    if (!force && !urgent && have_delivery_ &&
        std::chrono::duration<double>(Clock::now() - last_delivery_).count() <
            1 / options_.max_frequency_hz)
      return;
    std::optional<KeyframeReadView> view;
    std::vector<OnlineSubmap> groups;
    if (pending_->header.status == GeometryMapStatus::kReady) {
      try {
        view = system_.GetKeyframeReadView();
        if (view->frame_count() < pending_->header.frame_count)
          throw std::invalid_argument("Geometry read view precedes its poses");
        groups = groups_;
      } catch (...) {
        pending_->header.status = GeometryMapStatus::kError;
        pending_->header.error = ErrorText(std::current_exception());
        pending_->poses.reset();
        view.reset();
        terminal_pending_ = true;
        latest_poses_.reset();
        std::vector<OnlineSubmap>().swap(groups_);
      }
    }
    task_.emplace(
        Task{std::move(*pending_), std::move(groups), std::move(view)});
    pending_.reset();
    busy_ = true;
    ++stats_.submitted;
    stats_.peak_work_payloads = 1;
    stats_.max_poll_seconds = std::max(stats_.max_poll_seconds, Seconds(start));
    condition_.notify_all();
  } catch (...) {
    RecordFailure(ErrorText(std::current_exception()));
    pending_.reset();
  }
}

void GeometryMapOutput::RecordFailure(const std::string& error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (failed_) return;
  failed_ = true;
  ++stats_.failed;
  stats_.error = error.substr(0, 1024);
  LOG(ERROR) << "livo_atlas geometry map output: " << stats_.error;
}

void GeometryMapOutput::Run() {
  for (;;) {
    std::optional<Task> task;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [&] { return stopping_.load() || task_; });
      if (stopping_) return;
      task = std::move(task_);
      task_.reset();
    }
    const auto start = Clock::now();
    auto result = std::move(task->source.header);
    std::uint64_t reads = 0;
    try {
      if (result.status == GeometryMapStatus::kReady) {
        view_ = std::move(task->view);
        double timestamp = 0;
        if (result.frame_count) {
          const auto id = result.frame_count - 1;
          if (!timestamp_ || timestamp_->first != id) {
            timestamp_ = std::make_pair(id, view_->ReadTimestamp(id));
            ++reads;
          }
          timestamp = timestamp_->second;
          if (!std::isfinite(timestamp))
            throw std::invalid_argument(
                "Geometry map timestamp must be finite");
        }
        auto built = builder_->Build(task->groups, task->source.poses);
        built.producer_id = std::move(result.producer_id);
        built.revision = result.revision;
        built.max_keyframes = result.max_keyframes;
        built.timestamp =
            built.status == GeometryMapStatus::kReady ? timestamp : 0;
        result = std::move(built);
      }
    } catch (...) {
      result.status = GeometryMapStatus::kError;
      result.error = ErrorText(std::current_exception());
      result.timestamp = 0;
      result.source_voxel_count = 0;
      result.sampled = false;
      result.sources.clear();
      result.points.clear();
    }
    GeometryMapBuilderStats geometry_stats;
    const bool have_geometry_stats = static_cast<bool>(builder_);
    if (builder_) geometry_stats = builder_->GetStats();
    if (result.status != GeometryMapStatus::kReady) {
      builder_.reset();
      view_.reset();
      geometry_stats.cached_points = 0;
      geometry_stats.geometry.resident_submaps = 0;
      geometry_stats.geometry.resident_voxels = 0;
    }
    bool delivered = false;
    try {
      if (!stopping_) {
        consumer_(result);
        delivered = true;
      }
    } catch (...) {
      RecordFailure(ErrorText(std::current_exception()));
    }
    if (result.status == GeometryMapStatus::kError && !stopping_)
      RecordFailure(result.error);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (have_geometry_stats) stats_.geometry = geometry_stats;
      stats_.timestamp_reads += reads;
      stats_.max_worker_seconds =
          std::max(stats_.max_worker_seconds, Seconds(start));
      if (delivered) {
        ++stats_.delivered;
        stats_.resource_limited +=
            result.status == GeometryMapStatus::kResourceLimited;
        stats_.last_revision = result.revision;
        stats_.last_pose_version = result.pose_version;
        stats_.last_frame_count = result.frame_count;
        stats_.last_optimized_frame_count = result.optimized_frame_count;
        stats_.last_submap_count = result.submap_count;
        stats_.last_source_voxel_count = result.source_voxel_count;
        stats_.last_point_count = result.points.size();
        stats_.last_latency_seconds = Seconds(task->source.observed_at);
        last_delivery_ = Clock::now();
        have_delivery_ = true;
        nonready_delivered_ =
            nonready_delivered_ || result.status != GeometryMapStatus::kReady;
      }
      busy_ = false;
    }
    condition_.notify_all();
  }
}

void GeometryMapOutput::Finish() {
  if (finished_) return;
  if (options_.enabled) {
    if (!GetStats().failed && !terminal_pending_) {
      OnlineSubmapUpdate final;
      final.poses = system_.GetPoseSnapshot();
      Observe(final);
    }
    for (;;) {
      Poll(true);
      std::unique_lock<std::mutex> lock(mutex_);
      if (failed_ || (!busy_ && !pending_)) break;
      condition_.wait(lock, [&] { return !busy_; });
    }
  }
  Stop();
  finished_ = true;
  std::lock_guard<std::mutex> lock(mutex_);
  stats_.finished = true;
}

void GeometryMapOutput::Stop() {
  {
    // Change the wait condition under its mutex to avoid a lost wakeup.
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  condition_.notify_all();
  if (thread_.joinable()) thread_.join();
  pending_.reset();
}

GeometryMapOutputStats GeometryMapOutput::GetStats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

}  // namespace livo_atlas
