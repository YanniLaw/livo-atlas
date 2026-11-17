#ifndef LIVO_ATLAS_TESTS_CORE_LOOP_BACKEND_FIXTURES_H_
#define LIVO_ATLAS_TESTS_CORE_LOOP_BACKEND_FIXTURES_H_

#include <exception>
#include <memory>
#include <stdexcept>
#include <utility>

#include "backend/loop_dataset_fixtures.h"
#include "core/fixtures.h"
#include "core/online_submap_manager.h"
#include "livo_atlas/core/loop_backend.h"
#include "livo_atlas/io/online_submap_store.h"

namespace livo_atlas {

// Test-only access: real committed assets and notifications, with a barrier
// around the real runner. Production callers cannot replace admission.
class LoopBackendTestPeer {
 public:
  static void FailCompletedSystem(LivoSystem& system) {
    system.FailLoopBackend(std::make_exception_ptr(
        std::runtime_error("Terminal backend storage failure")));
  }

  static std::unique_ptr<LoopBackend> Create(LivoSystem& system,
                                             LoopAdmissionOptions options,
                                             LoopTaskRunner runner) {
    return std::unique_ptr<LoopBackend>(
        new LoopBackend(system, std::move(options), std::move(runner)));
  }

  static void Append(LivoSystem& system, Keyframe frame) {
    system.CheckInputOpen();
    if (frame.id != system.next_keyframe_id_)
      throw std::invalid_argument("Fixture frame is not the next committed ID");
    auto owned = std::make_shared<const Keyframe>(std::move(frame));
    system.keyframe_store_->Append(*owned);
    system.Online().AppendCommittedKeyframe(std::move(owned));
    ++system.next_keyframe_id_;
  }
};

namespace loop_test {
inline SystemOptions OnlineOptions() {
  auto options = test::Options(FrontendMode::kLidarOnly);
  options.keyframes.enabled = options.submaps.enabled = true;
  options.submaps.max_keyframes = 1;
  options.submap_mode = SubmapMode::kOnline;
  options.online_submaps.support_keyframes = 0;
  return options;
}

inline LoopTask CaptureTask(LivoSystem& system, std::uint64_t query_id,
                            std::vector<AcceptedLoop> history = {}) {
  auto view = system.GetKeyframeReadView();
  KeyframeStore frames(view.directory());
  OnlineSubmapStore store(view.directory() / "online_submaps", frames);
  LoopAdmissionInput input;
  input.query_id = query_id;
  input.poses = *system.GetPoseSnapshot();
  input.accepted_loops = std::move(history);
  std::vector<double> times;
  for (std::uint64_t id = 0; id < view.frame_count(); ++id) {
    input.raw_poses.push_back(view.ReadRawBodyPose(id));
    times.push_back(view.ReadTimestamp(id));
  }
  for (std::uint64_t id = 0; id < store.size(); ++id) {
    auto group = store.Read(id);
    if (group.state != OnlineSubmapState::kSealed) break;
    input.submaps.push_back(
        {group, times.at(group.primary.keyframe_ids.front()),
         times.at(group.primary.keyframe_ids.back()),
         input.poses.poses.at(group.primary.anchor_keyframe_id).map_from_body,
         input.poses.version, input.poses.frame_count,
         input.poses.optimized_frame_count});
  }
  return {1, std::move(input), std::move(times), std::move(view)};
}
}  // namespace loop_test
}  // namespace livo_atlas

#endif  // LIVO_ATLAS_TESTS_CORE_LOOP_BACKEND_FIXTURES_H_
