#ifndef LIVO_ATLAS_SRC_CORE_ONLINE_SUBMAP_MANAGER_H_
#define LIVO_ATLAS_SRC_CORE_ONLINE_SUBMAP_MANAGER_H_

#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/io/online_submap_store.h"
namespace livo_atlas {
class OnlineSubmapManager {
 public:
  OnlineSubmapManager(const std::filesystem::path& directory,
                      KeyframeStore& keyframes, int max_keyframes,
                      OnlineSubmapOptions options);
  void AppendCommittedKeyframe(std::shared_ptr<const Keyframe> keyframe);
  void SetCallback(LivoSystem::OnlineSubmapCallback callback);
  std::shared_ptr<const PoseSnapshot> GetPoseSnapshot() const;
  void ApplyPoseUpdate(const PoseUpdate& update);
  void Finish();
  void CheckRunning() const;
  void RethrowFailure() const;

 private:
  void Notify(OnlineSubmapUpdate update);
  OnlineSubmapBuilder builder_;
  OnlineSubmapStore store_;
  std::vector<RigidTransform> raw_poses_;
  PoseSnapshot frozen_;
  LivoSystem::OnlineSubmapCallback callback_;
  std::exception_ptr failure_;
  bool finished_ = false;
};
}  // namespace livo_atlas
#endif  // LIVO_ATLAS_SRC_CORE_ONLINE_SUBMAP_MANAGER_H_
