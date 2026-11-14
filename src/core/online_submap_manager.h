#ifndef LIVO_ATLAS_SRC_CORE_ONLINE_SUBMAP_MANAGER_H_
#define LIVO_ATLAS_SRC_CORE_ONLINE_SUBMAP_MANAGER_H_

#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/io/online_submap_store.h"
#include "livo_atlas/mapping/geometry_mapper.h"

namespace livo_atlas {
class OnlineSubmapManager {
 public:
  OnlineSubmapManager(const std::filesystem::path& directory,
                      KeyframeStore& keyframes, int max_keyframes,
                      OnlineSubmapOptions options, GeometryOptions geometry);
  void AppendCommittedKeyframe(std::shared_ptr<const Keyframe> keyframe);
  void SetCallback(LivoSystem::OnlineSubmapCallback callback);
  std::shared_ptr<const PoseSnapshot> GetPoseSnapshot() const;
  MapCorrection GetMapCorrection() const;
  void ApplyPoseUpdate(const PoseUpdate& update);
  GeometrySubmap GetGeometrySubmap(std::uint64_t id);
  GeometryCacheStats GetGeometryStats() const;
  void SealInput();
  void Finish();
  void CheckRunning() const;
  void RethrowFailure() const;

 private:
  void Notify(OnlineSubmapUpdate update);
  OnlineSubmapBuilder builder_;
  OnlineSubmapStore store_;
  std::unique_ptr<GeometryMapper> geometry_;
  std::vector<RigidTransform> raw_poses_;
  PoseSnapshot frozen_;
  LivoSystem::OnlineSubmapCallback callback_;
  std::exception_ptr failure_;
  bool input_sealed_ = false;
  bool finished_ = false;
};
}  // namespace livo_atlas
#endif  // LIVO_ATLAS_SRC_CORE_ONLINE_SUBMAP_MANAGER_H_
