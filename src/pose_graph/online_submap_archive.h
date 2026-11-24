#ifndef LIVO_ATLAS_SRC_POSE_GRAPH_ONLINE_SUBMAP_ARCHIVE_H_
#define LIVO_ATLAS_SRC_POSE_GRAPH_ONLINE_SUBMAP_ARCHIVE_H_

#include <filesystem>
#include <memory>
#include <optional>
#include <vector>

#include "livo_atlas/core/online_submap_update.h"
#include "livo_atlas/io/online_submap_store.h"
#include "livo_atlas/mapping/map_correction.h"

namespace livo_atlas::pose_graph {

// Owned by the serial pose graph writer. A returned delta describes the
// entire write, including any membership revisions that precede publication.
class OnlineSubmapArchive {
 public:
  OnlineSubmapArchive(const std::filesystem::path& directory,
                      const KeyframeStore& keyframes, int max_keyframes,
                      OnlineSubmapOptions options);

  OnlineSubmapDelta Append(std::shared_ptr<const Keyframe> keyframe);
  std::optional<OnlineSubmapDelta> SealInput();
  PoseSnapshot PreparePoseUpdate(const PoseUpdate& update) const;
  OnlineSubmapDelta WritePoses(std::shared_ptr<const PoseSnapshot> candidate);

  PoseSnapshot MaterializePoses() const;
  const std::vector<RigidTransform>& raw_poses() const { return raw_poses_; }
  std::optional<MapCorrection> correction() const { return correction_; }

 private:
  PoseFrontier GetPoseFrontier() const;

  OnlineSubmapBuilder builder_;
  OnlineSubmapStore store_;
  std::vector<RigidTransform> raw_poses_;
  std::shared_ptr<const PoseSnapshot> frozen_;
  std::optional<MapCorrection> correction_{MapCorrection{}};
};

}  // namespace livo_atlas::pose_graph

#endif  // LIVO_ATLAS_SRC_POSE_GRAPH_ONLINE_SUBMAP_ARCHIVE_H_
