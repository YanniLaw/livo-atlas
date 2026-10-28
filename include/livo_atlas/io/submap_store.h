#ifndef LIVO_ATLAS_IO_SUBMAP_STORE_H_
#define LIVO_ATLAS_IO_SUBMAP_STORE_H_

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <vector>

#include "livo_atlas/io/keyframe_store.h"
#include "livo_atlas/mapping/submap.h"
#include "livo_atlas/sensor/extrinsics.h"
#include "livo_atlas/sensor/point_types.h"

namespace livo_atlas {
struct SubmapCloud {
  std::uint64_t id = 0;
  std::uint64_t pose_version = 0;
  // Odom for version zero, map for a positive version. Cloud is in anchor body.
  RigidTransform parent_from_submap;
  PointCloud cloud;
};

// One serial writer. The referenced keyframe store must outlive this object.
// Reopening reads a committed membership prefix and never appends to it.
class SubmapStore {
 public:
  SubmapStore(const std::filesystem::path& directory,
              const KeyframeStore& keyframes, int max_keyframes);
  SubmapStore(const std::filesystem::path& directory,
              const KeyframeStore& keyframes);
  SubmapStore(const SubmapStore&) = delete;
  SubmapStore& operator=(const SubmapStore&) = delete;

  void Append(const Submap& submap);
  Submap Read(std::uint64_t id) const;
  std::size_t size() const { return submaps_.size(); }
  SubmapCloud ReadCloud(std::uint64_t id, std::uint64_t pose_version = 0) const;

 private:
  void ValidateNext(const Submap& submap) const;
  Submap ReadFile(std::uint64_t id) const;
  std::filesystem::path directory_;
  const KeyframeStore& keyframes_;
  int max_keyframes_ = 0;
  std::vector<Submap> submaps_;
  bool append_enabled_ = false;
  bool failed_ = false;
};
}  // namespace livo_atlas

#endif  // LIVO_ATLAS_IO_SUBMAP_STORE_H_
