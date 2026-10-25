#ifndef LIVO_ATLAS_IO_KEYFRAME_STORE_H_
#define LIVO_ATLAS_IO_KEYFRAME_STORE_H_

#include <cstdint>
#include <filesystem>
#include <vector>

#include "livo_atlas/frontend/calibration.h"
#include "livo_atlas/mapping/keyframe.h"

namespace livo_atlas {
struct KeyframePose {
  std::uint64_t id = 0;
  RigidTransform map_from_body;
};

// One serial writer per dataset. Reopening never appends or replaces raw
// frames; it can read them and add complete, independently numbered poses.
class KeyframeStore {
 public:
  KeyframeStore(const std::filesystem::path& directory,
                const Calibration& calibration);
  explicit KeyframeStore(const std::filesystem::path& directory);
  KeyframeStore(const KeyframeStore&) = delete;
  KeyframeStore& operator=(const KeyframeStore&) = delete;

  void Append(const Keyframe& keyframe);
  Keyframe Read(std::uint64_t id) const;
  const std::vector<std::uint64_t>& ids() const { return ids_; }
  const Calibration& calibration() const { return calibration_; }
  void WritePoses(std::uint64_t version,
                  const std::vector<KeyframePose>& poses);
  std::vector<KeyframePose> ReadPoses(std::uint64_t version) const;

 private:
  void CheckWritable() const;
  std::filesystem::path directory_;
  Calibration calibration_;
  std::vector<std::uint64_t> ids_;
  bool append_enabled_ = false;
  bool failed_ = false;
};
}  // namespace livo_atlas

#endif  // LIVO_ATLAS_IO_KEYFRAME_STORE_H_
