#ifndef LIVO_ATLAS_IO_KEYFRAME_READ_VIEW_H_
#define LIVO_ATLAS_IO_KEYFRAME_READ_VIEW_H_

#include <cstdint>
#include <filesystem>

#include "livo_atlas/sensor/extrinsics.h"
#include "livo_atlas/sensor/point_types.h"

namespace livo_atlas {

class KeyframeStore;
struct Calibration;
struct Keyframe;

// Owns a fixed committed prefix and its location, never the writer or index.
// The caller keeps the committed files unchanged and available while reading.
class KeyframeReadView {
 public:
  const std::filesystem::path& directory() const { return directory_; }

  std::uint64_t frame_count() const { return frame_count_; }

  RigidTransform ReadRawBodyPose(std::uint64_t id) const;
  double ReadTimestamp(std::uint64_t id) const;
  PointCloud::ConstPtr ReadBodyCloud(std::uint64_t id,
                                     std::uint64_t max_points) const;
  // Reads only this committed frame, without scanning an appending dataset.
  Keyframe Read(std::uint64_t id, const Calibration& calibration,
                std::uint64_t max_points, std::uint64_t max_pixels) const;

 private:
  friend class KeyframeStore;
  KeyframeReadView(std::filesystem::path directory, std::uint64_t frame_count);
  std::filesystem::path FrameDirectory(std::uint64_t id) const;

  std::filesystem::path directory_;
  std::uint64_t frame_count_;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_IO_KEYFRAME_READ_VIEW_H_
