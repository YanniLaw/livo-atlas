#ifndef LIVO_ATLAS_IO_ONLINE_SUBMAP_STORE_H_
#define LIVO_ATLAS_IO_ONLINE_SUBMAP_STORE_H_
#include <exception>

#include "livo_atlas/io/keyframe_store.h"
#include "livo_atlas/mapping/online_submap.h"
namespace livo_atlas {
// One serial owner; keyframes must outlive this store. Reopening is read-only.
class OnlineSubmapStore {
 public:
  OnlineSubmapStore(const std::filesystem::path& directory,
                    const KeyframeStore& keyframes, int max_keyframes,
                    OnlineSubmapOptions options);
  OnlineSubmapStore(const std::filesystem::path& directory,
                    const KeyframeStore& keyframes);
  OnlineSubmapStore(const OnlineSubmapStore&) = delete;
  OnlineSubmapStore& operator=(const OnlineSubmapStore&) = delete;
  void AppendRevision(const OnlineSubmap& submap);
  OnlineSubmap Read(std::uint64_t id) const;
  std::size_t size() const { return submaps_.size(); }
  std::uint64_t indexed_frame_count() const;
  void WritePoseSnapshot(const PoseSnapshot& snapshot);
  PoseSnapshot ReadPoseSnapshot(std::uint64_t version) const;
  PoseSnapshot MaterializePoseSnapshot(std::uint64_t version,
                                       std::uint64_t frame_count) const;

 private:
  void CheckWritable() const;
  void ValidateNext(const OnlineSubmap& submap) const;
  void Accept(const OnlineSubmap& submap);
  OnlineSubmap ReadRecord(std::uint64_t id, std::uint64_t revision) const;
  PoseSnapshot ReadPoseFile(std::uint64_t version) const;
  std::vector<RigidTransform> RawPoses() const;
  [[noreturn]] void SaveFailure(const std::filesystem::path& path,
                                const std::exception& error);
  std::filesystem::path directory_;
  const KeyframeStore& keyframes_;
  int max_keyframes_ = 0;
  OnlineSubmapOptions options_;
  std::vector<OnlineSubmap> submaps_;
  std::uint64_t pose_version_ = 0;
  std::uint64_t optimized_count_ = 0;
  bool writable_ = false;
  std::exception_ptr failure_;
};
}  // namespace livo_atlas
#endif  // LIVO_ATLAS_IO_ONLINE_SUBMAP_STORE_H_
