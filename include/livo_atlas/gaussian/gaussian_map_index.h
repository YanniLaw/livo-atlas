#ifndef LIVO_ATLAS_GAUSSIAN_GAUSSIAN_MAP_INDEX_H_
#define LIVO_ATLAS_GAUSSIAN_GAUSSIAN_MAP_INDEX_H_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "livo_atlas/core/online_submap_update.h"
#include "livo_atlas/gaussian/gaussian_map.h"

namespace livo_atlas {

// Owner-thread metadata only. No sensor assets, filesystem work or GPU state.
class GaussianMapIndex {
 public:
  explicit GaussianMapIndex(std::string session_id);

  void Observe(const OnlineSubmapUpdate& update);
  GaussianTarget MakeTarget(std::uint64_t submap_id,
                            std::uint64_t generation) const;
  bool CanTrain(std::uint64_t submap_id) const;
  void RecordConfirmed(const GaussianTarget& target,
                       GaussianOperation operation, std::uint64_t job_id,
                       const std::vector<std::uint64_t>& frame_ids,
                       int gaussian_count);
  void RecordConfirmed(
      const GaussianTarget& target, GaussianOperation operation,
      std::uint64_t job_id, const std::vector<std::uint64_t>& frame_ids,
      int gaussian_count,
      const std::optional<GaussianRenderMetadata>& render_metadata);
  bool RecordCheckpoint(const GaussianAssetReference& asset);
  // Empty activation releases the previous durable CPU/GPU confirmation.
  void RecordResidentReleased();
  std::optional<std::uint64_t> NextRepair() const;
  void MarkRepairAttempt(std::uint64_t submap_id);
  GaussianMapSnapshot Snapshot() const;

  std::uint64_t sequence() const { return sequence_; }

 private:
  void Reclassify();

  const std::string session_id_;
  std::shared_ptr<const PoseSnapshot> poses_;
  std::vector<GaussianSubmapEntry> entries_;
  std::uint64_t sequence_ = 0;
  std::uint64_t repair_sequence_ = 0;
  std::uint64_t last_asset_id_ = 0;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_GAUSSIAN_GAUSSIAN_MAP_INDEX_H_
