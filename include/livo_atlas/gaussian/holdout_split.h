#ifndef LIVO_ATLAS_GAUSSIAN_HOLDOUT_SPLIT_H_
#define LIVO_ATLAS_GAUSSIAN_HOLDOUT_SPLIT_H_

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "livo_atlas/gaussian/gaussian_evaluation.h"
#include "livo_atlas/mapping/keyframe.h"

namespace livo_atlas {

// Small immutable admission index. The worker additionally verifies the
// canonical manifest digest and the full reconstruction observation bytes.
class GaussianHoldoutSplit {
 public:
  static GaussianHoldoutSplit Read(const std::filesystem::path& path);

  // Validates heldout metadata too; returns whether training is permitted.
  bool ValidateFrame(const Keyframe& frame, std::uint64_t submap_id) const;
  void ValidateTraining(const Keyframe& frame, std::uint64_t submap_id) const;
  const std::string& split_id() const { return split_id_; }

  const std::string& payload() const { return payload_; }

  const std::vector<GaussianEvaluationQuery>& queries() const {
    return queries_;
  }

 private:
  struct Observation {
    std::uint64_t calibration_id = 0, submap_id = 0;
    double state_timestamp = 0, cloud_timestamp = 0, image_timestamp = 0;
    bool train = false;
  };

  std::string split_id_;
  std::string payload_;
  std::vector<Observation> observations_;
  std::vector<GaussianEvaluationQuery> queries_;
};

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_GAUSSIAN_HOLDOUT_SPLIT_H_
