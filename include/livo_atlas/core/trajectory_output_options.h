#ifndef LIVO_ATLAS_CORE_TRAJECTORY_OUTPUT_OPTIONS_H_
#define LIVO_ATLAS_CORE_TRAJECTORY_OUTPUT_OPTIONS_H_

namespace livo_atlas {

struct TrajectoryOutputOptions {
  bool enabled = false;
  double max_frequency_hz = 1;
  int max_keyframes = 20000;
};

void ValidateTrajectoryOutputOptions(const TrajectoryOutputOptions& options);

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_CORE_TRAJECTORY_OUTPUT_OPTIONS_H_
