#ifndef LIVO_ATLAS_TESTS_TRAJECTORY_FIXTURE_H_
#define LIVO_ATLAS_TESTS_TRAJECTORY_FIXTURE_H_

#include <string>

#include "livo_atlas/mapping/trajectory_snapshot.h"

namespace livo_atlas::test {

inline TrajectorySnapshot TrajectoryFixture() {
  TrajectorySnapshot result;
  result.producer_id = std::string(32, 'a');
  result.revision = 1;
  result.frame_count = 2;
  result.keyframes = {{0, 10.25, {}}, {1, 9.5, {}}};
  result.keyframes[0].parent_from_body.translation = {1, 2, 3};
  result.keyframes[1].parent_from_body.translation = {-4, 6, 9};
  result.keyframes[1].parent_from_body.rotation << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  return result;
}

}  // namespace livo_atlas::test

#endif  // LIVO_ATLAS_TESTS_TRAJECTORY_FIXTURE_H_
