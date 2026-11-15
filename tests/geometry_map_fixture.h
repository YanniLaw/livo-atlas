#ifndef LIVO_ATLAS_TESTS_GEOMETRY_MAP_FIXTURE_H_
#define LIVO_ATLAS_TESTS_GEOMETRY_MAP_FIXTURE_H_

#include <string>

#include "livo_atlas/mapping/geometry_map_snapshot.h"

namespace livo_atlas::test {

inline GeometryMapSnapshot GeometryMapFixture() {
  GeometryMapSnapshot result;
  result.producer_id = std::string(32, 'a');
  result.revision = 1;
  result.frame_count = 3;
  result.submap_count = 2;
  result.timestamp = 9.5;
  result.max_keyframes = 20;
  result.options.max_points = 3;
  result.effective_max_points = 3;
  result.source_voxel_count = 5;
  result.sampled = true;
  GeometryMapSourceInfo first;
  first.submap_id = 0;
  first.anchor_keyframe_id = 0;
  first.first_keyframe_id = 0;
  first.primary_keyframe_count = 2;
  first.membership_revision = 3;
  first.state = OnlineSubmapState::kSealed;
  first.representation_revision = 1;
  first.source_voxel_count = 3;
  first.input_point_count = 10;
  first.point_count = 2;
  GeometryMapSourceInfo second;
  second.submap_id = 1;
  second.anchor_keyframe_id = 2;
  second.first_keyframe_id = 2;
  second.primary_keyframe_count = 1;
  second.membership_revision = 1;
  second.representation_revision = 2;
  second.source_voxel_count = 2;
  second.input_point_count = 8;
  second.point_offset = 2;
  second.point_count = 1;
  result.sources = {first, second};
  result.points = {
      {{1.25, -2.5, 3.75}, 4}, {{-5.5, 6.25, 7.5}, 8}, {{9, 10, 11}, 12}};
  return result;
}

}  // namespace livo_atlas::test

#endif  // LIVO_ATLAS_TESTS_GEOMETRY_MAP_FIXTURE_H_
