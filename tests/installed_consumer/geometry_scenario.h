#ifndef LIVO_ATLAS_TESTS_INSTALLED_CONSUMER_GEOMETRY_SCENARIO_H_
#define LIVO_ATLAS_TESTS_INSTALLED_CONSUMER_GEOMETRY_SCENARIO_H_

#include <memory>
#include <stdexcept>

#include "livo_atlas/mapping/geometry_mapper.h"

namespace livo_atlas::test_data {

inline void RunGeometryScenario() {
  GeometryOptions options;
  options.max_resident_submaps = 2;
  PointCloud::Ptr cloud(new PointCloud);
  Point point;
  point.x = point.y = point.z = 0.1f;
  cloud->push_back(point);
  GeometryMapper mapper(options, [&](std::uint64_t) { return cloud; });
  auto poses = std::make_shared<PoseSnapshot>();
  poses->frame_count = 4;
  for (std::uint64_t i = 0; i < 4; ++i) {
    RigidTransform pose;
    pose.translation.x() = i;
    poses->poses.push_back({i, pose});
  }
  OnlineSubmap first, second, third;
  first.primary = {0, 0, {0, 1}};
  first.membership_revision = 3;
  first.state = OnlineSubmapState::kSealed;
  second.primary = {1, 2, {2}};
  second.membership_revision = 2;
  second.state = OnlineSubmapState::kSealed;
  third.primary = {2, 3, {3}};
  mapper.Update({first, second, third}, poses);
  const auto old = mapper.GetSubmap(0);
  mapper.GetSubmap(1);
  mapper.GetSubmap(2);
  const auto reloaded = mapper.GetSubmap(0);
  if (reloaded.representation_revision <= old.representation_revision ||
      mapper.GetStats().resident_submaps > 2) {
    throw std::runtime_error("Geometry cache scenario failed");
  }
  auto corrected = std::make_shared<PoseSnapshot>(*poses);
  corrected->version = 1;
  corrected->optimized_frame_count = 4;
  corrected->parent_frame = PoseFrame::kMap;
  for (auto& pose : corrected->poses) pose.map_from_body.translation.x() += 10;
  mapper.Update({}, corrected);
  const auto rigid = mapper.GetSubmap(0);
  if (rigid.data != reloaded.data || rigid.pose_version != 1) {
    throw std::runtime_error("Geometry rigid correction scenario failed");
  }
  auto relative = std::make_shared<PoseSnapshot>(*corrected);
  relative->version = 2;
  relative->base_version = 1;
  relative->poses[1].map_from_body.translation.x() += 0.1;
  mapper.Update({}, relative);
  if (mapper.GetSubmap(0).representation_revision <=
          rigid.representation_revision ||
      old.data->input_point_count != 2) {
    throw std::runtime_error("Geometry relative correction scenario failed");
  }
}

}  // namespace livo_atlas::test_data

#endif  // LIVO_ATLAS_TESTS_INSTALLED_CONSUMER_GEOMETRY_SCENARIO_H_
