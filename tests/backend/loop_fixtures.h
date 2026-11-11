#ifndef LIVO_ATLAS_TESTS_BACKEND_LOOP_FIXTURES_H_
#define LIVO_ATLAS_TESTS_BACKEND_LOOP_FIXTURES_H_

#include <Eigen/Geometry>
#include <cmath>
#include <memory>

#include "livo_atlas/mapping/geometry_submap.h"

namespace livo_atlas::loop_test {

inline RigidTransform Transform() {
  return {(Eigen::AngleAxisd(0.10, Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(-0.06, Eigen::Vector3d::UnitX()))
              .toRotationMatrix(),
          Eigen::Vector3d(0.35, -0.25, 0.2)};
}

inline GeometrySubmap Scene(std::uint64_t id = 0, std::uint64_t anchor = 0,
                            const RigidTransform& target_from_local = {},
                            bool plane_only = false) {
  GeometrySubmap result;
  result.membership.primary = {id, anchor, {anchor}};
  result.membership.membership_revision = 2;
  result.membership.state = OnlineSubmapState::kSealed;
  result.frame_count = 200;
  result.representation_revision = 1;
  auto data = std::make_shared<GeometryData>();
  data->sources.push_back({anchor, {}});
  for (int face = 0; face < (plane_only ? 1 : 3); ++face) {
    for (int a = 0; a < 10; ++a) {
      for (int b = 0; b < 10; ++b) {
        Eigen::Vector3d p, n;
        if (face == 0) {
          p = {2, -4 + 0.4 * a, 0.4 * b};
          n = Eigen::Vector3d::UnitX();
        } else if (face == 1) {
          p = {-4 + 0.4 * a, 3, -2 + 0.4 * b};
          n = Eigen::Vector3d::UnitY();
        } else {
          p = {0.4 * a, 0.4 * b, -3};
          n = Eigen::Vector3d::UnitZ();
        }
        GeometryVoxel voxel;
        voxel.centroid = target_from_local.rotation.transpose() *
                         (p - target_from_local.translation);
        voxel.key = {
            static_cast<std::int64_t>(std::floor(voxel.centroid.x() / 0.2)),
            static_cast<std::int64_t>(std::floor(voxel.centroid.y() / 0.2)),
            static_cast<std::int64_t>(std::floor(voxel.centroid.z() / 0.2))};
        voxel.point_count = 8;
        voxel.normal = target_from_local.rotation.transpose() * n;
        voxel.spatial_covariance =
            0.001 * (Eigen::Matrix3d::Identity() -
                     *voxel.normal * voxel.normal->transpose());
        voxel.keyframe_ids = {anchor};
        data->voxels.push_back(voxel);
      }
    }
  }
  data->input_point_count = data->voxels.size() * 8;
  result.data = std::move(data);
  return result;
}

}  // namespace livo_atlas::loop_test

#endif  // LIVO_ATLAS_TESTS_BACKEND_LOOP_FIXTURES_H_
