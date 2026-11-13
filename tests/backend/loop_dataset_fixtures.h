#ifndef LIVO_ATLAS_TESTS_BACKEND_LOOP_DATASET_FIXTURES_H_
#define LIVO_ATLAS_TESTS_BACKEND_LOOP_DATASET_FIXTURES_H_

#include <cstdint>

#include "backend/loop_fixtures.h"
#include "livo_atlas/backend/loop_admission.h"
#include "livo_atlas/mapping/keyframe.h"

namespace livo_atlas::loop_test {

inline PointCloud::ConstPtr DenseCloud() {
  PointCloud::Ptr cloud(new PointCloud);
  const auto scene = Scene();
  for (const auto& voxel : scene.data->voxels) {
    const auto n = *voxel.normal;
    const Eigen::Vector3d u =
        n.x() ? Eigen::Vector3d::UnitY() : Eigen::Vector3d::UnitX();
    const Eigen::Vector3d v = n.cross(u);
    for (double a : {-0.04, 0.0, 0.04}) {
      for (double b : {-0.04, 0.0, 0.04}) {
        const Eigen::Vector3d xyz =
            voxel.centroid + Eigen::Vector3d::Constant(0.1) + a * u + b * v;
        Point p;
        p.x = xyz.x();
        p.y = xyz.y();
        p.z = xyz.z();
        p.intensity = 8;
        cloud->push_back(p);
      }
    }
  }
  return cloud;
}

inline Keyframe DenseFrame(std::uint64_t id,
                           const PointCloud::ConstPtr& cloud) {
  Keyframe frame;
  frame.id = id;
  frame.timestamp = frame.cloud_timestamp = 10 + id;
  frame.odom_from_body.translation.x() = 0.02 * id;
  frame.cloud = cloud;
  return frame;
}

inline LoopAdmissionOptions NearbyOptions() {
  LoopAdmissionOptions options;
  options.search.min_submap_gap = 1;
  options.search.min_keyframe_gap = 2;
  options.search.min_time_gap_s = 0.5;
  return options;
}

}  // namespace livo_atlas::loop_test

#endif  // LIVO_ATLAS_TESTS_BACKEND_LOOP_DATASET_FIXTURES_H_
