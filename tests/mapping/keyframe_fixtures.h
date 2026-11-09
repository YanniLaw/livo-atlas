#ifndef LIVO_ATLAS_TESTS_MAPPING_KEYFRAME_FIXTURES_H_
#define LIVO_ATLAS_TESTS_MAPPING_KEYFRAME_FIXTURES_H_

#include <opencv2/core.hpp>

#include "livo_atlas/frontend/frontend_result.h"

namespace livo_atlas::test_data {
inline FrontendResult KeyframeResult(UpdateKind kind = UpdateKind::kVisual) {
  FrontendResult result;
  result.kind = kind;
  result.updated = true;
  result.timestamp = 10.25;
  result.cloud_timestamp = 10.2;
  result.estimate.rot_end << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  result.estimate.pos_end << 1, 2, 3;
  result.estimate.inv_expo_time = .8;
  PointCloud::Ptr cloud(new PointCloud);
  Point point;
  point.x = 1;
  point.y = 3;
  point.z = 3;
  point.intensity = 42;
  cloud->push_back(point);
  result.world_cloud = cloud;
  if (kind == UpdateKind::kVisual) {
    result.image_timestamp = 10.24;
    result.color_image = cv::Mat(4, 6, CV_8UC3, cv::Scalar(7, 8, 9));
    result.camera_from_world_translation << 7, 8, 9;
  }
  return result;
}
}  // namespace livo_atlas::test_data

#endif  // LIVO_ATLAS_TESTS_MAPPING_KEYFRAME_FIXTURES_H_
