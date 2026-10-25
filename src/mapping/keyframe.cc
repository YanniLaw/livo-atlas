#include "livo_atlas/mapping/keyframe.h"

#include <cmath>
#include <stdexcept>

#include "keyframe_source.h"

namespace livo_atlas {
namespace {
void ValidateCloud(const PointCloud& cloud) {
  if (cloud.empty()) throw std::invalid_argument("Keyframe cloud is empty");
  for (const auto& point : cloud) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || !std::isfinite(point.intensity)) {
      throw std::invalid_argument("Keyframe XYZI must be finite");
    }
  }
}

void Transform(PointCloud* cloud, const RigidTransform& transform) {
  const auto error = ValidateExtrinsic(transform);
  if (!error.empty()) throw std::invalid_argument("Keyframe pose: " + error);
  for (auto& point : *cloud) {
    const Eigen::Vector3d mapped =
        transform.rotation * Eigen::Vector3d(point.x, point.y, point.z) +
        transform.translation;
    point.x = mapped.x();
    point.y = mapped.y();
    point.z = mapped.z();
  }
  ValidateCloud(*cloud);
}
}  // namespace

void ValidateKeyframe(const Keyframe& keyframe) {
  if (!std::isfinite(keyframe.timestamp) ||
      !std::isfinite(keyframe.cloud_timestamp) ||
      !std::isfinite(keyframe.inverse_exposure_time)) {
    throw std::invalid_argument("Keyframe times and exposure must be finite");
  }
  const auto error = ValidateExtrinsic(keyframe.odom_from_body);
  if (!error.empty()) throw std::invalid_argument("Keyframe pose: " + error);
  if (!keyframe.cloud) throw std::invalid_argument("Missing keyframe cloud");
  ValidateCloud(*keyframe.cloud);
  if (keyframe.image_timestamp) {
    if (!std::isfinite(*keyframe.image_timestamp) || keyframe.image.empty() ||
        keyframe.image.type() != CV_8UC3 || !keyframe.camera_from_odom) {
      throw std::invalid_argument("Keyframe image requires BGR data and pose");
    }
    const auto camera_error = ValidateExtrinsic(*keyframe.camera_from_odom);
    if (!camera_error.empty()) {
      throw std::invalid_argument("Keyframe camera pose: " + camera_error);
    }
  } else if (!keyframe.image.empty() || keyframe.camera_from_odom) {
    throw std::invalid_argument("Keyframe image has no timestamp");
  }
}

Keyframe CaptureKeyframe(std::uint64_t id, const FrontendResult& result) {
  auto snapshot = KeyframeView(result);
  snapshot.id = id;
  snapshot.image = snapshot.image.clone();
  PointCloud::Ptr cloud(new PointCloud(*snapshot.cloud));
  const auto& pose = snapshot.odom_from_body;
  RigidTransform body_from_odom;
  body_from_odom.rotation = pose.rotation.transpose();
  body_from_odom.translation = -body_from_odom.rotation * pose.translation;
  Transform(cloud.get(), body_from_odom);
  snapshot.cloud = cloud;
  return snapshot;
}

PointCloud TransformKeyframeCloud(const Keyframe& keyframe,
                                  const RigidTransform& parent_from_body) {
  ValidateKeyframe(keyframe);
  PointCloud cloud(*keyframe.cloud);
  Transform(&cloud, parent_from_body);
  return cloud;
}
}  // namespace livo_atlas
