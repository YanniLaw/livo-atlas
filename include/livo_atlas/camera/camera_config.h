#ifndef LIVO_ATLAS_CAMERA_CAMERA_CONFIG_H_
#define LIVO_ATLAS_CAMERA_CAMERA_CONFIG_H_

#include <array>
#include <string>

namespace livo_atlas {

// Intrinsics and dimensions describe the original image. The camera applies
// scale exactly once; positive scaled dimensions are truncated to integers.
struct CameraConfig {
  std::string model;
  int width = 0;
  int height = 0;
  double scale = 1.0;
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  // Pinhole: radial k1, k2, tangential p1, p2. Equidistant: k1, k2, k3, k4.
  std::array<double, 4> distortion{};
};

// An empty error string denotes success. This does not modify calibration.
std::string ValidateCameraConfig(const CameraConfig& config);

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_CAMERA_CAMERA_CONFIG_H_
