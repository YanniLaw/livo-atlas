#ifndef LIVO_ATLAS_TESTS_RUNTIME_FIXTURES_H_
#define LIVO_ATLAS_TESTS_RUNTIME_FIXTURES_H_
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include "livo_atlas/runtime/mapping_session.h"
namespace livo_atlas::test {
inline SessionOptions Options(FrontendMode mode) {
  static int sequence = 0;
  SessionOptions options;
  options.frontend.calibration.camera = {"Pinhole", 512, 512, 1, 200,
                                         200,       256, 256, {}};
  options.frontend.imu.enabled = mode != FrontendMode::kLidarOnly;
  options.frontend.imu.initialization_frames = 1;
  options.frontend.surface_filter_size = 0.1;
  options.frontend.visual.grid_size = 32;
  options.frontend.visual.normal_enabled = false;
  options.image_enabled = mode == FrontendMode::kLidarInertialVisual;
  options.output.directory =
      std::filesystem::temp_directory_path() /
      ("livo_atlas_convergence/tests/session-" +
       std::to_string(
           std::chrono::steady_clock::now().time_since_epoch().count()) +
       "-" + std::to_string(++sequence));
  options.preprocess.point_filter = 1;
  return options;
}
inline LidarScan Scan(double timestamp) {
  std::vector<LivoxPoint> points;
  for (int y = 0; y < 16; ++y)
    for (int x = 0; x < 16; ++x) {
      LivoxPoint point;
      point.x = (x - 7.5) * 0.125;
      point.y = (y - 7.5) * 0.125;
      point.z = 4;
      point.offset_time = static_cast<std::uint32_t>(
          (std::int64_t{y * 16 + x} * 125000000) / 255);
      point.line = x % 6;
      point.tag = 0x10;
      points.push_back(point);
    }
  return {timestamp, std::move(points)};
}
}  // namespace livo_atlas::test
#endif
