#include <iostream>
#include <string>

#include "livo_atlas/frontend/frontend.h"

namespace {

bool CreateAndDestroyPopulatedFrontend(bool create_map) {
  livo_atlas::FrontendOptions options;
  auto& camera = options.calibration.camera;
  camera.model = "Pinhole";
  camera.width = 512;
  camera.height = 512;
  camera.fx = camera.fy = 200;
  camera.cx = camera.cy = 256;
  livo_atlas::Frontend frontend(options);
  if (!create_map) return true;
  livo_atlas::SynchronizedMeasurements input;
  input.update_kind = livo_atlas::UpdateKind::kLidarOnly;
  input.scan_start_time = 100;
  input.scan_end_time = 100.125;
  for (int i = 0; i < 2; ++i) {
    livo_atlas::Point point{};
    point.x = 2 + i;
    point.curvature = 125 * i;
    input.lidar_cloud->push_back(point);
  }
  input.current_cloud = input.lidar_cloud;
  input.updates.emplace_back();
  input.updates.back().lidar_update_time = 100.125;
  const auto result = frontend.Process(&input);
  return result.updated && result.world_cloud && !result.world_cloud->empty();
}

}  // namespace

// LeakSanitizer supplies the post-destruction assertion and nonzero exit
// status. It also observes allocations made by the unchanged estimator shared
// library.
int main(int argc, char** argv) {
  if (argc > 2 || (argc == 2 && std::string(argv[1]) != "--no-map")) return 2;
  for (int i = 0; i < 4; ++i) {
    if (!CreateAndDestroyPopulatedFrontend(argc == 1)) {
      std::cerr << "Lifecycle check did not create a populated map\n";
      return 1;
    }
  }
  return 0;
}
