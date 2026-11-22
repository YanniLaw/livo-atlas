#include "livo_atlas/camera/camera_config.h"
#include "livo_atlas/common/logging.h"
#include "scenario.h"

int main(int, char** argv) {
  livo_atlas::InitializeLogging(argv[0]);
  livo_atlas::CameraConfig camera{
      "Pinhole", 32, 24, 1, 10, 10, 16, 12, {}};
  if (!livo_atlas::ValidateCameraConfig(camera).empty()) return 1;
  camera.width = 0;
  if (livo_atlas::ValidateCameraConfig(camera).empty()) return 2;
  for (auto mode : {livo_atlas::FrontendMode::kLidarOnly,
                    livo_atlas::FrontendMode::kLidarInertial,
                    livo_atlas::FrontendMode::kLidarInertialVisual}) {
    if (!livo_atlas::test_data::RunFrontendScenario(mode)) return 3;
  }
  LOG(INFO) << "Separate catkin consumer passed LO/LIO/VIO and validation";
}
