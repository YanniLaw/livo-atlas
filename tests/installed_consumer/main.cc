#include <iostream>

#include "scenario.h"

int main() {
  for (auto mode : {livo_atlas::FrontendMode::kLidarOnly,
                    livo_atlas::FrontendMode::kLidarInertial,
                    livo_atlas::FrontendMode::kLidarInertialVisual}) {
    if (!livo_atlas::test_data::RunFrontendScenario(mode)) return 1;
  }
  std::cout << "Installed core: repeated LO, LIO and tracked VIO passed\n";
}
