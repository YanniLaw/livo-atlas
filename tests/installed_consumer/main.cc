#include <iostream>

#include "runtime_scenario.h"
#include "scenario.h"

int main() {
  for (auto mode : {livo_atlas::FrontendMode::kLidarOnly,
                    livo_atlas::FrontendMode::kLidarInertial,
                    livo_atlas::FrontendMode::kLidarInertialVisual}) {
    if (!livo_atlas::test_data::RunFrontendScenario(mode)) return 1;
    if (!livo_atlas::test_data::RunRuntimeScenario(mode)) return 1;
  }
  std::cout << "Installed core and runtime: repeated LO, LIO and tracked VIO "
               "passed\n";
}
