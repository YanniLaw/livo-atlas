#include <iostream>

#include "export_scenario.h"
#include "online_submap_scenario.h"
#include "scenario.h"
#include "system_scenario.h"

int main(int argc, char** argv) {
  if (argc != 1) {
    if (argc != 3 || std::string(argv[1]) != "--online-fixtures") return 2;
    try {
      livo_atlas::test_data::WriteOnlineFixtures(argv[2]);
    } catch (const std::exception& error) {
      std::cerr << error.what() << '\n';
      return 1;
    }
    return 0;
  }
  for (auto mode : {livo_atlas::FrontendMode::kLidarOnly,
                    livo_atlas::FrontendMode::kLidarInertial,
                    livo_atlas::FrontendMode::kLidarInertialVisual}) {
    if (!livo_atlas::test_data::RunFrontendScenario(mode)) return 1;
    if (!livo_atlas::test_data::RunSystemScenario(mode)) return 1;
    if (!livo_atlas::test_data::RunSystemScenario(mode, true)) return 1;
  }
  if (!livo_atlas::test_data::RunColmapScenario()) return 1;
  std::cout
      << "Installed core: repeated LO, LIO, tracked VIO and online submaps "
         "passed\n";
}
