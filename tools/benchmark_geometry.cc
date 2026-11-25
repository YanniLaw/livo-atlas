#include <string>
#include <vector>

#include "geometry_benchmark.h"
#include "livo_atlas/common/logging.h"

int main(int argc, char** argv) {
  livo_atlas::InitializeLogging(argv[0]);
  return livo_atlas::RunGeometryBenchmark(
      std::vector<std::string>(argv + 1, argv + argc));
}
