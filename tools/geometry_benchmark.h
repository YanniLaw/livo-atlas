#ifndef LIVO_ATLAS_TOOLS_GEOMETRY_BENCHMARK_H_
#define LIVO_ATLAS_TOOLS_GEOMETRY_BENCHMARK_H_

#include <string>
#include <vector>

namespace livo_atlas {

// Tool-only entry shared with its integration tests; not an SDK interface.
int RunGeometryBenchmark(const std::vector<std::string>& arguments);

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_TOOLS_GEOMETRY_BENCHMARK_H_
